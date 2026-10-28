"""Own attitude controller (replaces the kRPC AutoPilot).

Cascaded attitude -> rate -> torque control in the vessel body frame, running in a
background thread on its own kRPC connection at ~25 Hz.

Body frame (kRPC vessel.reference_frame, origin at the centre of mass): x right,
y nose, z belly; the orbiter roof is -z. Calibrated on the orbiter in flight:
control.pitch=+1 rotates about -x (nose toward -z), yaw=+1 about -z, roll=+1
about -y. The frame behaves right-handed and vessel.rotation(nrf) maps body to
inertial vectors.

The torque command uses the live moment of inertia and available torque (gimbals,
surfaces, wheels; RCS only when the caller enables it), and feeds forward the
engines' thrust-line moment about the live centre of mass (canted SSMEs, SRBs),
so the gimbals are pre-positioned instead of waiting for an integrator. A slow
integrator removes what the model misses.
"""

import csv
import math
import threading
import time

import krpc

from .engine_roles import is_oms_engine


def _dot(a, b):
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]


def _cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    n = _norm(a)
    return (0.0, 0.0, 0.0) if n <= 1e-12 else (a[0]/n, a[1]/n, a[2]/n)


def qrot(q, v):
    """Rotate v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    tx = 2.0*(y*v[2]-z*v[1])
    ty = 2.0*(z*v[0]-x*v[2])
    tz = 2.0*(x*v[1]-y*v[0])
    return (v[0]+w*tx+(y*tz-z*ty), v[1]+w*ty+(z*tx-x*tz), v[2]+w*tz+(x*ty-y*tx))


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def rotate_about(v, axis, angle):
    """Right-hand rotation of v about a unit axis (Rodrigues)."""
    c, s = math.cos(angle), math.sin(angle)
    k = axis
    kxv = _cross(k, v)
    kdv = _dot(k, v)
    return tuple(v[i]*c + kxv[i]*s + k[i]*kdv*(1.0-c) for i in range(3))


def engine_moment(vessel, frame, min_thrust=1.0, axis_override=None):
    """Thrust-line moment vector of the running engines about the CoM (body frame).

    axis_override maps an engine-title substring to a measured thrust axis (body
    frame) for engines whose part axis is not their thrust axis (the OMS).
    """
    moment = [0.0, 0.0, 0.0]
    for engine in vessel.parts.engines:
        try:
            if not engine.active:
                continue
            thrust = engine.thrust
            if thrust <= min_thrust:
                continue
            d = None
            for key, axis in (axis_override or {}).items():
                if key.casefold() in engine.part.title.casefold():
                    d = axis
            if d is None:
                if is_oms_engine(engine):
                    continue  # OMS part axis is wrong; skip unless a measured axis is given
                d = engine.part.direction(frame)
            p = engine.part.position(frame)
        except Exception:
            continue
        m = _cross(p, (thrust*d[0], thrust*d[1], thrust*d[2]))
        for i in range(3):
            moment[i] += m[i]
    return tuple(moment)


def rcs_up_translation_moment(vessel, frame):
    """Estimate the body-frame torque for full roofward translation.

    Weight reported nozzle vectors by their roofward component and scale each
    block to its available roofward force. If a mod reports unusable nozzle
    axes, use the block position and available force instead. The game's
    allocation between nozzles is not exposed, so this remains an estimate.
    """
    moment = [0.0, 0.0, 0.0]
    usable = False
    try:
        for block in vessel.parts.rcs:
            if not block.enabled or not block.up_enabled:
                continue
            _, negative = block.available_force
            force = abs(negative[2])
            if not math.isfinite(force):
                return None
            if force <= 0:
                continue
            nozzles = []
            for nozzle in block.thrusters:
                direction = nozzle.thrust_direction(frame)
                if any(not math.isfinite(x) for x in direction):
                    return None
                weight = max(0.0, -direction[2])
                if weight > 1e-6:
                    nozzles.append((nozzle.thrust_position(frame), direction, weight))
            projection = sum(weight*(-direction[2]) for _, direction, weight in nozzles)
            if projection <= 1e-9:
                contribution = _cross(block.part.position(frame), (0.0, 0.0, -force))
                moment = [moment[i]+contribution[i] for i in range(3)]
            else:
                for position, direction, weight in nozzles:
                    vector = tuple(force*weight*x/projection for x in direction)
                    contribution = _cross(position, vector)
                    moment = [moment[i]+contribution[i] for i in range(3)]
            usable = True
    except Exception:
        return None
    if any(not math.isfinite(x) for x in moment):
        return None
    return tuple(moment) if usable else None


class AttitudeController:
    # control input k produces torque along -axis k (calibrated); axis order x, y, z
    INPUTS = (('pitch', 0), ('roll', 1), ('yaw', 2))

    def __init__(self, address='127.0.0.1', rpc_port=50000, stream_port=50001, hz=25.0,
                 telemetry_path=None):
        self.conn = krpc.connect(name='attitude-controller', address=address,
                                 rpc_port=rpc_port, stream_port=stream_port)
        self.sc = self.conn.space_center
        self.vessel = self.sc.active_vessel
        self.control = self.vessel.control
        self.frame = self.vessel.reference_frame
        self.nrf = self.vessel.orbit.body.non_rotating_reference_frame
        self._rotation = self.conn.add_stream(self.vessel.rotation, self.nrf)
        self._angvel = self.conn.add_stream(self.vessel.angular_velocity, self.nrf)
        self._ut = self.conn.add_stream(getattr, self.sc, 'ut')
        self.dt = 1.0/hz
        self.lock = threading.Lock()
        self.forward = None        # target nose direction, inertial (nrf)
        self.roof = None           # target roof (-z) direction, inertial; None = roll free
        self._hold_until = None
        self._held_attitude = None
        self.engaged = False
        # tuning
        self.tau = (2.0, 2.0, 2.0)         # attitude time constant per axis (x, y, z), s
        self.rate_tau = 0.6                # rate-loop time constant, s
        self.max_rate = tuple(math.radians(x) for x in (4.0, 8.0, 4.0))
        self.int_time = 6.0
        self.int_limit = 0.35              # fraction of available torque
        self.feedforward = True
        self.axis_override = {}
        self.deadband = 0.0
        # Fraction of the reported torque trusted for braking. available_torque
        # overstates roll (in-line SSME gimbals give little roll), so roll brakes early.
        self.brake_fraction = (0.6, 0.25, 0.6)
        self.roll_authority_fraction = 1.0
        self.roll_input_limit = 1.0
        self.roll_gate = (8.0, 5.0)        # nose error (deg) above which roll is suppressed / fully on
        # telemetry
        self.error_deg = 0.0
        self.roll_error_deg = 0.0
        self.inputs = (0.0, 0.0, 0.0)
        self.disturbance = (0.0, 0.0, 0.0)
        self.translation_moment = (0.0, 0.0, 0.0)
        self._integral = [0.0, 0.0, 0.0]
        self._moi = (1.0, 1.0, 1.0)
        self._torque = (1.0, 1.0, 1.0)
        self._last_slow = 0.0
        self._dynamics_dirty = threading.Event()
        self._telemetry_file = open(telemetry_path, 'w', newline='', buffering=1) if telemetry_path else None
        self._telemetry = csv.writer(self._telemetry_file) if self._telemetry_file else None
        if self._telemetry:
            self._telemetry.writerow(['ut', 'qx', 'qy', 'qz', 'qw', 'wx', 'wy', 'wz',
                                      'target_fx', 'target_fy', 'target_fz',
                                      'target_rx', 'target_ry', 'target_rz',
                                      'pitch', 'roll', 'yaw', 'mx', 'my', 'mz',
                                      'tx', 'ty', 'tz', 'ix', 'iy', 'iz'])
        self._running = True
        self._thread = threading.Thread(target=self._run, name='attitude', daemon=True)
        self._thread.start()

    def set_target(self, forward, roof=None):
        with self.lock:
            self.forward = _unit(forward)
            self.roof = None if roof is None else _unit(roof)

    def hold_attitude(self, until_ut, preserve_target_roof=False):
        """Hold the measured nose and either the measured or commanded roof."""
        rotation = self._rotation()
        forward = qrot(rotation, (0., 1., 0.))
        measured_roof = qrot(rotation, (0., 0., -1.))
        with self.lock:
            roof = self.roof if preserve_target_roof and self.roof is not None else measured_roof
            attitude = (forward, roof)
            self._held_attitude = attitude
            self._hold_until = until_ut
        return attitude[0]

    def extend_attitude_hold(self, until_ut):
        """Keep the already captured attitude through the actual staging time."""
        with self.lock:
            if self._held_attitude is None:
                return False
            self._hold_until = until_ut
            return True

    def hold_commanded_attitude(self, until_ut):
        """Freeze the current inertial target while feedback damps the vehicle."""
        with self.lock:
            if self.forward is None:
                return False
            self._held_attitude = (self.forward, self.roof)
            self._hold_until = until_ut
            return True

    def release_attitude_hold(self):
        with self.lock:
            self._hold_until = None
            self._held_attitude = None

    def _target_for_step(self):
        with self.lock:
            if self._hold_until is not None:
                if self._ut() < self._hold_until:
                    return self._held_attitude
                self._hold_until = None
                self._held_attitude = None
            return self.forward, self.roof

    def reset_integral(self):
        self._integral = [0.0, 0.0, 0.0]

    def invalidate_dynamics(self):
        """Request a topology refresh on the control thread before its next step."""
        self._dynamics_dirty.set()

    def _update_dynamics(self, now):
        dirty = self._dynamics_dirty.is_set()
        if dirty or now-self._last_slow > 0.4:
            if dirty:
                # Clear before reading so an invalidation during RPC reads is
                # retained for the following step too.
                self._dynamics_dirty.clear()
                self.reset_integral()
            self._refresh_slow()
            self._last_slow = now

    def stop(self):
        self._running = False
        self._thread.join(timeout=2.0)
        if self._telemetry_file and not self._thread.is_alive():
            self._telemetry_file.close()
        try:
            self.control.pitch = self.control.yaw = self.control.roll = 0.0
        except Exception:
            pass

    def _refresh_slow(self):
        try:
            self._moi = tuple(max(1.0, x) for x in self.vessel.moment_of_inertia)
            positive, negative = self.vessel.available_torque
            self._torque = tuple(max(1.0, min(abs(p), abs(n))) for p, n in zip(positive, negative))
            self.disturbance = (engine_moment(self.vessel, self.frame,
                                              axis_override=self.axis_override)
                                if self.feedforward else (0.0, 0.0, 0.0))
        except Exception:
            pass

    def _run(self):
        idle_written = False
        while self._running:
            t0 = time.monotonic()
            try:
                if not self.engaged or self.forward is None:
                    if not idle_written:
                        self.control.pitch = self.control.yaw = self.control.roll = 0.0
                        idle_written = True
                    self.reset_integral()
                    time.sleep(self.dt)
                    continue
                idle_written = False
                self._update_dynamics(t0)
                self._step()
            except Exception:
                pass
            time.sleep(max(0.0, self.dt-(time.monotonic()-t0)))

    def _step(self):
        forward, roof = self._target_for_step()
        q = self._rotation()
        qi = qconj(q)
        w = qrot(qi, self._angvel())
        f = _unit(qrot(qi, forward))
        nose = (0.0, 1.0, 0.0)
        axis = _cross(nose, f)
        s = _norm(axis)
        angle = math.atan2(s, _dot(nose, f))
        if s > 1e-9:
            phi = [axis[i]/s*angle for i in range(3)]
        elif angle > 1.0:
            phi = [angle, 0.0, 0.0]
        else:
            phi = [0.0, 0.0, 0.0]
        self.error_deg = math.degrees(angle)
        roll_on = 0.0
        if roof is not None:
            u = qrot(qi, roof)
            u = (u[0], 0.0, u[2])
            if _norm(u) > 1e-6:
                cur = (0.0, 0.0, -1.0)
                roll_angle = math.atan2(_dot(nose, _cross(cur, u)), _dot(cur, _unit(u)))
                start, full = self.roll_gate
                roll_on = max(0.0, min(1.0, (start-self.error_deg)/max(1e-3, start-full)))
                phi[1] = roll_angle*roll_on
                self.roll_error_deg = math.degrees(roll_angle)
        # Time-optimal-ish slew: fast while far, but never faster than the speed
        # that can still be braked to zero over the remaining angle (alpha = T/I).
        w_des = []
        for i in range(3):
            alpha = self._torque[i]/self._moi[i]
            brake = math.sqrt(2.0*self.brake_fraction[i]*alpha*abs(phi[i]))
            mag = min(self.max_rate[i], abs(phi[i])/self.tau[i], brake)
            w_des.append(math.copysign(mag, phi[i]))
        torque_cmd = [0.0, 0.0, 0.0]
        inputs = [0.0, 0.0, 0.0]
        for i in range(3):
            err = w_des[i]-w[i]
            t_avail = self._torque[i] * (self.roll_authority_fraction if i == 1 else 1.0)
            cmd = (self._moi[i]*err/self.rate_tau + self._integral[i]
                   - self.disturbance[i] - self.translation_moment[i])
            torque_cmd[i] = cmd
            u = -cmd/t_avail
            limit = self.roll_input_limit if i == 1 else 1.0
            saturated = abs(u) >= limit
            if not saturated or (u > 0) != (err*-1 > 0):
                lim = self.int_limit*t_avail
                self._integral[i] = max(-lim, min(lim, self._integral[i] +
                                                  self._moi[i]*err/self.int_time*self.dt))
            u = max(-limit, min(limit, u))
            # small deadband: holding attitude must not drain the wheels' power
            inputs[i] = 0.0 if abs(u) < self.deadband else u
        self.control.pitch = inputs[0]
        self.control.roll = inputs[1]
        self.control.yaw = inputs[2]
        self.inputs = tuple(inputs)
        if self._telemetry:
            try:
                self._telemetry.writerow([self._ut(), *q, *w, *forward,
                                          *(roof if roof is not None else (None,)*3),
                                          *inputs, *self.disturbance, *self._torque, *self._moi])
            except OSError:
                # Logging must not prevent subsequent flight-control steps.
                self._telemetry = None


class CustomAutopilot:
    """Drop-in for the subset of the kRPC AutoPilot API this project uses."""

    _OWN = {'_conn', '_vessel', 'ctrl', 'reference_frame', '_direction', '_up', '_roll'}

    def __init__(self, conn, vessel, **kwargs):
        object.__setattr__(self, '_conn', conn)
        object.__setattr__(self, '_vessel', vessel)
        object.__setattr__(self, 'ctrl', AttitudeController(**kwargs))
        object.__setattr__(self, 'reference_frame', vessel.surface_reference_frame)
        object.__setattr__(self, '_direction', None)
        object.__setattr__(self, '_up', None)
        object.__setattr__(self, '_roll', float('nan'))

    # --- target API ---------------------------------------------------------------
    def _push(self):
        if self._direction is None:
            return
        sc = self._conn.space_center
        nrf = self.ctrl.nrf_main if hasattr(self.ctrl, 'nrf_main') else \
            self._vessel.orbit.body.non_rotating_reference_frame
        f = _unit(sc.transform_direction(self._direction, self.reference_frame, nrf))
        roof = None
        if self._up is not None:
            up = sc.transform_direction(self._up, self.reference_frame, nrf)
            up = tuple(up[i]-f[i]*_dot(up, f) for i in range(3))
            if _norm(up) > 1e-9:
                roll = 0.0 if math.isnan(self._roll) else self._roll
                # inverse of flight.roll_for_roof: roll = -(signed angle up -> roof)
                roof = rotate_about(_unit(up), f, -math.radians(roll))
        self.ctrl.set_target(f, roof)

    def set_direction_and_up(self, direction, up, roll=0.0):
        object.__setattr__(self, '_direction', tuple(direction))
        object.__setattr__(self, '_up', tuple(up))
        object.__setattr__(self, '_roll', float(roll))
        self._push()

    @property
    def target_direction(self):
        return self._direction

    @target_direction.setter
    def target_direction(self, value):
        object.__setattr__(self, '_direction', tuple(value))
        object.__setattr__(self, '_up', None)
        self._push()

    @property
    def target_roll(self):
        return self._roll

    @target_roll.setter
    def target_roll(self, value):
        object.__setattr__(self, '_roll', float(value))

    @property
    def engaged(self):
        return self.ctrl.engaged

    @engaged.setter
    def engaged(self, value):
        self.ctrl.engaged = bool(value)

    def engage(self):
        self.ctrl.engaged = True

    def disengage(self):
        self.ctrl.engaged = False

    @property
    def error(self):
        return self.ctrl.error_deg

    @property
    def roll_error(self):
        return self.ctrl.roll_error_deg

    @property
    def roll_control_oscillation(self):
        return 0.0

    # --- tuning knobs mapped from the AutoPilot vocabulary --------------------------
    @property
    def time_to_peak(self):
        return self.ctrl.tau

    @time_to_peak.setter
    def time_to_peak(self, value):
        self.ctrl.tau = tuple(max(1.5, float(x)) for x in (value[0], value[1], value[2]))

    @property
    def max_angular_velocity(self):
        return self.ctrl.max_rate

    @max_angular_velocity.setter
    def max_angular_velocity(self, value):
        # (pitch, roll, yaw) in rad/s; pitch/yaw are kept gentle for the stack.
        p, r, y = value
        cap = math.radians(5.0)
        self.ctrl.max_rate = (min(p, cap), r, min(y, cap))

    @property
    def roll_start_angle(self):
        return self.ctrl.roll_gate[0]

    @roll_start_angle.setter
    def roll_start_angle(self, value):
        self.ctrl.roll_gate = (float(value), self.ctrl.roll_gate[1])

    @property
    def roll_engage_angle(self):
        return self.ctrl.roll_gate[1]

    @roll_engage_angle.setter
    def roll_engage_angle(self, value):
        self.ctrl.roll_gate = (self.ctrl.roll_gate[0], float(value))

    def __getattr__(self, name):
        # Unused AutoPilot knobs (auto_tune, pid gains, overshoot, ...) read as None.
        return None

    def __setattr__(self, name, value):
        cls_attr = getattr(type(self), name, None)
        if isinstance(cls_attr, property):
            cls_attr.fset(self, value)
        elif name in self._OWN:
            object.__setattr__(self, name, value)
        # other AutoPilot knobs are accepted and ignored
