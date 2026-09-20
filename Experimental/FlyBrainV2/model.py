from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import mlx.core as mx
import mlx.nn as nn
from mlx.utils import tree_flatten

AXES = 3
INPUT_DIM = 22
ENCODER_DIM = 128
HIDDEN_DIM = 96
HEAD_DIM = 128
BOTTLENECK_DIM = 64
RESIDUAL_LIMIT = 0.35


class FlyBrainV2(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.encoder = nn.Linear(INPUT_DIM, ENCODER_DIM)
        self.gru = nn.GRU(ENCODER_DIM, HIDDEN_DIM)
        self.head = nn.Linear(HIDDEN_DIM, HEAD_DIM)
        self.bottleneck = nn.Linear(HEAD_DIM, BOTTLENECK_DIM)
        self.output = nn.Linear(BOTTLENECK_DIM, AXES)

    def __call__(self, x: mx.array, hidden: Optional[mx.array] = None):
        z = nn.silu(self.encoder(x))
        sequence = self.gru(z[:, None, :], hidden)
        next_hidden = sequence[:, -1, :]
        y = nn.silu(self.head(next_hidden))
        y = nn.silu(self.bottleneck(y))
        residual = RESIDUAL_LIMIT * mx.tanh(self.output(y))
        return residual, next_hidden

    def parameter_count(self) -> int:
        total = 0
        for _, value in tree_flatten(self.parameters()):
            total += int(value.size)
        return total


@dataclass
class Scenario:
    angle0: mx.array
    rate0: mx.array
    actuator0: mx.array
    authority: mx.array
    authority_hint: mx.array
    actuator_tau: mx.array
    damping: mx.array
    bias: mx.array
    coupling: mx.array
    target_base: mx.array
    target_amp: mx.array
    target_freq: mx.array
    target_phase: mx.array
    target_step: mx.array
    target_switch: mx.array
    target_width: mx.array
    gust_amp: mx.array
    gust_freq: mx.array
    gust_phase: mx.array
    sensor_noise: mx.array
    dt: float
    horizon: int


def _uniform(shape, lo: float, hi: float) -> mx.array:
    return mx.random.uniform(low=lo, high=hi, shape=shape)


def sample_scenario(batch: int, horizon: int, hard: bool = False, dt: float = 0.08) -> Scenario:
    if hard:
        authority = _uniform((batch, AXES), 8.0, 260.0)
        tau = _uniform((batch, AXES), 0.055, 0.85)
        damping = _uniform((batch, AXES), 0.02, 1.05)
        cross = 0.30
        bias_mag = 7.0
        noise = 1.35
        initial_angle = 42.0
        initial_rate = 13.0
        step_mag = 38.0
    else:
        authority = _uniform((batch, AXES), 18.0, 205.0)
        tau = _uniform((batch, AXES), 0.07, 0.58)
        damping = _uniform((batch, AXES), 0.05, 0.82)
        cross = 0.18
        bias_mag = 4.5
        noise = 0.75
        initial_angle = 30.0
        initial_rate = 8.0
        step_mag = 28.0

    authority_error = _uniform((batch, AXES), 0.68 if hard else 0.78, 1.34 if hard else 1.24)
    authority_hint = authority * authority_error

    off = _uniform((batch, 6), -cross, cross)
    ones = mx.ones((batch,))
    row0 = mx.stack([ones, off[:, 0], off[:, 1]], axis=1)
    row1 = mx.stack([off[:, 2], ones, off[:, 3]], axis=1)
    row2 = mx.stack([off[:, 4], off[:, 5], ones], axis=1)
    coupling = mx.stack([row0, row1, row2], axis=1)

    return Scenario(
        angle0=_uniform((batch, AXES), -initial_angle, initial_angle),
        rate0=_uniform((batch, AXES), -initial_rate, initial_rate),
        actuator0=_uniform((batch, AXES), -0.08, 0.08),
        authority=authority,
        authority_hint=authority_hint,
        actuator_tau=tau,
        damping=damping,
        bias=_uniform((batch, AXES), -bias_mag, bias_mag),
        coupling=coupling,
        target_base=_uniform((batch, AXES), -12.0, 12.0),
        target_amp=_uniform((batch, AXES), 2.0, 18.0 if hard else 14.0),
        target_freq=_uniform((batch, AXES), 0.05, 0.28 if hard else 0.22),
        target_phase=_uniform((batch, AXES), -3.14159265, 3.14159265),
        target_step=_uniform((batch, AXES), -step_mag, step_mag),
        target_switch=_uniform((batch, 1), horizon * dt * 0.25, horizon * dt * 0.65),
        target_width=_uniform((batch, 1), 0.22, 0.75),
        gust_amp=_uniform((batch, AXES), 0.0, 8.0 if hard else 4.0),
        gust_freq=_uniform((batch, AXES), 0.15, 0.9),
        gust_phase=_uniform((batch, AXES), -3.14159265, 3.14159265),
        sensor_noise=mx.full((batch, AXES), noise),
        dt=dt,
        horizon=horizon,
    )


def target_state(s: Scenario, step: int):
    t = step * s.dt
    phase = (2.0 * 3.14159265) * s.target_freq * t + s.target_phase
    smooth_switch = mx.tanh((t - s.target_switch) / s.target_width)
    target = s.target_base + s.target_amp * mx.sin(phase) + 0.5 * s.target_step * (1.0 + smooth_switch)
    target_rate = (2.0 * 3.14159265) * s.target_freq * s.target_amp * mx.cos(phase)
    target_rate = target_rate + 0.5 * s.target_step * (1.0 - smooth_switch * smooth_switch) / s.target_width
    return target, target_rate


def deterministic_core(error, measured_rate, measured_accel, authority_hint, previous_command, target_rate, dt):
    desired_rate = 8.0 * mx.tanh((2.5 * error) / 8.0)
    reference_accel = mx.clip((desired_rate - target_rate) / max(dt, 1e-6), -20.0, 20.0)
    desired_accel = 1.5541775 * (desired_rate - measured_rate) + 0.5662493 * reference_accel
    delta = 0.3434180 * (desired_accel - measured_accel) / mx.maximum(authority_hint, 5.0)
    delta = mx.clip(delta, -3.6201671 * dt, 3.6201671 * dt)
    command = mx.clip(previous_command + delta, -1.0, 1.0)
    return command, desired_rate


def make_features(error, rate, accel, command, actuator, authority_hint, desired_rate, dt):
    dt_column = mx.full((error.shape[0], 1), dt)
    features = mx.concatenate(
        [
            error / 40.0,
            rate / 18.0,
            accel / 120.0,
            command,
            actuator,
            authority_hint / 120.0,
            desired_rate / 8.0,
            dt_column,
        ],
        axis=1,
    )
    return mx.clip(features, -4.0, 4.0)


def rollout(model: Optional[FlyBrainV2], s: Scenario, collect_loss: bool = True):
    angle = s.angle0
    rate = s.rate0
    actuator = s.actuator0
    accel = mx.zeros_like(rate)
    command = mx.zeros_like(rate)
    hidden = None

    squared_error = mx.zeros((angle.shape[0],))
    final_abs_error = mx.zeros((angle.shape[0],))
    max_abs_rate = mx.max(mx.abs(rate), axis=1)
    max_abs_error = mx.zeros((angle.shape[0],))
    activity = mx.zeros((angle.shape[0],))
    saturation = mx.zeros((angle.shape[0],))
    loss = mx.array(0.0)

    for step in range(s.horizon):
        target, target_rate = target_state(s, step)
        noise_phase = 0.73 * step + s.gust_phase
        sensed_rate = rate + 0.22 * s.sensor_noise * mx.sin(noise_phase)
        sensed_accel = accel + s.sensor_noise * mx.sin(noise_phase * 1.37 + 0.4)
        error = target - angle

        core, desired_rate = deterministic_core(
            error, sensed_rate, sensed_accel, s.authority_hint, command, target_rate, s.dt
        )
        if model is None:
            next_command = core
        else:
            features = make_features(
                error, sensed_rate, sensed_accel, command, actuator, s.authority_hint, desired_rate, s.dt
            )
            residual, hidden = model(features, hidden)
            next_command = mx.clip(core + residual, -1.0, 1.0)

        command_delta = next_command - command
        actuator = actuator + (s.dt / s.actuator_tau) * (next_command - actuator)
        actuator = mx.clip(actuator, -1.0, 1.0)

        direct_accel = actuator * s.authority
        coupled_accel = mx.einsum("bij,bj->bi", s.coupling, direct_accel)
        gust = s.gust_amp * mx.sin((2.0 * 3.14159265) * s.gust_freq * (step * s.dt) + s.gust_phase)
        accel = coupled_accel - s.damping * rate + s.bias + gust
        rate = rate + s.dt * accel
        angle = angle + s.dt * rate
        command = next_command

        abs_error = mx.max(mx.abs(error), axis=1)
        abs_rate = mx.max(mx.abs(rate), axis=1)
        max_abs_error = mx.maximum(max_abs_error, abs_error)
        max_abs_rate = mx.maximum(max_abs_rate, abs_rate)
        squared_error = squared_error + mx.mean(error * error, axis=1)
        activity = activity + mx.mean(mx.abs(command_delta), axis=1) / s.dt
        saturation = saturation + mx.mean((mx.abs(command) > 0.985).astype(mx.float32), axis=1)

        if step >= s.horizon - max(12, s.horizon // 8):
            final_abs_error = final_abs_error + mx.mean(mx.abs(error), axis=1)

        if collect_loss:
            rate_excess = mx.maximum(mx.abs(rate) - 8.0, 0.0)
            tracking = mx.mean((error / 12.0) ** 2)
            rate_cost = mx.mean((rate / 12.0) ** 2)
            excess_cost = mx.mean((rate_excess / 8.0) ** 2)
            control_cost = mx.mean(command * command)
            slew_cost = mx.mean((command_delta / max(s.dt, 1e-6)) ** 2)
            loss = loss + s.dt * (
                1.00 * tracking + 0.14 * rate_cost + 0.30 * excess_cost + 0.018 * control_cost + 0.005 * slew_cost
            )

    denom = float(s.horizon)
    final_denom = float(max(12, s.horizon // 8))
    metrics = {
        "rms_error": mx.sqrt(squared_error / denom),
        "final_error": final_abs_error / final_denom,
        "max_rate": max_abs_rate,
        "max_error": max_abs_error,
        "activity": activity / denom,
        "saturation": saturation / denom,
    }
    if collect_loss:
        loss = loss / (s.horizon * s.dt)
        loss = loss + 0.18 * mx.mean(metrics["final_error"] ** 2)
    return loss, metrics
