# STS body-flap trim: offline inspection and entry RCS policy

## Scope and authorization

Follow-up to `runs/20261101T1551KST-terrier-oms-orbit-live` (a crash,
not a landing). No KSP RPCs, save loads, unpauses, burns or live flights were
performed. The installed game assemblies and recorded save were inspected
offline. Pre-existing working-tree changes were retained; policy hunks were
staged independently with `git apply --cached`.

The user explicitly selected **allow configured RCS during entry; keep
MM305/flare off**. Commit `094eb30` implements:

- FCS RCS request = `phase == .entry && vehicle.rcsAttitudeControl`.
  Acquisition, HAC, approach, preflare and flare request false, including the
  first terminal command after entry. Orbital pointing ownership is unchanged.
- Configuration documentation now reflects that policy; no new JSON key.
- `supportedTrimAlpha` no longer treats the global orbital/entry RCS switch as
  evidence of aerodynamic trim authority. An identified ceiling still takes
  precedence; otherwise the existing supported provisional subsonic model is
  used where applicable. Outside that support the existing maximum-AoA fallback
  remains **unknown authority**, not a newly validated envelope.
- Existing exclusion of RCS-on samples from aerodynamic trim identification
  was preserved. There is no newly identified RCS torque model or proof that
  the predictor matches an RCS-assisted entry.

Smallest decisive check: verify configured entry enable/disable, immediate
terminal release, and removal of the RCS/trim shortcut without changing plant
capability. Command, run from `swift/KSL-Landing`:

```sh
swift test --filter 'EntryRCSPolicyTests|ControlTests.testPositivePitchAndRightBankCommands'
```

Result: three tests passed, zero failures (debug build, 2026-11-01).
This was a command-policy check, not a trajectory simulation or flight test.
New regression checks are in `EntryRCSPolicyTests.swift`; existing uncommitted
test changes were not swept into the commit.

## Deployment retains control input: confirmed in installed stock code

Inspected with ILSpy CLI 11.2.0.10050, type `ModuleControlSurface`, in the
installed `KSP_x64_Data/Managed/Assembly-CSharp.dll`. The installation root is
`/Applications/.Misc/KSP.app/Contents/SharedSupport/prefix/drive_c/Program Files (x86)/Kerbal Space Program`.
DLL SHA-256:
`d9e42483f25ee80a9c11d6c1c0a0d29b4ec78c1e08d76c971b71580c9cce51e4`.

- `CtrlSurfaceUpdate` dispatches to `FixedCtrlSurfaceUpdate` for a conventional
  surface (`displaceVelocity == false`), otherwise to `RotatingCtrlSurfaceUpdate`.
- Fixed update computes enabled pitch/yaw/roll input, scales it by
  `ctrlSurfaceRange * authorityLimiter * 0.01`, **adds** `currentDeployAngle`
  when deployed, clamps to `+/- 1.5 * ctrlSurfaceRange`, applies
  `deflectionDirection`, and slews the deflection at `actuatorSpeed` (or uses
  the configured exponential interpolation).
- Rotating update also adds deployment to the control deflection before the
  same clamp. Deployment does not, by itself, disable pitch input in either
  branch.
- For fixed surfaces the deployment sign includes `deployInvert`,
  `partDeployInvert`, and mirror/geometry handling. Do not equate a positive
  configured deploy angle or `Inverted == false` with a nose-up moment.
- `authorityLimiter` scales the input contribution, not the added deployment
  offset in these methods. It is not a proportional trim-angle actuator.

Installed `GameData/Benjee10_shuttleOrbiter/Parts/OV_bodyFlap.cfg` specifies
`ctrlSurfaceRange = 20` degrees, `ctrlSurfaceArea = 1`,
`actuatorSpeed = 40`, and `deflectionLiftCoeff = 1.4`.
CFG SHA-256:
`e77dcaeccf8eea49838941ae4c562239cd4d73e2b85eea9f7d46e35b89de8c58`.
Thus this stock deflection clamp is +/-30 degrees. A 20-degree deployment
offset can consume substantial one-direction travel; additive trim is not
free additional bidirectional control authority. These are source-derived
geometry/actuator facts, not measured aerodynamic gains.

Public references (secondary to inspection of the installed binaries):

- [ModuleControlSurface API](https://kspmoddinglibs.github.io/KSPDocsSite/class_module_control_surface.html)
- [kRPC 0.5.4 parts API](https://krpc.github.io/krpc/cpp/api/space-center/parts.html)

## Actual checkpoint and installed RPC caveat

Recorded save: `runs/20261101T1551KST-terrier-oms-orbit-live/checkpoint-72km.sfs`,
SHA-256 `5b8bebbcf6a81dddfea5f6396990bf4a8a33323efd4c46b087f58a6e6a856c31`.
Parsed offline with `ksl.ascent.craft.parse`. VESSEL index 23 is STS,
launch UT `208672.24653151014`, vessel persistent ID `1530902840`.
Its `benjee10.shuttle.bodyFlap` (part persistent ID `1086142554`) has:

```text
ignorePitch=True, ignoreYaw=True, ignoreRoll=True
deploy=False, deployAngle=20, authorityLimiter=100
usesMirrorDeploy=True, mirrorDeploy=False
deployInvert=False, partDeployInvert=False
```

This save therefore supplies no deployed-flap pitch-moment comparison.
Enabling its axes (`fee8974`) and deploying it are distinct interventions.
The disabled-axis flight cannot identify a deployment gain from pitch-stick
response alone. The cited earlier axis-enabled flight establishes a pitch
trim shortfall, not the sign or magnitude of a deployment increment.

The currently installed `GameData/kRPC/kRPC.version` reports **0.6.0**, not
0.5.4. Installed `KRPC.SpaceCenter.dll` SHA-256:
`ef3855f132477d3dd7c3a8fb8aae1fbbe8cca54f23bec5c333c04a91665bd8a4`.
Its `KRPC.SpaceCenter.Services.Parts.ControlSurface` maps:

- `Deployed` directly to `ModuleControlSurface.deploy` (a flag, not proof
  the actuator has reached its requested deflection).
- `Inverted` to `deployInvert`, **not** inversion of normal pitch response.
- `AuthorityLimiter` to `authorityLimiter`.
- Pitch/Roll/YawEnabled to the negation of the corresponding ignore flag.

This installed DLL additionally advertises direct `DeflectionOverride` and
`Deflection` properties, whose documented intent is to bypass normal control
input and restore prior state when released/disconnected. They were not used
or added as a backend dependency. The current installed DLL is not, without
run-time provenance, proof of what the November 1 crash's server exposed.
Any future experimental code must capability-check the actual server.

## Deliberately not implemented / next evidence needed

No deployment schedule, default change, synthetic plant multiplier, or HAC
ranking adjustment was made. Deployment is mechanically compatible with
stick input, but **the useful sign, moment increment, and remaining pitch
authority have not been measured for this STS**. Flying a guessed sign could
worsen the existing saturation.

Before implementing a schedule, obtain matched-state/matched-command force
and moment evidence for undeployed versus each deployment sign, keeping
mass/CoM, attitude, q, Mach, axis state, RCS availability and actuator history
identical and accounting for settling. A live measurement requires explicit
authorization and existing safety gates. Separate total torque from the
deployment increment; do not infer it from reported potential torque alone.
Use the resulting actuator state and moment model consistently in FCS and
predictors. Any experimental schedule must be explicit, opt-in configuration
with backward-compatible decoding (synthesized Codable defaults alone do not
make missing nonoptional JSON keys decodable).

The all-infeasible HAC continuity-cost question remains open: there was no
candidate replay in this follow-up, so north-HAC superiority apart from the
bank-reversal penalty is still unproven. Surface enablement, body-flap trim,
and RCS-assisted entry all remain unvalidated in live flight. Landing
readiness is unknown.

Foundation Memory capability discovery returned no connected Foundation
search/record tool in this session; these findings are recorded here instead.
