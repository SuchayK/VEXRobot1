# VEX V5 Competition Robot — Drive, Odometry & Autonomous

Competition code for a VEX V5 robot: six-motor tank drive, pneumatic wings, a lift, and
position tracking off three dedicated encoders plus an inertial sensor. Written for the
VEXcode V5 `competition` template, so the same codebase serves both the autonomous period
and driver control.

The reason this isn't just `spinFor` calls: VEX fields are slippery and matches are 15
seconds of autonomous where a few degrees of heading drift compounds into a missed scoring
element. Most of the code here exists to make the robot end up where it thinks it is.

## Hardware

| Subsystem | Configuration |
|---|---|
| Drivetrain | 6× motors on `ratio6_1` — 3 per side, grouped as `left_drive` / `right_drive` |
| Lift | 1× motor on `ratio18_1` (torque gearing) |
| Pneumatics | `wings` via 3-wire digital out |
| Odometry | 3× quadrature encoders — left, right, and a rear encoder for lateral drift |
| Heading | V5 inertial sensor |
| Wheels | 3.0" diameter, 360 ticks/rev |

## How it works

**Motion with heading correction.** `moveForward` / `moveBack` run a PID loop
(`kP 0.6, kI 0.04, kD 0.3`) on encoder distance, but the drive output is split by a
proportional heading term: `left = power + Kc * angleError`, `right = power - Kc * angleError`,
with `Kc = 0.5` and `angleError` measured against the inertial reading captured at the start
of the move. A straight-line command therefore actively corrects its own drift instead of
accumulating it — one side gets throttled the moment the robot starts to veer. The loop
ticks every 20 ms and exits inside 0.1 turns of target.

**Three-wheel odometry.** `updateOdometry()` maintains a global pose (`robotX`, `robotY`,
`robotTheta`), updated after every movement primitive. The rear encoder is what makes this
more than differential-drive dead reckoning — it captures sideways displacement from
defensive contact, which a two-encoder model silently misses.

**Two movement APIs.** Both a simple blocking form (`turnLeft`, `rightDegrees` — direct
`spinFor` calls, useful for quick tuning) and the PID form. Overloads taking a trailing
`bool` control whether the call blocks, which is how two subsystems get driven at once.

## Layout

| File | Role |
|---|---|
| `main.cpp` | Movement primitives, PID, odometry, autonomous + driver control |
| `robot-config.cpp` / `.h` | Device constructors, ports, motor groups |
| `vex.h` | VEXcode includes |
| `*.png` | CAD renders and 360° views of the build |

## Status

The movement, PID, odometry, and driver-control code are complete and were run on the robot.
The `autonomous()` routine currently holds a **commented-out route** — a sequence of
`moveForward` / `leftDegrees` / `wings.set()` calls kept as a scratchpad, since the route was
re-tuned per field setup and per match. Uncomment and adjust the distances for a given field
before competing; the primitives it calls are working.

## Running it

Open in **VEXcode V5** (or VS Code with the VEX extension), match the ports in
`robot-config.cpp` to your build, then build and download to the V5 brain. Calibrate the
inertial sensor on a still robot at startup — heading correction is only as good as that
initial reading.
