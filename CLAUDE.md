# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An FTC (FIRST Tech Challenge) robot controller project for team ftc3543 (Titan Robotics Club), built on the official
FIRST-Tech-Challenge/FtcRobotController SDK. It vendors the team's own "Titan Robotics Framework Library" as two git
submodules — `trclib` (generic robotics framework: state machines, PID/pure-pursuit drive, sensors, vision pipeline
plumbing) and `ftclib` (thin FTC-SDK-specific adapters over `trclib`) — so the framework can be updated independently
of both the FTC SDK and the season's team code. This repo serves as the season's starting template: it includes a
working mecanum drive base with TeleOp control out of the box, plus autonomous infrastructure driven by choice menus
(`FtcChoiceMenu`) so one `FtcAuto` OpMode can cover many autonomous permutations (alliance, start position, delay,
strategy) instead of one OpMode per permutation.

This is an Android Studio / Gradle project, not a typical application — there is no `main()`; the FTC Driver Station
app loads OpModes (`FtcAuto`, `FtcTeleOp`, `FtcTest`) annotated with `@Autonomous`/`@TeleOp` and drives their
lifecycle via the FTC SDK event loop.

## Build / test / run

There is no CLI test suite. Building and running only makes sense against physical FTC hardware (a REV Control
Hub) via Android Studio or Gradle:

```bash
./gradlew build              # compile both modules
./gradlew :TeamCode:assembleDebug   # build just the team code APK
./gradlew installDebug        # install to a connected/ADB-reachable Control Hub
```

- Open the project root in Android Studio to get code navigation, Gradle sync, and one-click deploy to a Control
  Hub connected over USB or WiFi Direct/ADB.
- After cloning, submodules must be initialized: `git submodule update --init --recursive` (`trclib`/`ftclib` live
  under `TeamCode/src/main/java/`).
- There is no unit test framework in this repo; validating behavior requires deploying to a robot and running
  OpModes from the Driver Station. If asked to verify a change, say so explicitly rather than claiming it was
  tested — reason about correctness by reading the code (state machine transitions, PID/menu wiring, pin/hardware
  names) instead.
- `TeamCode`'s `FtcTest` OpMode (see below) provides an in-robot diagnostics/tuning menu — that's the closest thing
  to a test harness this project has.

### Module layout

- `FtcRobotController/` — the stock FTC SDK app module (Android library `com.qualcomm.ftcrobotcontroller`). Avoid
  editing; it's regenerated from upstream SDK releases (see version history in `README.md`).
- `TeamCode/` — the only module meant for regular edits. `build.gradle` here is intentionally thin; shared Android
  build config lives in root `build.common.gradle` (SDK versions, signing, NDK ABI filters) and
  `build.dependencies.gradle` (Maven deps: FTC SDK artifacts, FTC Dashboard, acmerobotics dashboard). Prefer editing
  `TeamCode/build.gradle` over the shared files unless intentionally updating shared SDK plumbing.
- `TeamCode/src/main/java/trclib/` and `.../ftclib/` — git submodules (github.com/trc492/trclib,
  github.com/trc492/ftclib). Don't casually edit files here; changes belong upstream in those repos and get pulled
  in via submodule update. Each has an `archive/` subfolder of retired/superseded classes kept for reference.
- `TeamCode/src/main/java/teamcode/` — this season's actual team code; this is where nearly all work happens.

## Architecture (`teamcode` package)

- **`RobotParams.java`** — single source of truth for all tunable constants: feature toggles (`Preferences` — e.g.
  `useVision`, `useDriveBase`, `useLED`; used to compile out/skip subsystems that aren't ready yet during
  development), physical robot dimensions, per-season `Game` constants (AprilTag poses, timing, starting poses),
  field dimensions (`Field`), and Gobilda motor specs. New subsystems add their tunable constants here following the
  existing naming convention (`HWNAME_*` for hardware config names, `*_INVERTED`, `*_KP/KI/KD/KF`, `*_PRESETS`,
  etc.) — see the "Creating Subsystems" walkthrough in `README.md` for the expected shape of a new subsystem file.
- **`Robot.java`** — composition root. Constructs the drive base first (other components depend on
  `robotInfo`/`robotBase`), then sensors/indicators, then vision, then season-specific subsystems and auto tasks —
  each gated by its `RobotParams.Preferences` flag so a subsystem that isn't wired up yet is simply `null` rather
  than crashing the rest of the robot.
- **OpModes** — `FtcTeleOp`, `FtcAuto`, `FtcTest` (each `extends FtcOpMode`, annotated `@TeleOp`/`@Autonomous`) are
  the three entry points the Driver Station can launch. `FtcAuto` builds its choice menus (`AutoChoices`, using
  `FtcChoiceMenu`/`FtcValueMenu`) at init time, then hands off to `autocommands/CmdAuto`. `FtcTest` hosts hardware
  diagnostics/tuning tests (drive motor test, PID tuning, etc.) selectable via the same menu pattern.
- **State-machine command pattern** (`autocommands/`, `trclib/command/`) — autonomous behavior is expressed as
  classes implementing `TrcRobot.RobotCommand` (`start()`, `cancel()`, `isActive()`, `cmdPeriodic(elapsedTime)`)
  driven by a `TrcStateMachine<State>` enum. `CmdAuto` is the top-level command dispatched from `FtcAuto`; season
  auto routines are added as new `State` cases (or new `Cmd*` classes composed together) rather than by writing
  imperative code in the OpMode itself. `autotasks/TaskAuto.java` follows the same shape for autonomous "tasks" that
  can run concurrently with/independent of the main auto command (see `TrcAutoTask` in trclib).
- **`subsystems/DriveBase.java`** — wraps `ftclib.drivebase.FtcRobotBase` to construct the drive base
  (mecanum/differential/swerve per `RobotParams.Preferences.robotType`), wire up odometry (drive-encoder or
  odometry-pod based on `useExternalOdometry`), and expose `RobotInfo`. New non-drive subsystems (arm, intake,
  slide, etc.) go in this package as their own classes, typically wrapping a `FtcMotorActuator`/`FtcServoActuator`
  configured from `RobotParams` constants — follow the pattern shown in the "Creating Subsystems" section of
  `README.md`.
- **`vision/Vision.java`** — wraps AprilTag / color-blob / Limelight vision processors (`ftclib.vision.*`) behind
  `RobotParams.Preferences` flags (`useWebcamAprilTagVision`, `useWebcamColorBlobVision`, `useLimelightVision`,
  etc.), and feeds `TrcVisionRelocalize` when `visionRelocalizeEnabled` is set.
- **`indicators/`** — `LEDIndicator`/`RumbleIndicator` give driver feedback (robot state, alliance, game-piece
  detection) via addressable LEDs or gamepad rumble; both are optional (`useLED`/`useRumble`).

### Conventions specific to this codebase

- Feature-gate new/incomplete subsystems behind a boolean in `RobotParams.Preferences` rather than commenting out
  code or leaving half-wired references — this is the established way to keep the rest of the robot compilable and
  runnable while a given subsystem is still in progress.
- Closing comments on class/method braces (`}   //ClassName`, `}   //methodName`) and Javadoc (`/** ... */` with
  `@param`/`@return`) are used consistently throughout `teamcode`, `trclib`, and `ftclib` — match this style in new
  files.
- Units and coordinate system: distances are in inches, angles in degrees, and `trclib` poses use an ENU
  (East-North-Up) convention with the robot's center of rotation as the origin — X to robot-right, Y to
  robot-forward. Keep new pose/offset constants consistent with this (see the odometry-pod placement discussion in
  `README.md`).
- Don't edit vendored `trclib`/`ftclib` submodule sources in place for team-specific behavior — extend/wrap them
  from `teamcode` instead; submodule changes get lost/conflict on the next `git submodule update`.
- `FtcRobotController/`, `build.common.gradle`, and root `build.gradle` mirror the upstream FTC SDK release and are
  called out in their own headers as files that should rarely be touched — season-specific changes belong in
  `TeamCode/`.
