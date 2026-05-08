# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Deploy Commands

```bash
# Build the project
./gradlew build

# Deploy to the robot (requires network connection to RoboRIO)
./gradlew deploy

# Run tests (tests are disabled by default; must explicitly invoke)
./gradlew test --tests "*"

# Run a specific test class
./gradlew test --tests "frc.robot.RobotParametersTest"

# Run simulation
./gradlew simulatejava
```

Tests only run when the `test` task is explicitly on the command line. Omitting `--tests` with `test` task will skip all tests without error.

## Architecture Overview

This is a **WPILib 2026 command-based Java robot** for FRC Team 3620. Java 17, built with GradleRIO 2026.2.1.

### Entry Points

- `Robot.java` — `TimedRobot` that bootstraps logging (DogLog + TinyLog), creates `RobotContainer`, and drives the `CommandScheduler` each periodic loop
- `RobotContainer.java` — instantiates all subsystems, joysticks, button bindings, PathPlanner autos, and the FSM state machine

### Multi-Robot Variant Support

Three chassis variants share the same code via different `TunerConstants` classes in `src/main/java/frc/robot/Generated/`:
- `ChudbotTunerConstants` — primary competition robot
- `RaptorTunerConstants` — alternate chassis
- `TunerConstants` — default/fallback

The active variant is selected at runtime via `RobotParametersContainer`, which reads a JSON config file keyed on robot identity (e.g., `"chudbot"`, `"raptor"`). Robot-specific hardware options (non-critical CAN devices, breaker ignore lists) live in `RobotParameters.java`.

### Subsystems (`src/main/java/frc/robot/Subsystems/`)

All subsystems extend `SubsystemBase`. Key patterns:
- Motors are **CTRE TalonFX** (Phoenix 6) or **REV SparkMax**
- The **YAMS library** wraps motor control for mechanisms (flywheels, arms, gearing)
- `SmartMotorController` provides a unified control layer
- `SwerveSubsystem` extends `TunerSwerveDrivetrain` (Phoenix 6 generated swerve)

Subsystems:
| Class | Mechanism |
|---|---|
| `SwerveSubsystem` | Holonomic swerve drive |
| `TurretSubsystem` | Turret rotation (dual CANcoder) |
| `ShooterSubsystem` | Dual-flywheel shooter |
| `ShooterHoodSubsystem` | Hood angle |
| `PreshooterSubsystem` | Pre-shooter feed |
| `ConveyerSubsystem` | Note conveyor |
| `IntakeShoulderSubsystem` | Intake pivot arm |
| `IntakeRollerSubsytem` | Intake rollers (note: typo in class name) |
| `IntakeAgitatorSubsytem` | Intake agitator (note: typo in class name) |
| `ClimberSubsystem` | Climb mechanism |
| `LimelightSubsystem` | Limelight vision integration |
| `QuestNavSubsystem` | Meta Quest Nav odometry |
| `HealthSubsystem` | PDH/PDM diagnostics |
| `BlinkyLightsSubsystem` | LED control |

### FSM (`src/main/java/frc/robot/fsm/`)

A custom state machine runs during teleop (disabled during auto). States implement `IState` and extend `SuperState`. The FSM is ticked in `Robot.robotPeriodic()` and publishes the current state to SmartDashboard. States handle complex multi-subsystem behaviors (scoring, climbing, depot/outpost interactions).

### Helpers (`src/main/java/frc/robot/Helpers/`)

Utilities that don't belong to a subsystem:
- `AllianceFlipUtil` — mirrors field coordinates for red alliance
- `ButtonTriggers` / `FieldTriggers` / `FMSTriggers` — trigger factory helpers
- `ShotCalculator` + `VelocityVector` — ballistic shot math
- `OrchestraManager` — plays music through TalonFX motors (runs in disabled mode)
- `RollingAveragePose3d` / `RollingAverageVelocity` — smoothing utilities

### Constants and Configuration

- `Constants.java` — CAN IDs for all motors and encoders
- `FieldConstants.java` — field geometry and scoring target poses
- `RobotParameters.java` / `RobotParametersContainer` — runtime robot identity and hardware config

### Vendor Libraries (in `vendordeps/`)

| Library | Purpose |
|---|---|
| Phoenix 6 | CTRE TalonFX motors, CANcoders, Pigeon2 IMU, swerve |
| REVLib | SparkMax motor controllers |
| PathPlannerLib | Autonomous path following |
| YAMS | Mechanism control abstractions |
| DogLog | Structured telemetry logging |
| QuestNavLib | Meta Quest Nav positioning |

### Logging

Two logging systems run in parallel:
- **DogLog** — captures NT4, DriverStation events, and structured robot data to `.wpilog`
- **TinyLog** — tagged text logging (`LoggingMaster.getLogger(getClass())`) with `logger.info(...)` calls throughout

The `name.remal.merge-resources` Gradle plugin is required because TinyLog uses `META-INF/services` and would otherwise conflict with fat-jar assembly.

### Generated Code

`src/main/java/frc/robot/Generated/` contains Phoenix 6 Tuner-generated swerve constants. Do not manually edit these files; regenerate via Phoenix Tuner X when hardware changes.

Git metadata is written to `src/main/deploy/git.properties` at build time via the `gradle-git-properties` plugin and is accessible at runtime through `GitNess`.
