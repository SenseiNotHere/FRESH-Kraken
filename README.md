# FRESH-Kraken (RobotPy)

Robot code for FRC Team 1811.

This codebase is being actively worked on — especially the swerve drive. If you want to help, please open a PR or reach out.

**Credits**
- FRC Team 1811
- Project contributions alongside https://github.com/epanov1602

## Quick start (dev machine)

### Requirements
- Python 3.11+ recommended (RobotPy 2025)
- Git

### Install dependencies

```powershell
python -m pip install -r requirements.txt
```

### Run in simulation

```powershell
python -m robotpy sim
```

## Deploy to RoboRIO

RobotPy projects are typically deployed using the RobotPy CLI.

Tip: if you want to pull down/install the same requirements locally as the robot image, run:

```powershell
python -m robotpy sync
```

```powershell
python -m robotpy deploy
```

## What’s in this repo

- `robot.py`: Robot entrypoint (Command-based v2)
- `robotcontainer.py`: Initializes subsystems, default commands, and button bindings
- `subsystems/drivesubsystem.py`: Swerve drivetrain + odometry + PathPlanner AutoBuilder integration
- `subsystems/phoenixswervemodule.py`: Phoenix 6 TalonFX + CANcoder swerve module implementation
- `commands/`: Command-based behaviors (drive, follow object, trajectories, etc.)
- `deploy/`: Files copied to the RoboRIO on deploy
  - `deploy/pathplanner/`: PathPlanner autos/paths + navgrid
  - `deploy/files/*.chrp`: Phoenix Orchestra music files

## Swerve notes (help wanted)

We’ve been chasing reliability issues with the swerve; help is welcome.

Relevant implementation details:
- CANcoder offsets and inversions are in `constants.py` under `ModuleConstants`.
- Chassis angular offsets are in `constants.py` under `DrivingConstants` (set `kAssumeZeroOffsets` to control whether offsets are applied).
- Module steering alignment:
  - Each `PhoenixSwerveModule` calls `resetEncoders()` on startup.
  - The drivetrain also syncs turning encoders once in `DriveSubsystem.disabledInit()`.
- Gyro is a NavX (`navx.AHRS.create_spi()`), with a sign flip via `DrivingConstants.kGyroReversed`.

Simulation note:
- `subsystems/drivesubsystem.py` contains a `BadSimPhysics` helper that is intentionally labeled as a placeholder. Improving the swerve sim physics would be a great contribution.

## Contributing

- Open a PR (preferred), and include:
  - What you changed
  - How you tested (real robot vs sim)
  - Any tuning constants you touched
- If you’re unsure where to start, open an issue describing what you want to tackle.

## Contact

- Instagram: https://instagram.com/le_jonaspains
- Or just open a PR on GitHub
