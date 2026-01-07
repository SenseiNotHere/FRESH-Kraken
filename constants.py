import math

from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians

from phoenix6.hardware import talon_fx
from phoenix6.signals import NeutralModeValue

from pathplannerlib.config import RobotConfig, ModuleConfig, DCMotor

class KrakenX60:
    kFreeSpeedRpm = 5800
    kMaxSpeedMetersPerSecond = 2.8

class DrivingConstants:
    #Driving Motors
    kFrontLeftDriving = 7
    kFrontRightDriving = 5
    kBackLeftDriving = 2
    kBackRightDriving = 3

    #Turning Motors
    kFrontLeftTurning = 8
    kFrontRightTurning = 6
    kBackLeftTurning = 1
    kBackRightTurning = 4

    # CANCoders
    kFrontLeftTurningEncoder = 4
    kFrontRightTurningEncoder = 3
    kBackLeftTurningEncoder = 1
    kBackRightTurningEncoder = 2

    #Other settings
    #These are the maximum speeds the robot can reach, not maximum speeds allowed.
    kMaxMetersPerSecond = 3.0
    kMaxAngularSpeed = math.tau

    #Slew rate settings
    kDirectionSlewRate = 2.0  # radians per second
    kMagnitudeSlewRate = 1.8  # percent per second (1 = 100%)
    kRotationalSlewRate = 2.0  # percent per second (1 = 100%)

    # Chassis configuration
    kTrackWidth = units.inchesToMeters(26.5)
    # Distance between centers of right and left wheels on robot
    kWheelBase = units.inchesToMeters(26.5)

    # Distance between front and back wheels on robot
    kModulePositions = [
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    kAssumeZeroOffsets = False

    # set the above to == False, if you are manually zeroing (and you have to tinker with offsets below)
    kFrontLeftChassisAngularOffset = 0  # FL
    kFrontRightChassisAngularOffset = 0  # FR
    kBackLeftChassisAngularOffset = 0  # BL
    kBackRightChassisAngularOffset = 0  # BR

    kGyroReversed = -1

class ModuleConstants:
    # For most modules, encoder and turning CANNOT be both False or Positive!
    kTurningEncoderInverted = False
    kTurningMotorInverted = False

    # Driving motor inversions
    kFrontLeftDriveMotorInverted = True
    kFrontRightDriveMotorInverted = False
    kBackLeftDriveMotorInverted = True
    kBackRightDriveMotorInverted = False

    # Magnet Offsets
    kFrontLeftTurningEncoderOffset = 0.264404296875
    kFrontRightTurningEncoderOffset = 0.218505859375
    kBackLeftTurningEncoderOffset = -0.0810546875
    kBackRightTurningEncoderOffset = 0.0966796875

    kDrivingMotorPinionTeeth = 14

    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = KrakenX60.kFreeSpeedRpm / 60
    kWheelDiameterMeters = 0.0965
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi

    kDrivingMotorReduction = 6.12
    kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction
    kTurningMotorReduction = 12.6

    # radians are converted internally, these are MOTOR ROT units
    kTurningSyncIntervalSeconds = 0.25  # how often we check drift
    kTurningKalmanGain = 0.08  # 0 = disabled, 0.02–0.08 is sane
    kTurningDriftDegrees = 2.0  # hard snap threshold

    kTurningEncoderPositionFactor = math.tau  # radian
    kTurningEncoderVelocityFactor = math.tau / 60.0  # radians per second

    kTurningEncoderPositionPIDMinInput = 0  # radian
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor  # radian

    kDrivingP = 0.3
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = 3.0  # can be dialed down if you see oscillations in the turning motor
    kTurningI = 0
    kTurningD = 0
    kTurningFF = 0.05
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorIdleMode = talon_fx.signals.NeutralModeValue(NeutralModeValue.BRAKE)
    kTurningMotorIdleMode = talon_fx.signals.NeutralModeValue(NeutralModeValue.COAST)

    kDrivingMotorCurrentLimit = 70 # amp
    kDrivingMotorStatorCurrentLimit = 120 # amp
    kTurningMotorCurrentLimit = 40 # amp
    kTurningStatorCurrentLimit = 60 # amp

    kDrivingMinSpeedMetersPerSecond = 0.01

class OIConstants:
    kDriverControllerPort = 0
    kDriveDeadband = 0.05

class AutoConstants:
    moduleConfig = ModuleConfig(
        maxDriveVelocityMPS=DrivingConstants.kMaxMetersPerSecond,
        driveMotor=DCMotor.krakenX60(),
        driveCurrentLimit=ModuleConstants.kDrivingMotorCurrentLimit,
        numMotors=4,
        wheelRadiusMeters=ModuleConstants.kWheelDiameterMeters / 2,
        wheelCOF=1.0
    )

    config = RobotConfig(
        massKG=60.00,
        MOI=8.0,
        moduleConfig=moduleConfig,
        moduleOffsets=DrivingConstants.kModulePositions
    )

    #Additional settings
    config.maxModuleSpeed = DrivingConstants.kMaxMetersPerSecond
    config.driveBaseRadius = 0.45  # meters
    config.maxCentripetalAcceleration = 3.0  # m/s²

    kUseSqrtControl = True  # improves arrival time and precision for simple driving commands

    # Below are really trajectory constants
    kMaxMetersPerSecond = 1.2
    kMaxAccelerationMetersPerSecondSquared = 3.5
    kMaxAngularSpeedRadiansPerSecond = 5
    kMaxAngularSpeedRadiansPerSecondSquared = 25.0

    # PID Values
    # "Safer" to change
    kPXController = 5.0
    kPYController = 5.0
    kPThetaController = 6.0

    # ONLY CHANGE IF YOU KNOW WHAT YOU'RE DOING
    kIXController = 0.0
    kIYController = 0.0
    kIThetaController = 0.0

    kDXController = 0.0
    kDYController = 0.0
    kDThetaController = 0.0

    # Constraint for the motion profiled robot angle controller
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    )