from __future__ import annotations

import math
import typing

import wpilib

from commands2 import Subsystem
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)
from wpilib import SmartDashboard, Field2d, DriverStation

from commands.aimToDirection import AimToDirectionConstants
from .phoenixswervemodule import PhoenixSwerveModule
import navx

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants

from phoenix6.orchestra import Orchestra

from constants import DrivingConstants, ModuleConstants, AutoConstants
from commands.holonomicDrive import HolonomicDrive

import commands2

class DriveSubsystem(Subsystem):
    def __init__(self, maxSpeedScaleFactor=None) -> None:
        super().__init__()

        if maxSpeedScaleFactor is not None:
            assert callable(maxSpeedScaleFactor)

        self.maxSpeedScaleFactor = maxSpeedScaleFactor

        self.frontLeft = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kFrontLeftDriving,
            turningCANId=DrivingConstants.kFrontLeftTurning,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            driveMotorInverted=ModuleConstants.kFrontLeftDriveMotorInverted,
            canCoderCANId=DrivingConstants.kFrontLeftTurningEncoder,
            canCoderInverted=ModuleConstants.kTurningEncoderInverted,
            canCoderOffset=ModuleConstants.kFrontLeftTurningEncoderOffset,
            modulePlace="FL"
        )

        self.frontRight = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kFrontRightDriving,
            turningCANId=DrivingConstants.kFrontRightTurning,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            driveMotorInverted=ModuleConstants.kFrontRightDriveMotorInverted,
            canCoderCANId=DrivingConstants.kFrontRightTurningEncoder,
            canCoderInverted=ModuleConstants.kTurningEncoderInverted,
            canCoderOffset=ModuleConstants.kFrontRightTurningEncoderOffset,
            modulePlace="FR"
        )

        self.backLeft = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kBackLeftDriving,
            turningCANId=DrivingConstants.kBackLeftTurning,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            driveMotorInverted=ModuleConstants.kBackLeftDriveMotorInverted,
            canCoderCANId=DrivingConstants.kBackLeftTurningEncoder,
            canCoderInverted=ModuleConstants.kTurningEncoderInverted,
            canCoderOffset=ModuleConstants.kBackLeftTurningEncoderOffset,
            modulePlace="BL"
        )

        self.backRight = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kBackRightDriving,
            turningCANId=DrivingConstants.kBackRightTurning,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            driveMotorInverted=ModuleConstants.kBackRightDriveMotorInverted,
            canCoderCANId=DrivingConstants.kBackRightTurningEncoder,
            canCoderInverted=ModuleConstants.kTurningEncoderInverted,
            canCoderOffset=ModuleConstants.kBackRightTurningEncoderOffset,
            modulePlace="BR"
        )

        # Override for the direction where robot should point
        self.overrideControlsToFaceThisPoint: Translation2d | None = None

        #RoboRIO Gyro Sensor
        self.gyro = navx.AHRS.create_spi()
        self._lastGyroAngleTime = 0
        self._lastGyroAngle = 0
        self._lastGyroState = "ok"

        self.xSpeedLimiter = SlewRateLimiter(DrivingConstants.kMagnitudeSlewRate)
        self.ySpeedLimiter = SlewRateLimiter(DrivingConstants.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(DrivingConstants.kRotationalSlewRate)

        #Odometry class for tracking robot pose
        self.odometry = SwerveDrive4Odometry(
            DrivingConstants.kDriveKinematics,
            Rotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
        self.odometryHeadingOffset = Rotation2d(0)
        self.resetOdometry(Pose2d(0, 0, 0))

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        self.simPhysics = None

        AutoBuilder.configure(
            self.getPose,  # Robot pose supplier
            self.resetOdometry,  # Reset odometry at auto start
            self.getRobotRelativeSpeeds,  # MUST be robot-relative speeds
            lambda speeds, _: self.driveRobotRelativeChassisSpeeds(speeds),
            PPHolonomicDriveController(
                PIDConstants(
                    AutoConstants.kPXController,
                    AutoConstants.kIXController,
                    AutoConstants.kDXController
                ),
                PIDConstants(
                    AutoConstants.kPThetaController,
                    AutoConstants.kIThetaController,
                    AutoConstants.kDThetaController
                )
            ),
            AutoConstants.config,  # RobotConfig
            self.shouldFlipPath,  # Alliance-based flipping
            self  # Subsystem requirement
        )

        # Initialize Orchestra for playing music through all motors
        self.orchestra = Orchestra([
            self.frontLeft.drivingMotor,
            self.frontLeft.turningMotor,
            self.frontRight.drivingMotor,
            self.frontRight.turningMotor,
            self.backLeft.drivingMotor,
            self.backLeft.turningMotor,
            self.backRight.drivingMotor,
            self.backRight.turningMotor,
        ])

    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        """Returns the current robot-relative ChassisSpeeds"""
        return DrivingConstants.kDriveKinematics.toChassisSpeeds(
            (
                self.frontLeft.getState(),
                self.frontRight.getState(),
                self.backLeft.getState(),
                self.backRight.getState(),
            )
        )

    def shouldFlipPath(self):
        """
        :return: Whether to flip the path based on alliance color
        """
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def periodic(self) -> None:
        if self.simPhysics is not None:
            self.simPhysics.periodic()

        # Sync turning encoders on all modules to prevent drift
        self.frontLeft.periodic()
        self.frontRight.periodic()
        self.backLeft.periodic()
        self.backRight.periodic()

        # Update the odometry in the periodic block
        pose = self.odometry.update(
            self.getGyroHeading(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        self.field.setRobotPose(pose)

        # Data Puts
        # Pose
        SmartDashboard.putNumber("X Coordinate", pose.x)
        SmartDashboard.putNumber("Y Coordinate", pose.y)
        SmartDashboard.putNumber("Heading (deg)", pose.rotation().degrees())

        # Temperatures
        SmartDashboard.putNumberArray("Front Left Temp", self.frontLeft.getTemperature())
        SmartDashboard.putNumberArray("Front Right Temp", self.frontRight.getTemperature())
        SmartDashboard.putNumberArray("Back Left Temp", self.backLeft.getTemperature())
        SmartDashboard.putNumberArray("Back Right Temp", self.backRight.getTemperature())

        # Positions
        SmartDashboard.putNumber("Front Left Position", self.frontLeft.getPosition().angle.degrees())
        SmartDashboard.putNumber("Front Right Position", self.frontRight.getPosition().angle.degrees())
        SmartDashboard.putNumber("Back Left Position", self.backLeft.getPosition().angle.degrees())
        SmartDashboard.putNumber("Back Right Position", self.backRight.getPosition().angle.degrees())

    def getHeading(self) -> Rotation2d:
        """
        :return: The robot's heading as a Rotation2d
        """

        return self.getPose().rotation()

    def getPose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        return self.odometry.getPose()

    def resetOdometry(self, pose: Pose2d) -> None:
        """Resets the odometry to the specified pose.

        :param pose: The pose to which to set the odometry.

        """
        self.gyro.reset()
        self.gyro.setAngleAdjustment(0)
        self._lastGyroAngleTime = 0
        self._lastGyroAngle = 0

        self.odometry.resetPosition(
            self.getGyroHeading(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            pose,
        )
        self.odometryHeadingOffset = self.odometry.getPose().rotation() - self.getGyroHeading()

    def adjustOdometry(self, dTrans: Translation2d, dRot: Rotation2d):
        """Adjusts the odometry by a specified translation and rotation delta.
        :param dTrans: The translation delta to apply.
        :param dRot: The rotation delta to apply.
        """
        pose = self.getPose()
        newPose = Pose2d(pose.translation() + dTrans, pose.rotation() + dRot)
        self.odometry.resetPosition(
            pose.rotation() - self.odometryHeadingOffset,
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            newPose,
        )
        self.odometryHeadingOffset += dRot

    def stop(self):
        """
        Stops the robot by setting all speeds to zero.
        """
        self.arcadeDrive(0, 0)

    def arcadeDrive(
            self,
            xSpeed: float,
            rot: float,
            assumeManualInput: bool = False,
    ) -> None:
        """
        Drive the robot using arcade controls.
        :param xSpeed: forward speed
        :param rot: rotation speed
        :param assumeManualInput: whether to square the inputs for manual control
        """
        self.drive(xSpeed, 0, rot, False, False, square=assumeManualInput)

    def rotate(self, rotSpeed) -> None:
        """
        Rotate the robot in place, without moving laterally (for example, for aiming)
        :param rotSpeed: rotation speed
        """
        self.arcadeDrive(0, rotSpeed)

    def drive(
            self,
            xSpeed: float,
            ySpeed: float,
            rot: float,
            fieldRelative: bool,
            rateLimit: bool,
            square: bool = False
    ) -> None:
        """Method to drive the robot using joystick info.

        :param xSpeed:        Speed of the robot in the x direction (forward).
        :param ySpeed:        Speed of the robot in the y direction (sideways).
        :param rot:           Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the
                              field.
        :param rateLimit:     Whether to enable rate limiting for smoother control.
        :param square:        Whether to square the inputs (useful for manual control)
        """

        # Apply constraints and transformations to the user (joystick) input
        if square:
            rot = rot * abs(rot)
            norm = math.hypot(xSpeed, ySpeed)
            xSpeed = xSpeed * norm
            ySpeed = ySpeed * norm
        if (xSpeed != 0 or ySpeed != 0) and self.maxSpeedScaleFactor is not None:
            norm = math.hypot(xSpeed, ySpeed)
            scale = abs(self.maxSpeedScaleFactor() / norm)
            if scale < 1:
                xSpeed = xSpeed * scale
                ySpeed = ySpeed * scale
        xSpeedCommanded = xSpeed
        ySpeedCommanded = ySpeed

        if self.overrideControlsToFaceThisPoint:
            rot = self.calaculateOverrideRotSpeed()

        # Convert the commanded speeds into the correct units for the drivetrain
        xSpeedGoal = xSpeedCommanded * DrivingConstants.kMaxMetersPerSecond
        ySpeedGoal = ySpeedCommanded * DrivingConstants.kMaxMetersPerSecond
        rotSpeedGoal = rot * DrivingConstants.kMaxAngularSpeed

        # field relative conversion must happen before rate limiting, since rate limiting is optional
        if fieldRelative:
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedGoal, ySpeedGoal, rotSpeedGoal, self.getGyroHeading(),
            )
        else:
            targetChassisSpeeds = ChassisSpeeds(xSpeedGoal, ySpeedGoal, rotSpeedGoal)

        # rate limiting has to be applied this way, to keep the rate limiters current (with time)
        slewedX = self.xSpeedLimiter.calculate(targetChassisSpeeds.vx)
        slewedY = self.ySpeedLimiter.calculate(targetChassisSpeeds.vy)
        slewedRot = self.rotLimiter.calculate(targetChassisSpeeds.omega)
        if rateLimit:
            targetChassisSpeeds.vx, targetChassisSpeeds.vy, targetChassisSpeeds.omega = slewedX, slewedY, slewedRot

        # from chassis speed targets, calculate and set the wheel speeds targets
        swerveModuleStates = DrivingConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds)
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, DrivingConstants.kMaxMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(rl)
        self.backRight.setDesiredState(rr)


    def driveRobotRelativeChassisSpeeds(self, speeds: ChassisSpeeds) -> None:
        states = DrivingConstants.kDriveKinematics.toSwerveModuleStates(speeds)
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            states, DrivingConstants.kMaxMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(rl)
        self.backRight.setDesiredState(rr)

        SmartDashboard.putNumber("Auto vx", speeds.vx)
        SmartDashboard.putNumber("Auto omega", speeds.omega)


    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        self.frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.frontRight.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        )
        self.backLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.backRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def setModuleStates(
            self,
            desiredStates: typing.Tuple[
                SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
            ],
    ) -> None:
        """Sets the swerve ModuleStates.

        :param desiredStates: The desired SwerveModule states.
        """
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, DrivingConstants.kMaxMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(rl)
        self.backRight.setDesiredState(rr)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.frontLeft.resetEncoders()
        self.backLeft.resetEncoders()
        self.frontRight.resetEncoders()
        self.backRight.resetEncoders()

    def isSteerReady(self, tolerance_rad=2.0/57.) -> bool:
        """
        Returns True if all swerve modules are within tolerance of their desired angle.
        :param tolerance_rad: Allowed angle error (degrees) before a module is considered aligned.
        :return:
        """
        modules = [
            self.frontLeft,
            self.frontRight,
            self.backLeft,
            self.backRight,
        ]

        for m in modules:
            error = abs(
                m.getTurningPosition()
                - m.desiredState.angle.radians()
            )
            if math.degrees(error) > tolerance_rad:
                return False

        return True

    def getGyroHeading(self) -> Rotation2d:
        """Returns the heading of the robot, tries to be smart when gyro is disconnected

        :returns: the robot's heading as Rotation2d
        """
        now = wpilib.Timer.getFPGATimestamp()
        past = self._lastGyroAngleTime
        state = "ok"

        if not self.gyro.isConnected():
            state = "disconnected"
        else:
            if self.gyro.isCalibrating():
                state = "calibrating"
            self._lastGyroAngle = self.gyro.getAngle()
            self._lastGyroAngleTime = now

        if state != self._lastGyroState:
            SmartDashboard.putString("gyro", f"{state} after {int(now - past)}s")
            self._lastGyroState = state

        return Rotation2d.fromDegrees(self._lastGyroAngle * DrivingConstants.kGyroReversed)

    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot (in degrees per second)

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * DrivingConstants.kGyroReversed

    def getTurnRateDegreesPerSec(self) -> float:
        """Returns the turn rate of the robot (in degrees per second)

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.getTurnRate() * 180 / math.pi

    def playSound(self, path: str = "/home/lvuser/py/deploy/files/Bloodline.chrp"):
        """
        Play music through the swerve drive motors using Phoenix Orchestra.
        All 8 motors (4 drive + 4 turn) will play together for maximum volume.
        :param path: Path to the .chrp music file on the RoboRIO
        """
        print(f"Loading and playing music: {path}")
        self.orchestra.load_music(path)
        self.orchestra.play()

    def stopSound(self):
        """
        Stop playing music through the motors.
        """
        self.orchestra.stop()


    def startOverrideToFaceThisPoint(self, point: Translation2d) -> bool:
        if self.overrideControlsToFaceThisPoint is not None:
            return False
        self.overrideControlsToFaceThisPoint = point
        return True


    def stopOverrideToFaceThisPoint(self, point: Translation2d):
        if self.overrideControlsToFaceThisPoint == point:
            self.overrideControlsToFaceThisPoint = None
            return True
        return False


    def calaculateOverrideRotSpeed(self):
        # 1. how many degrees we need to turn?
        pose = self.getPose()
        vectorToTarget = self.overrideControlsToFaceThisPoint - pose.translation()
        if not vectorToTarget.squaredNorm() > 0:
            return 0.0
        targetDirection = vectorToTarget.angle()
        degreesRemainingToTurn = (targetDirection - pose.rotation()).degrees()

        # (do not turn left 350 degrees if you can just turn right -10 degrees, and vice versa)
        while degreesRemainingToTurn > 180:
            degreesRemainingToTurn -= 360
        while degreesRemainingToTurn < -180:
            degreesRemainingToTurn += 360

        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        proportionalSpeed = AimToDirectionConstants.kP * abs(degreesRemainingToTurn)
        if AimToDirectionConstants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)  # will match the non-sqrt value when 50% max speed
        rotSpeed = min(proportionalSpeed, 1.0)

        # 3. if need to turn left, return the positive speed, otherwise negative
        return rotSpeed if degreesRemainingToTurn > 0 else -rotSpeed


class BadSimPhysics(object):
    """
    this is the wrong way to do it, it does not scale!!!
    the right way is shown here: https://github.com/robotpy/examples/blob/main/Physics/src/physics.py
    and documented here: https://robotpy.readthedocs.io/projects/pyfrc/en/stable/physics.html
    (but for a swerve drive it will take some work to add correctly)
    """
    def __init__(self, drivetrain: DriveSubsystem, robot: wpilib.RobotBase):
        self.drivetrain = drivetrain
        self.robot = robot
        self.t = 0

    def periodic(self):
        past = self.t
        self.t = wpilib.Timer.getFPGATimestamp()
        if past == 0:
            return  # it was first time

        dt = self.t - past
        if self.robot.isEnabled():
            drivetrain = self.drivetrain

            states = (
                drivetrain.frontLeft.desiredState,
                drivetrain.frontRight.desiredState,
                drivetrain.backLeft.desiredState,
                drivetrain.backRight.desiredState,
            )
            speeds = DrivingConstants.kDriveKinematics.toChassisSpeeds(states)

            dx = speeds.vx * dt
            dy = speeds.vy * dt

            heading = drivetrain.getHeading()
            trans = Translation2d(dx, dy).rotateBy(heading)
            rot = (speeds.omega * 180 / math.pi) * dt

            g = drivetrain.gyro
            g.setAngleAdjustment(g.getAngleAdjustment() + rot * DrivingConstants.kGyroReversed)
            drivetrain.adjustOdometry(trans, Rotation2d())