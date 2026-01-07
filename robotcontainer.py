from __future__ import annotations
import math

import commands2
import wpimath
import wpilib
import typing

from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import CommandGenericHID
from wpilib import XboxController, PS4Controller, SmartDashboard, DriverStation, DutyCycle
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Translation3d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)

from subsystems.drivesubsystem import DriveSubsystem, AutoBuilder, BadSimPhysics
from subsystems.limelightcamera import LimelightCamera
from subsystems.limelight_localizer import LimelightLocalizer
from commands.holonomicDrive import HolonomicDrive
from commands.approach import ApproachTag
from buttonbindings import ButtonBindings
from constants import OIConstants

from subsystems.phoenixswervemodule import PhoenixSwerveModule
from constants import *
import tests


class RobotContainer:
    """
    The container for the robot. Subsystems are initialized here, button bindings are set up, and
    auto chooser sent to the dashboard.
    """

    def __init__(self, robot):
        #The robot's subsystems
        self.robotDrive = DriveSubsystem()
        if commands2.TimedCommandRobot.isSimulation():
            self.robotDrive.simPhysics = BadSimPhysics(self.robotDrive, robot)
        
        # Auto chooser
        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

        # Test chooser
        self.testChooser = wpilib.SendableChooser()
        
        # Song chooser
        self.songChooser = wpilib.SendableChooser()
        self.songChooser.setDefaultOption("Bloodline - Ariana Grande", "/home/lvuser/py/deploy/files/Bloodline.chrp")
        self.songChooser.addOption("Yes And? - Ariana Grande", "/home/lvuser/py/deploy/files/Yesand.chrp")
        self.songChooser.addOption("Lavender Town", "/home/lvuser/py/deploy/files/LavenderTown.chrp")
        self.songChooser.addOption("Espresso - Sabrina Carpenter", "/home/lvuser/py/deploy/files/Espresso.chrp")
        self.songChooser.addOption("Needy - Ariana Grande", "/home/lvuser/py/deploy/files/Needy.chrp")
        SmartDashboard.putData("Song Selection", self.songChooser)

        #Setting up controllers
        self.driverController = CommandGenericHID(OIConstants.kDriverControllerPort)
        self.localizer = LimelightLocalizer(
            drivetrain=self.robotDrive,
            flipIfRed=True
        )
        self.limelight = LimelightCamera("limelight-front")
        self.limelightBack = LimelightCamera("limelight-back")
        self.limelightBack.setPiPMode(1)

        self.localizer.addCamera(
            camera=self.limelight,
            cameraPoseOnRobot=Translation3d(
                0.00,   # X forward (meters)
                0.00,   # Y left (meters)
                0.00    # Z up (meters)
            ),
            cameraHeadingOnRobot=Rotation2d.fromDegrees(0),
            minPercentFrame=0.07,
            maxRotationSpeed=720  # optional sanity limit
        )
        self.localizer.addCamera(
            camera=self.limelightBack,
            cameraPoseOnRobot=Translation3d(
                -18.00,
                0.00,
                0.00
            ),
            cameraHeadingOnRobot=Rotation2d.fromDegrees(180),
            minPercentFrame=0.07,
            maxRotationSpeed=720
        )

        self.robotDrive.setDefaultCommand(
            HolonomicDrive(
                self.robotDrive,
                forwardSpeed=lambda: self.driverController.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: self.driverController.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: self.driverController.getRawAxis(XboxController.Axis.kRightX),
                deadband=OIConstants.kDriveDeadband,
                fieldRelative=True,
                rateLimit=True,
                square=True,
            )
        )

        # Initialize button bindings
        self.buttonBindings = ButtonBindings(self)
        self.buttonBindings.configureButtonBindings()

    def disablePIDSubsystems(self):
        """
        Disables all PID Subsystems
        """

    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous mode.
        """
        command = self.autoChooser.getSelected()
        if command is None:
            print("WARNING: No autonomous routines selected!") # Will return a command that does nothing
            return InstantCommand()

        print("Running autonomous routine: " + command.getName())
        return command

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command that will be running when test mode is enabled
        """
        self.testChooser.setDefaultOption("None", None)
        self.testChooser.addOption("Drivetrain ", InstantCommand(tests.drivetrainTest.testDrive(self.robotDrive)))