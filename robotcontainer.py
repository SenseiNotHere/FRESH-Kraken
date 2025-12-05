from __future__ import annotations
import math

import commands2
import phoenix6.controls
import wpimath
import wpilib
import typing
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import NeutralModeValue, InvertedValue
from phoenix6.controls import VelocityVoltage, PositionVoltage, DutyCycleOut

from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import CommandGenericHID
from wpilib import XboxController, PS4Controller, SmartDashboard, DriverStation, DutyCycle
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)

from subsystems.drivesubystem import DriveSubsystem, AutoBuilder, BadSimPhysics
from subsystems.limelightcamera import LimelightCamera
from commands.holonomicDrive import HolonomicDrive
from buttonbindings import ButtonBindings
from constants import OIConstants

from subsystems.phoenixswervemodule import PhoenixSwerveModule
from constants import *


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
        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

        #Setting up controllers
        self.driverController = CommandGenericHID(OIConstants.kDriverControllerPort)
        self.limelight = LimelightCamera("camera")
        self.limelight.setPiPMode(1)
        self.buttonBindings = ButtonBindings(self)
        self.buttonBindings.configureButtonBindings()

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