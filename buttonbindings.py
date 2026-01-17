from wpilib import XboxController, PS4Controller
from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import CommandGenericHID
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.limelightcamera import LimelightCamera
from constants import *

from commands.reset_XY import ResetXY, ResetSwerveFront
from commands.followObject import FollowObject
from commands.approach import ApproachTag
from commands.limelightComands import SetCameraPipeline

class ButtonBindings:
    def __init__(self, robot_container):
        """Initialize ButtonBindings with access to the robot container
        """
        self.robotContainer = robot_container
        self.robotDrive = robot_container.robotDrive
        self.driverController = robot_container.driverController
        self.limelight = robot_container.limelight
        if ShooterConstants.kShooterEnabled:
            self.shooter = robot_container.shooter
        if IndexerConstants.kIndexerEnabled:
            self.indexer = robot_container.indexer

    def configureButtonBindings(self):
        """Configure button bindings for the robot."""

        # Driver Controls
        # Reset XY Position
        povUpDriverButton = self.driverController.pov(0)
        povUpDriverButton.onTrue(
            ResetXY(
                x=0.0,
                y=0.0,
                headingDegrees=0.0,
                drivetrain=self.robotDrive
            )
        )

        # Reset Swerve Front
        povDownDriverButton = self.driverController.pov(180)
        povDownDriverButton.onTrue(ResetSwerveFront(self.robotDrive))

        # X-Break
        povLeftDriverButton = self.driverController.pov(270)
        povLeftDriverButton.whileTrue(InstantCommand(self.robotDrive.setX, self.robotDrive))

        # Follow Object
        yDriverButton = self.driverController.button(XboxController.Button.kY)
        yDriverButton.whileTrue(
            ApproachTag(
                camera=self.limelight,
                drivetrain=self.robotDrive,
                dashboardName="ApproachTag",
                reverse=True
            )
        )

        # Play selected song
        bButton = self.driverController.button(XboxController.Button.kB)
        bButton.onTrue(InstantCommand(lambda: self.robotDrive.playSound(self.robotContainer.songChooser.getSelected())))

        # Stop song
        aButton = self.driverController.button(XboxController.Button.kA)
        aButton.onTrue(InstantCommand(lambda: self.robotDrive.stopSound()))

        # Shooter + Indexer
        xButton = self.driverController.button(XboxController.Button.kX)

        # Shooter only
        if ShooterConstants.kShooterEnabled and not IndexerConstants.kIndexerEnabled:
            # Shooter
            xButton.whenTrue(InstantCommand(lambda: self.robotContainer.shooter.enable()))
            xButton.whenFalse(InstantCommand(lambda: self.robotContainer.shooter.disable()))

        # Shooter + Indexer
        elif ShooterConstants.kShooterEnabled and IndexerConstants.kIndexerEnabled:
            # Shooter
            xButton.whenTrue(InstantCommand(lambda: self.shooter.enable()))
            xButton.whenFalse(InstantCommand(lambda: self.shooter.disable()))

            # Indexer
            xButton.whileTrue(InstantCommand(lambda: self.indexer.enable() if self.shooter.atSpeed() else print("Not ready to index yet")))
            xButton.whenFalse(InstantCommand(lambda: self.indexer.disable()))