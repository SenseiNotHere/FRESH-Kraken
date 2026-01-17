import time

from commands2 import Subsystem
from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage
from phoenix6.configs import Slot0Configs, CurrentLimitsConfigs
from phoenix6.signals import NeutralModeValue, InvertedValue
from wpilib import SmartDashboard, SendableChooser

from constants import ShooterConstants


class Shooter(Subsystem):
    def __init__(self, motorCANID: int, motorInverted: bool):
        super().__init__()

        # Motor Init
        self.motor = TalonFX(motorCANID)

        motorConfig = TalonFXConfiguration()
        motorConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        motorConfig.motor_output.inverted = (
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            if motorInverted
            else InvertedValue.CLOCKWISE_POSITIVE
        )
        self.motor.configurator.apply(motorConfig)

        # Control gains
        motorSlotConfig = Slot0Configs()
        (motorSlotConfig
            .with_k_d(ShooterConstants.kD)
            .with_k_p(ShooterConstants.kP)
            .with_k_v(ShooterConstants.kFF)
        )
        self.motor.configurator.apply(motorSlotConfig)

        # Current Limits
        motorCurrentLimits = CurrentLimitsConfigs()
        (motorCurrentLimits
         .with_supply_current_limit(ShooterConstants.kShooterSupplyLimit)
         .with_stator_current_limit(ShooterConstants.kShooterStatorLimit)
         .with_supply_current_limit_enable(True)
         .with_stator_current_limit_enable(True)
        )
        self.motor.configurator.apply(motorCurrentLimits)

        # Velocity control request
        self.velocityRequest = VelocityVoltage(0.0).with_slot(0)

        # State
        self.enabled = False
        self.outputPercent = 0.0

        # Run-for-time state
        self.runForTimeActive = False
        self.runForTimeEndTime = 0.0

        # Dashboard Chooser (percent of max RPM)
        self.speedChooser = SendableChooser()
        self.speedChooser.addOption("Off", 0.0)
        self.speedChooser.addOption("10%", 0.1)
        self.speedChooser.addOption("20%", 0.2)
        self.speedChooser.addOption("30%", 0.3)
        self.speedChooser.addOption("40%", 0.4)
        self.speedChooser.addOption("50%", 0.5)
        self.speedChooser.addOption("60%", 0.6)
        self.speedChooser.addOption("70%", 0.7)
        self.speedChooser.addOption("80%", 0.8)
        self.speedChooser.addOption("90%", 0.9)
        self.speedChooser.setDefaultOption("100%", 1.0)

        SmartDashboard.putData("Shooter Speed", self.speedChooser)

        # Max shooter speed (Kraken more or less 6000 RPM free)
        self.kMaxRPM = ShooterConstants.kMaxRPM

    # Periodic

    def periodic(self):
        # Handle timed run
        if self.runForTimeActive and time.time() >= self.runForTimeEndTime:
            self.runForTimeActive = False
            self.outputPercent = 0.0

        # Normal control
        if self.enabled and not self.runForTimeActive:
            self.outputPercent = self.speedChooser.getSelected()
        elif not self.enabled and not self.runForTimeActive:
            self.outputPercent = 0.0

        # Percent => RPM => RPS
        target_rpm = self.outputPercent * self.kMaxRPM
        target_rps = target_rpm / 60.0

        # Apply velocity control
        self.motor.set_control(
            self.velocityRequest.with_velocity(target_rps)
        )

        # Telemetry
        SmartDashboard.putNumber("Shooter Target RPM", target_rpm)
        SmartDashboard.putNumber(
            "Shooter Velocity (RPS)",
            self.motor.get_velocity().value
        )

    # Public API

    def enable(self):
        """
        Enables the shooter motor.
        """
        self.enabled = True

    def disable(self):
        """
        Disables the shooter motor.
        """
        self.enabled = False
        self.outputPercent = 0.0
        self.motor.set_control(self.velocityRequest.with_velocity(0.0))

    def setPercent(self, percent: float):
        """
        Sets the shooter motor to a constant percent output.
        """
        self.outputPercent = max(min(percent, 1.0), 0.0)

    def runForTime(self, percent: float, seconds: float):
        """
        Runs the shooter motor at a constant percent output for a fixed duration.
        """
        self.outputPercent = max(min(percent, 1.0), 0.0)
        self.runForTimeActive = True
        self.runForTimeEndTime = time.time() + seconds

    def atSpeed(self, tolerance_rpm) -> bool:
        """
        Returns true if the shooter motor is at the desired speed.
        :return: True if at shooting speed, False otherwise.
        """
        target_rpm = self.outputPercent * self.kMaxRPM
        current_rpm = self.motor.get_velocity().value * 60.0
        return current_rpm >= (target_rpm - tolerance_rpm)