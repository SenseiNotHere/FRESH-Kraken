import time

from commands2 import Subsystem
from phoenix6.configs import TalonFXConfiguration, Slot0Configs
from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage
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

        # Slot 0 PID + FF
        slot0 = Slot0Configs()
        (slot0
            .with_k_p(ShooterConstants.kP)
            .with_k_d(ShooterConstants.kD)
            .with_k_v(ShooterConstants.kFF)
        )
        self.motor.configurator.apply(slot0)

        # Control Request
        self.velocityRequest = VelocityVoltage(0).with_slot(0)

        # State
        self.velocitySetpointRPM = 0.0

        # Dashboard Chooser
        self.speedChooser = SendableChooser()
        self.speedChooser.setDefaultOption("Off", 0.0)
        self.speedChooser.addOption("10%", 0.1)
        self.speedChooser.addOption("20%", 0.2)
        self.speedChooser.addOption("30%", 0.3)
        self.speedChooser.addOption("40%", 0.4)
        self.speedChooser.addOption("50%", 0.5)
        self.speedChooser.addOption("60%", 0.6)
        self.speedChooser.addOption("70%", 0.7)
        self.speedChooser.addOption("80%", 0.8)
        self.speedChooser.addOption("90%", 0.9)
        self.speedChooser.addOption("100%", 1.0)

        SmartDashboard.putData("Shooter Speed", self.speedChooser)

        # Enabled State
        self.enabled = False

        # Run For Time State
        self.runForTimeActive = False
        self.runForTimeEndTime = 0.0

    # Periodic
    def periodic(self):
        if self.runForTimeActive and time.time() >= self.runForTimeEndTime:
            self.runForTimeActive = False
            self.velocitySetpointRPM = 0.0

        if self.enabled and not self.runForTimeActive:
            percent = self.speedChooser.getSelected()
            self.setVelocityRPM(percent * ShooterConstants.kMaxRPM)
        elif not self.enabled and not self.runForTimeActive:
            self.velocitySetpointRPM = 0.0

        self.velocityRequest = self.velocityRequest.with_velocity(
            self.velocitySetpointRPM
        )
        self.motor.set_control(self.velocityRequest)

        measuredRPM = self.getVelocityRPM()

        SmartDashboard.putNumber("Shooter RPM", measuredRPM)
        SmartDashboard.putNumber("Shooter Setpoint", self.velocitySetpointRPM)
        SmartDashboard.putNumber(
            "Shooter Error",
            self.velocitySetpointRPM - measuredRPM
        )

    # Private API

    def _setVelocityRPM(self, rpm: float):
        self.velocitySetpointRPM = max(
            min(rpm, ShooterConstants.kMaxRPM),
            0.0
        )

    def _stop(self):
        self.velocitySetpointRPM = 0.0
        self.motor.set_control(self.velocityRequest.with_velocity(0.0))

    # Public API

    def setVelocityRPM(self, rpm: float):
        self._setVelocityRPM(rpm)

    def runForTime(self, rpm: float, seconds: float):
        self._setVelocityRPM(rpm)
        self.runForTimeActive = True
        self.runForTimeEndTime = time.time() + seconds

    def getVelocityRPM(self) -> float:
        return self.motor.get_velocity().value

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False
        self._stop()