import time

from commands2 import Subsystem
from phoenix6.configs import TalonFXConfiguration, Slot0Configs
from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage, Follower
from phoenix6.signals import NeutralModeValue, InvertedValue, MotorAlignmentValue
from wpilib import SmartDashboard

from constants import IntakeConstants


class Intake(Subsystem):
    def __init__(
        self,
        leadMotorCANID: int,
        followerMotorCANID: int | None = None,
        leadMotorInverted: bool = False,
    ):
        super().__init__()

        # Motors
        self.leadMotor = TalonFX(leadMotorCANID)
        self.followMotor = TalonFX(followerMotorCANID) if followerMotorCANID else None

        # Config
        leadConfig = TalonFXConfiguration()
        leadConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        leadConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if leadMotorInverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.leadMotor.configurator.apply(leadConfig)

        slot0 = Slot0Configs()
        (
            slot0
            .with_k_p(IntakeConstants.kP)
            .with_k_d(IntakeConstants.kD)
            .with_k_v(IntakeConstants.kFF)
        )
        self.leadMotor.configurator.apply(slot0)

        if self.followMotor:
            followConfig = TalonFXConfiguration()
            followConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
            self.followMotor.configurator.apply(followConfig)

            self.followMotor.set_control(
                Follower(
                    leadMotorCANID,
                    motor_alignment=MotorAlignmentValue.OPPOSED
                )
            )

        # Control Requests
        self._stopRequest = VelocityVoltage(0).with_slot(0)
        self._velocityRequest = VelocityVoltage(0).with_slot(0)

        # State
        self._enabled = False
        self._velocityRPM: float | None = None

        self._timedActive = False
        self._timedEndTime = 0.0

    # Periodic
    def periodic(self) -> None:
        now = time.time()

        if self._timedActive and now >= self._timedEndTime:
            self._timedActive = False
            self._velocityRPM = None

        if self._enabled and self._velocityRPM is not None:
            self._velocityRequest.with_velocity(self._rpmToRps(self._velocityRPM))
            self.leadMotor.set_control(self._velocityRequest)
        else:
            self.leadMotor.set_control(self._stopRequest)

        # Debug
        SmartDashboard.putBoolean("Intake Enabled", self._enabled)
        SmartDashboard.putBoolean("Intake Timed", self._timedActive)
        SmartDashboard.putNumber("Intake RPM", self.getRPM())

    # Public API
    def enable(self) -> None:
        self._enabled = True

    def disable(self) -> None:
        self._enabled = False
        self.stop()

    def setRPM(self, rpm: float) -> None:
        """Run intake at a constant RPM (until changed or stopped)."""
        self._velocityRPM = rpm
        self._timedActive = False

    def runForTime(self, rpm: float, seconds: float) -> None:
        """Run intake at RPM for a fixed duration."""
        self._velocityRPM = rpm
        self._timedActive = True
        self._timedEndTime = time.time() + seconds

    def stop(self) -> None:
        self._velocityRPM = None
        self._timedActive = False
        self.leadMotor.set_control(self._stopRequest)

    def getRPM(self) -> float:
        return self.leadMotor.get_velocity().value * 60.0

    # Helpers
    @staticmethod
    def _rpmToRps(rpm: float) -> float:
        return rpm / 60.0
