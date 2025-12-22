#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license files in the root directory of this project.
#

from wpilib import Timer
from commands2 import Subsystem
from ntcore import NetworkTableInstance


class LimelightCamera(Subsystem):
    def __init__(self, cameraName: str) -> None:
        super().__init__()

        self.cameraName = _fix_name(cameraName)

        instance = NetworkTableInstance.getDefault()
        self.table = instance.getTable(self.cameraName)
        self._path = self.table.getPath()

        self.pipelineIndexRequest = self.table.getDoubleTopic("pipeline").publish()
        self.pipelineIndex = self.table.getDoubleTopic("getpipe").getEntry(-1)
        # "cl" and "tl" are additional latencies in milliseconds

        self.ledMode = self.table.getIntegerTopic("ledMode").getEntry(-1)
        self.camMode = self.table.getIntegerTopic("camMode").getEntry(-1)
        self.tx = self.table.getDoubleTopic("tx").getEntry(0.0)
        self.ty = self.table.getDoubleTopic("ty").getEntry(0.0)
        self.ta = self.table.getDoubleTopic("ta").getEntry(0.0)
        self.hb = self.table.getIntegerTopic("hb").getEntry(0)
        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False

    def setPipeline(self, index: int):
        """
        Sets the current pipeline index.
        :param index: The pipeline index to set.
        """
        self.pipelineIndexRequest.set(float(index))

    def getPipeline(self) -> int:
        """
        Gets the current pipeline index.
        :return: The current pipeline index.
        """
        return int(self.pipelineIndex.get(-1))

    def getA(self) -> float:
        """
        Gets the target area.
        :return: The target area.
        """
        return self.ta.get()

    def getX(self) -> float:
        """
        Gets the horizontal offset from crosshair to target.
        :return: The horizontal offset in degrees.
        """
        return self.tx.get()

    def getY(self) -> float:
        """
        Gets the vertical offset from crosshair to target.
        :return: The vertical offset in degrees.
        """
        return self.ty.get()

    def getHB(self) -> float:
        """
        Gets the heartbeat value.
        :return: The heartbeat value.
        """
        return self.hb.get()

    def hasDetection(self):
        """
        :return: Whether the camera currently has a valid target detection.
        """
        if self.getX() != 0.0 and self.heartbeating:
            return True

    def getSecondsSinceLastHeartbeat(self) -> float:
        """
        :return: The number of seconds since the last heartbeat was received.
        """
        return Timer.getFPGATimestamp() - self.lastHeartbeatTime

    def periodic(self) -> None:
        now = Timer.getFPGATimestamp()
        heartbeat = self.getHB()
        if heartbeat != self.lastHeartbeat:
            self.lastHeartbeat = heartbeat
            self.lastHeartbeatTime = now
        heartbeating = now < self.lastHeartbeatTime + 5  # no heartbeat for 5s => stale camera
        if heartbeating != self.heartbeating:
            print(f"Camera {self.cameraName} is " + ("UPDATING" if heartbeating else "NO LONGER UPDATING"))
        self.heartbeating = heartbeating

    def setPiPMode(self, mode: int):
        """
        Sets the picture-in-picture mode.
        mode: 0 = Side-by-Side, 1 = Secondary Camera in Lower-Right Corner
        """
        self.table.putNumber("stream", mode)

def _fix_name(name: str):
    """
    Ensures the camera name is valid for NetworkTables. | Not to be called directly.
    :param name: The camera name.
    :return: The fixed camera name.
    """
    if not name:
        name = "limelight"
    return name