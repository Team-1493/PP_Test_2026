import math
from typing import override
import typing
import commands2
from wpilib import SmartDashboard
import wpimath
from math import copysign
from generated.tuner_constants import TunerConstants
from Commands.stop_drive import StopDrive
from subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from phoenix6 import swerve
from wpimath.units import rotationsToRadians

from wpimath.geometry import Pose2d,Rotation2d

class DriveWhileShooting(commands2.Command):
    def __init__(self,
                _driveTrain:CommandSwerveDrivetrain,
                 forward: typing.Callable[[], float],
                 side: typing.Callable[[], float],
                # _targetPose: typing.Callable[[], Pose2d],
                ) -> None:
        super().__init__()
        self.driveTrain = _driveTrain
        self.forward = forward
        self.side = side
        self._max_speed = (TunerConstants.speed_at_12_volts)  
        self._max_angular_rate = rotationsToRadians(1)  
        targetPose = Pose2d(4.6, 4.0, Rotation2d(0))  # Example fixed target pose
        self.requestFC = (
            swerve.requests.FieldCentricFacingAngle()
            .with_heading_pid(4,0,0)
            .with_deadband(self._max_speed * 0.0025)  #squared input, so db starts at 0.05
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY
            ))

        # self.targetPose = _targetPose
        # self.addRequirements([self.driveTrain])
        self.isFinishedFlag = False

    def execute(self):
        currentPose = self.driveTrain.get_state().pose
        # targetPose = self.targetPose()
        forw=self.forward()
        sde=self.side()
        if forw<0.05 and forw>-0.05: forw=0
        if sde<0.05 and sde>-0.05: sde=0      
        forw = math.copysign(forw**2,forw)
        sde = math.copysign(sde**2,sde)
        deltaX = 4.6 - (currentPose.X() + (0.1 * self.driveTrain.get_state().speeds.vx))
        deltaY = 4 - (currentPose.Y() + (0.1 * self.driveTrain.get_state().speeds.vy))

        angleToTarget = Rotation2d(math.atan2(deltaY, deltaX))
        print(deltaX, deltaY, angleToTarget.degrees())
        SmartDashboard.putNumber("Delta X", deltaX)
        SmartDashboard.putNumber("Delta Y", deltaY)
        SmartDashboard.putNumber("Angle to Target", angleToTarget.degrees())
        self.driveTrain.set_control(
            self.requestFC
            .with_velocity_x(forw*self._max_speed)
            .with_velocity_y(sde*self._max_speed)
            .with_target_direction(angleToTarget)
        )