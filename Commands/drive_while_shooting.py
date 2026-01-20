import math
from typing import override
import typing
import commands2
from wpilib import SmartDashboard
import wpimath
from Commands.stop_drive import StopDrive
from subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from phoenix6 import swerve
from wpimath.geometry import Pose2d,Rotation2d

class DriveWhileShooting(commands2.Command):
    def __init__(self,
                _driveTrain:CommandSwerveDrivetrain,
                # _targetPose: typing.Callable[[], Pose2d],
                ) -> None:
        super().__init__()
        self.driveTrain = _driveTrain
        targetPose = Pose2d(4.6, 4.0, Rotation2d(0))  # Example fixed target pose

        # self.targetPose = _targetPose
        # self.addRequirements([self.driveTrain])
        self.isFinishedFlag = False

    def execute(self):
        currentPose = self.driveTrain.get_state().pose
        # targetPose = self.targetPose()

        deltaX = 4.6 - (currentPose.X() + 0*(0.05 * self.driveTrain.get_state().speeds.vx))
        deltaY = 4 - (currentPose.Y() + 0*(0.05 * self.driveTrain.get_state().speeds.vy))

        angleToTarget = Rotation2d(wpimath.units.degrees(math.atan2(deltaY, deltaX)))

        print(deltaX, deltaY, angleToTarget.degrees())
        SmartDashboard.putNumber("Delta X", deltaX)
        SmartDashboard.putNumber("Delta Y", deltaY)
        SmartDashboard.putNumber("Angle to Target", angleToTarget.degrees())
        self.driveTrain.set_control(
            swerve.requests.FieldCentricFacingAngle()
            # .with_velocity_x(self.driveTrain.get_state().speeds.vx)
            # .with_velocity_y(self.driveTrain.get_state().speeds.vy)
            .with_target_direction(angleToTarget)
        )