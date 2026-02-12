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
from Commands.drive_path_generator import DrivePathGenerator

class arcDrive(commands2.Command):
    def __init__(self,
                _driveTrain:CommandSwerveDrivetrain,
                # _targetPose: typing.Callable[[], Pose2d],
                ) -> None:
        super().__init__()
        self.driveTrain = _driveTrain
        targetPose = Pose2d(4.6, 4.0, Rotation2d(0))  # Example fixed target pose
        x= 0
        y= 0
        angleToTarget = 0
        # goToTarg = DrivePathGenerator.drive_path_to_pose(Pose2d(x,y,angleToTarget), 0)

        # self.targetPose = _targetPose
        # self.addRequirements([self.driveTrain])
        self.isFinishedFlag = False

    def execute(self):
        currentPose = self.driveTrain.get_state().pose
        # targetPose = self.targetPose()
        radius = 2
        deltaX = 4.6 - (currentPose.X() + 0*(0.05 * self.driveTrain.get_state().speeds.vx))
        deltaY = 4 - (currentPose.Y() + 0*(0.05 * self.driveTrain.get_state().speeds.vy))

        angleToTarget = Rotation2d(wpimath.units.degrees(math.atan2(deltaY, deltaX)))

        print(deltaX, deltaY, angleToTarget.degrees())
        SmartDashboard.putNumber("Delta X", deltaX)
        SmartDashboard.putNumber("Delta Y", deltaY)
        SmartDashboard.putNumber("Angle to Target", angleToTarget.degrees())
        distanceToTarget = math.atan2(deltaX, deltaY)
        if distanceToTarget < radius - 0.02:
            omega = 1
            print(distanceToTarget)
        if distanceToTarget > radius + 0.02:
            omega = -1
        else:
            omega = 0
            print(distanceToTarget)

        self.driveTrain.set_control(
            swerve.requests.FieldCentricFacingAngle()
            .with_velocity_x(omega * 2)
            .with_velocity_y(omega * 2)
            .with_target_direction(angleToTarget)
        )
        