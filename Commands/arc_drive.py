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
from pathplannerlib.config  import PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController 
from pathplannerlib.trajectory import PathPlannerTrajectoryState;
from wpimath.kinematics import ChassisSpeeds



class arcDrive(commands2.Command):
    def __init__(self,
                _driveTrain:CommandSwerveDrivetrain
                # _targetPose: typing.Callable[[], Pose2d],
                ) -> None:
        super().__init__()
        self.driveTrain = _driveTrain
        self.xh = 4.644
        self.yh= 4.030
        self.radius = 2

        self.isFinishedFlag = False
        self.addRequirements(self.driveTrain)
        self.kTranslationPID = PIDConstants(5.0,0,0)
        self.kRotationPID = PIDConstants(5.0,0,0)
        self.controller = PPHolonomicDriveController(
            self.kTranslationPID, self.kRotationPID)

        
   
        

    @override
    def initialize(self):
        currentPose = self.driveTrain.get_state().pose
        xr = currentPose.X()
        yr = currentPose.Y()
        yr1=-(self.xh - xr)
        xr1 = self.yh - yr

        mi = -yr1/xr1
        self.x_goal=self.xh - (self.radius/math.sqrt(1+mi*mi)) 
        self.y_goal=self.yh - (mi*self.x_goal)
        self.angle_goal=math.atan2(-yr1,xr1)
        self.goalPose = Pose2d(self.x_goal,self.y_goal,Rotation2d(self.angle_goal)) 
        self.goalState = PathPlannerTrajectoryState()
        self.goalState.pose = self.goalPose
        print(xr,"  ",yr,"  ",xr1,"  ",yr1,"  ",mi,"   ",self.x_goal,"  ",self.y_goal,"  ",self.angle_goal )

    def execute(self):
        speeds =self.controller.calculateRobotRelativeSpeeds(
            self.driveTrain.get_pose(), self.goalState)

        speeds = ChassisSpeeds(min(speeds.vx, 1.0),min(speeds.vy, 1.0),
                speeds.omega)

        self.driveTrain.drive_RC(speeds.vx,speeds.vy,speeds.omega)