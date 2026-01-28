import typing
import commands2
from math import copysign
from wpimath.geometry import Rotation2d
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from generated.tuner_constants import TunerConstants
from subsystems.Drive.heading_controller import HeadingController
from phoenix6 import swerve
from wpimath.units import rotationsToRadians
from wpilib import SmartDashboard
from  subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain
from Constants1 import ConstantValues
from wpimath.geometry import Pose2d


class SeedZero(commands2.Command):    
    def __init__(
        self,
        _drivetrain: CommandSwerveDrivetrain, 
        _headingController: HeadingController
        ) -> None:
        
        self.drivetrain = _drivetrain
        self.headingController = _headingController
        self.requestFC = (
            swerve.requests.FieldCentric()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY
            ).with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0))
        
      
        self.addRequirements(self.drivetrain)


    def execute(self) -> None:
        self.drivetrain.set_control(self.requestFC)
        self.headingController.zeroStateAngle()
        self.drivetrain.seed_field_centric()
        

    def end(self, interrupted: bool) :
        self.drivetrain.set_control(self.requestFC)

    def isFinished(self) -> bool:
        return True