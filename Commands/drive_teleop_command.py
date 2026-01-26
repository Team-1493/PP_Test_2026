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


class DriveTeleopCommand(commands2.Command):    
    def __init__(
        self,
        _drivetrain: CommandSwerveDrivetrain,
        forward: typing.Callable[[], float],
        side: typing.Callable[[], float],
        rotate: typing.Callable[[], float],
    ) -> None:
        super().__init__()
        self.scale_factor = 1
        self.requested_velocity = 0
        self.forward = forward
        self.side = side
        self.rotate = rotate

        self.drivetrain = _drivetrain

        self.headingController = HeadingController.getInstance()

        # speed_at_12_volts desired top speed
        self._max_speed = (TunerConstants.speed_at_12_volts)  

        # 3/4 of a rotation per second max angular velocity
        self._max_angular_rate = rotationsToRadians(1)  

        # Setting up bindings for necessary control of the swerve drive platform
        self.requestFC = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.0025)  #squared input, so db starts at 0.05
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY
            ))
        

       # Setting up bindings for necessary control of the swerve drive platform
        self.requestFC_Facing = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._max_speed * 0.0025)  #squared input, so db starts at 0.05
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_heading_pid(
                ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KP,
                0,
                ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KD)    
                )

      
        self.addRequirements(self.drivetrain)


    def execute(self) -> None:
        forw=self.forward()
        sde=self.side()
        rot = self.rotate()

        if rot<0.05 and rot>-0.05: rot=0
        if forw<0.05 and forw>-0.05: forw=0
        if sde<0.05 and sde>-0.05: sde=0                
        # square input
        forw = copysign(forw**2,forw)
        sde = copysign(sde**2,sde)
        rot = copysign(rot**2,rot)

        self.requested_velocity=forw*self._max_speed*self.scale_factor
        SmartDashboard.putNumber("requested velocity", self.requested_velocity)

        state, target_angle = self.headingController.get_rotation_state(rot*self._max_angular_rate)    
    
        if state==0:
            self.drivetrain.set_control(
                self.requestFC.
                with_velocity_x(forw*self._max_speed*self.scale_factor).
                with_velocity_y(sde*self._max_speed*self.scale_factor).
                with_rotational_rate(rot*self._max_angular_rate*self.scale_factor))
        else:

            self.drivetrain.set_control(
                self.requestFC_Facing.
                with_velocity_x(forw*self._max_speed*self.scale_factor).
                with_velocity_y(sde*self._max_speed*self.scale_factor).
                with_target_direction(Rotation2d(target_angle) )
                )
        
