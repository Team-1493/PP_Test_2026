# all distances in meters, all angles in radians

from typing import override
import commands2
from AutoPilot_py.APTarget import ap_target
from AutoPilot_py.AP_Driver import ap_driver
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from Utilities.helper_methods import HelperMethods




class AutoPilotCommand(commands2.Command):

    def __init__(self,i:int, x_offset = 0, y_offset = 0,velFInal = 0):
        print("******* AUTOPILOT COMMAND   ********")
        self.i=i
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.velFinal = velFInal
        self.m_drivetrain = DrivetrainGenerator.getInstance()


    @override
    def initialize(self):
        print("****STARTING AUTOPILOT")
        pose=HelperMethods.calculate_pose_goal_from_tag(self.i,self.x_offset,self.y_offset)
        self.ap_drive = ap_driver.getInstance()
        self.m_target = (
            ap_target(pose)
            .with_entry_angle(pose.rotation())
            .with_velocity(0)
        )
  

    @override
    def execute(self):
        state = self.m_drivetrain.state
        robotRelativeSpeeds = state.speeds
        pose = state.pose

        out = self.ap_drive.kAutopilot.calculate(pose, robotRelativeSpeeds, self.m_target)

        self.m_drivetrain.drive_autopilot(out.vx,out.vy,out.targetAngle.radians())
  

    @override
    def isFinished(self):
        return self.ap_drive.kAutopilot.atTarget(
            self.m_drivetrain.pose, 
            self.m_target)
  

    @override
    def end(self,interrupted:bool):
        self.m_drivetrain.drive_RC(0,0,0)
        print("done with autopilot")
