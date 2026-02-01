from Constants1 import ConstantValues
from math import pi, cos,sin
from wpimath.geometry import Pose2d, Rotation2d
import math

from subsystems.Drive.drivetrain_generator import DrivetrainGenerator

class HelperMethods():
    

    @staticmethod
    def calculate_pose_goal_from_tag(i : int, x_offset = 0, y_offset = 0):
        offset  = 0
        perspRot=DrivetrainGenerator.getInstance().get_operator_forward_direction().degrees()
        if perspRot==180:
            offset =16
        print("*****************  ",offset)
        poseTag = ConstantValues.VisionConstants.tags_list[i-1-offset].pose.toPose2d()
        rotTag = poseTag.rotation().radians()
        transTag = poseTag.translation()
        rotRobot=rotTag-pi

        poseGoal =  Pose2d(
            transTag.X()-x_offset*math.cos(rotTag)-y_offset*math.sin(rotTag),
            transTag.Y()-x_offset*math.sin(rotTag)+(y_offset)*math.cos(rotTag),
            Rotation2d(rotRobot))
        return (poseGoal)
