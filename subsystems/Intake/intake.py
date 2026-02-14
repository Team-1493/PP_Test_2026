from commands2 import Command, InstantCommand, Subsystem
from wpilib import SmartDashboard
import commands2
from wpimath.controller  import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from math import pi,hypot
from Constants1 import ConstantValues
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator 

class Intake(Subsystem):
    instance=None
    
    @staticmethod
    def getInstance():
        
        print('*' * 10 + ' Intake ' + '*' * 10)
    def __init__(self): pass
    def periodic(self): pass
