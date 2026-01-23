import math
from typing import override
import commands2
from wpilib import SmartDashboard, Timer
import wpimath
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from phoenix6 import swerve
from  subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain

class FindkS(commands2.Command):
    def __init__(self):
        self.drivetrain = DrivetrainGenerator.getInstance()

      
        self.addRequirements(self.drivetrain)
        self.timer = Timer()
        self.voltageRate = 0.01  #ramp rate volts per second
        self.driveKS = swerve.requests.SysIdSwerveTranslation()
        
    @override
    def initialize(self):
        self.timeCurrent = 0
        self.voltage = 0
        self.timer.reset()
        self.timer.start()
        self.timePrevious = self.timer.get()
        self.kS = 0
        self.avgVolt=0
        self.avgTorqueCurrnet=0        
        SmartDashboard.putNumber("Drive kS - measured",0)
        SmartDashboard.putNumber("Torqe Current- measured",0)
        SmartDashboard.putNumber("Torqe Current- measured (module 0)",0)
        SmartDashboard.putNumber("Torqe Current- measured (module 1)",0)
        SmartDashboard.putNumber("Torqe Current- measured (module 2)",0)
        SmartDashboard.putNumber("Torqe Current- measured (module 3)",0)

        SmartDashboard.putNumber("Voltage- measured (module 0)",0)
        SmartDashboard.putNumber("Voltage- measured (module 1)",0)
        SmartDashboard.putNumber("Voltage- measured (module 2)",0)
        SmartDashboard.putNumber("Voltage- measured (module 3)",0)        
        self.haskS = False

    def execute(self) -> None:
        timeCurrent = self.timer.get()
        deltaTime = timeCurrent - self.timePrevious
        self.voltage += deltaTime*self.voltageRate


        self.drivetrain.set_control(self.driveKS.with_volts(self.voltage))
        chassisSpeed = self.drivetrain.get_state().speeds
        self.spd = math.sqrt(chassisSpeed.vx**2+chassisSpeed.vy**2)
        if spd>.00125:
            self.haskS = True

        if not self.haskS:
            mod0 = self.get_torque_current(0)
            mod1 = self.get_torque_current(1)
            mod2 = self.get_torque_current(2)
            mod3 = self.get_torque_current(3)
            
            volt0 = self.get_voltage(0)
            volt1 = self.get_voltage(1)
            volt2 = self.get_voltage(2)
            volt3 = self.get_voltage(3)

            self.avgVolt=(volt0+volt1+volt2+volt3)/4
            self.avgTorqueCurrnet = (mod0+mod1+mod2+mod3)/4
            SmartDashboard.putNumber("Drive kS - measured",self.voltage)
            SmartDashboard.putNumber("Torqe Current- measured (module 0)",mod0)
            SmartDashboard.putNumber("Torqe Current- measured (module 1)",mod1)
            SmartDashboard.putNumber("Torqe Current- measured (module 2)",mod2)
            SmartDashboard.putNumber("Torqe Current- measured (module 3)",mod3)
            SmartDashboard.putNumber("Average Torque Current", self.avgTorqueCurrnet) 

            SmartDashboard.putNumber("Voltage- measured (module 0)",volt0)
            SmartDashboard.putNumber("Voltage- measured (module 1)",volt1)
            SmartDashboard.putNumber("Voltage- measured (module 2)",volt2)
            SmartDashboard.putNumber("Voltage- measured (module 3)",volt3)
            SmartDashboard.putNumber("Average Voltage", self.avgVolt)       

    @override
    def end(self, interrupted: bool) :
        self.drivetrain.set_control(self.driveKS.with_volts(0))

    @override
    def isFinished(self):
        return self.spd>0.5
    

        
    def get_voltage(self,module):
        return abs(self.drivetrain.get_module(module).drive_motor.get_motor_voltage().value_as_double)
    
    def get_torque_current(self,module):
        return abs(self.drivetrain.get_module(module).drive_motor.get_torque_current().value_as_double)
