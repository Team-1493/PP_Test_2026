from commands2 import Subsystem
from wpilib import SmartDashboard
from phoenix6 import hardware, configs, controls
from Constants1 import ConstantValues

class Intake(Subsystem):
    instance=None
    
    @staticmethod
    def getInstance():
        if Intake.instance == None:
            Intake.instance = Intake()
            print('*' * 22 + ' INTAKE ' + '*' * 22)
        return Intake.instance
    def __init__(self, intakeMotorID, armMotorID, dioPort):
        """
        Initialize PID constants for motors
        """
        Subsystem.__init__(self)

        self.intake_motor = hardware.TalonFX(intakeMotorID)
        self.arm_motor = hardware.TalonFX(armMotorID)
        self.voltage = ConstantValues.IntakeConstants.INTAKE_VOLTAGE

        self.arm_position_torque = controls.PositionTorqueCurrentFOC(0).with_slot(0)
        self.brake = controls.NeutralOut()

        self.cfg = configs.TalonFXConfiguration()
        self.cfg.slot0.k_p = ConstantValues.IntakeConstants.ARM_KP
        self.cfg.slot0.k_d = ConstantValues.IntakeConstants.ARM_KD
        self.cfg.slot0.k_i = ConstantValues.IntakeConstants.ARM_KI
        self.cfg.torque_current.peak_forward_torque_current = ConstantValues.IntakeConstants.ARM_PEAK_FORWARD_TORQUE_CURRENT
        self.cfg.torque_current.peak_reverse_torque_current = ConstantValues.IntakeConstants.ARM_PEAK_REVERSE_TORQUE_CURRENT
        self.arm_motor.configurator.apply(self.cfg)

        self.intake_duty = controls.DutyCycleOut(0)
        self.arm_motor.set_position(0)
    def periodic(self):
        """
        Show the rotation of the arm
        """
    def intake(self):
        self.intake_motor.set_control(self.intake_duty.with_output(self.voltage))
    def stop_intake(self):
        self.intake_motor.set_control(0)
    def arm_up(self):
        
    def arm_down(self):
        pass
    def stop_arm(self):
        self.arm_motor.set_control(self.brake)