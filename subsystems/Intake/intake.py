from commands2 import Subsystem
from wpilib import SmartDashboard, DigitalInput
from wpimath.controller import ArmFeedforward
import wpilib
from phoenix6 import hardware, configs, controls
from phoenix6.signals import GravityTypeValue
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
        self.limit_switch = DigitalInput(dioPort)
        self.voltage = ConstantValues.IntakeConstants.INTAKE_VOLTAGE

        self.arm_position_torque = controls.PositionTorqueCurrentFOC(0).with_slot(0)
        self.brake = controls.NeutralOut()

        self.goal_down = 0
        self.goal_up = 0.25

        self.cfg = configs.TalonFXConfiguration()
        self.cfg.slot0.k_p = ConstantValues.IntakeConstants.ARM_KP
        self.cfg.slot0.k_d = ConstantValues.IntakeConstants.ARM_KD
        self.cfg.slot0.k_i = ConstantValues.IntakeConstants.ARM_KI
        self.cfg.slot0.gravity_type = GravityTypeValue.ARM_COSINE
        self.cfg.slot0.k_g = ConstantValues.IntakeConstants.ARM_KG
        self.cfg.torque_current.peak_forward_torque_current = ConstantValues.IntakeConstants.ARM_PEAK_FORWARD_TORQUE_CURRENT
        self.cfg.torque_current.peak_reverse_torque_current = ConstantValues.IntakeConstants.ARM_PEAK_REVERSE_TORQUE_CURRENT
        self.arm_motor.configurator.apply(self.cfg)
        self.cfg.feedback.sensor_to_mechanism_ratio = 50

        self.intake_duty = controls.DutyCycleOut(0)
        self.arm_motor.set_position(0)

        self.current_goal_position = None

        self.feedforward = ArmFeedforward()
    def periodic(self):
        """
        rotation of the arm
        """
        if self.current_goal_position is not None:
            self.arm_motor.set_control(self.arm_position_torque.with_position(self.current_goal_position))          
    def intake(self):
        self.intake_motor.set_control(self.intake_duty.with_output(self.voltage))
    def stop_intake(self):
        self.intake_motor.set_control(0)
    def arm_up(self):
        self.current_goal_position = self.goal_up
    def arm_down(self):
        self.current_goal_position = self.goal_down
    def stop_arm(self):
        self.arm_motor.set_control(self.brake)