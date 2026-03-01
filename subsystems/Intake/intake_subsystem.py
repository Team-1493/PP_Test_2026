from commands2 import Subsystem
from wpilib import SmartDashboard, DigitalInput
import wpilib
from phoenix6 import hardware, configs, controls
from phoenix6.signals import GravityTypeValue
from Constants1 import ConstantValues

class IntakeSystem(Subsystem):
    instance=None
    
    @staticmethod
    def getInstance():
        if IntakeSystem.instance == None:
            IntakeSystem.instance = IntakeSystem()
            print('*' * 22 + ' INTAKE ' + '*' * 22)
        return IntakeSystem.instance
    def setup(self):
        self.voltage = ConstantValues.IntakeConstants.INTAKE_VOLTAGE

        self.goal_down = ConstantValues.IntakeConstants.MAX_DOWN_ROTATION
        self.goal_up = ConstantValues.IntakeConstants.MAX_UP_ROTATION

        self.cfg = configs.TalonFXConfiguration()
        self.cfg.slot0.k_p = ConstantValues.IntakeConstants.ARM_KP
        self.cfg.slot0.k_d = ConstantValues.IntakeConstants.ARM_KD
        self.cfg.slot0.k_i = ConstantValues.IntakeConstants.ARM_KI
        self.cfg.slot0.gravity_type = GravityTypeValue.ARM_COSINE
        self.cfg.slot0.k_g = ConstantValues.IntakeConstants.ARM_KG
        self.cfg.torque_current.peak_forward_torque_current = ConstantValues.IntakeConstants.ARM_PEAK_FORWARD_TORQUE_CURRENT
        self.cfg.torque_current.peak_reverse_torque_current = ConstantValues.IntakeConstants.ARM_PEAK_REVERSE_TORQUE_CURRENT
        self.cfg.feedback.sensor_to_mechanism_ratio = ConstantValues.IntakeConstants.SENSOR_TO_MECHANISM_RATIO
        
    def __init__(self, intakeMotorID, armMotorID, dioPortUp, dioPortDown):
        """
        Initialize PID constants for motors
        """
        Subsystem.__init__(self)

        self.intake_motor = hardware.TalonFX(intakeMotorID)
        self.arm_motor = hardware.TalonFX(armMotorID)
        self.up_limit_switch = DigitalInput(dioPortUp)
        self.down_limit_switch = DigitalInput(dioPortDown)

        self.arm_position_torque = controls.PositionTorqueCurrentFOC(0).with_slot(0)
        self.brake = controls.NeutralOut()

        self.setup()

        self.arm_motor.configurator.apply(self.cfg)

        self.intake_duty = controls.DutyCycleOut(0)
        self.arm_motor.set_position(0)

        self.current_goal_position = None
    def periodic(self):
        """
        rotation of the arm
        """
        if self.up_limit_switch.get() and self.current_goal_position == self.goal_up:
            self.stop_arm()
        if self.down_limit_switch.get() and self.current_goal_position == self.goal_down:
            self.stop_arm()
        if self.current_goal_position is not None:
            self.arm_motor.set_control(self.arm_position_torque.with_position(self.current_goal_position))          
    def intake(self):
        self.intake_motor.set_control(self.intake_duty.with_output(self.voltage))
    def stop_intake(self):
        self.intake_motor.set_control(0)
    def stop_arm(self):
        self.arm_motor.set_control(self.brake)
    def arm_up(self):
        self.current_goal_position = self.goal_up
        # self.stop_intake()
        # self.periodic()
    def arm_down(self):
        self.current_goal_position = self.goal_down
        # self.periodic()
        # if self.down_limit_switch:
        #     self.intake()