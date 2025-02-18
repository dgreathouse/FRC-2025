package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.CoralArmState;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.g;

public class Lift extends SubsystemBase implements IUpdateDashboard{
  TalonFX m_motor;
  PIDController m_pid;
  double m_kG = 0.0;
  double m_maxSpeed_volts = 5;
  VoltageOut m_voltageOut;
  /** Creates a new ScissorLift. */
  public Lift() {
    g.DASHBOARD.updates.add(this);
    m_motor = new TalonFX(10, g.CAN_IDS_ROBORIO.NAME);
    m_voltageOut = new VoltageOut(0).withEnableFOC(true);
    m_pid = new PIDController(0.1, 0.1, 0);
    m_pid.setIZone(20);
    m_pid.setIntegratorRange(-.1, .1);
    m_pid.setTolerance(0.1);
    
    
    MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs();
    motorOutputConfig.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
    m_motor.getConfigurator().apply(motorOutputConfig);

    OpenLoopRampsConfigs openLoopRampsConfig = new OpenLoopRampsConfigs();
    openLoopRampsConfig.VoltageOpenLoopRampPeriod = 0.0;
    m_motor.getConfigurator().apply(openLoopRampsConfig);

    g.DASHBOARD.updates.add(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * This will move to a position based on the Coral and Algae arm states.
   * Coral is higher priority at the beginning.
   * 
   */
  public void moveToPosition(CoralArmState _state){
    switch (_state) {
      case L1:
      moveToPosition(0);
        break;
      case L2:
      moveToPosition(0);
        break;
      case L3:
      moveToPosition(100);
        break;
      case L4:
      moveToPosition(200);
        break;
      case START:
        // TODO Add logic for algae if algae is not at START
        moveToPosition(0);
        break;
      default:
        break;

    }
  }

  public void moveToPosition(double _pos_mm) {
    double volts = m_pid.calculate(getPosition_mm(), _pos_mm);
    volts = MathUtil.clamp(volts, -m_maxSpeed_volts, m_maxSpeed_volts);
    moveWithVoltage(m_kG + volts);

  }
  public double getPosition_mm(){
    return m_motor.getPosition().getValueAsDouble() / g.LIFT.MOTOR_ROTATIONS_TO_LIFT_DISTANCE_rotPmm;
  } 
  public void moveWithVoltage(double _volts){
    m_motor.setControl(m_voltageOut.withOutput(_volts));

  }
  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Lift/Distance_mm", getPosition_mm());
    SmartDashboard.putNumber("Lift/Volts", m_voltageOut.Output);
  }
}
