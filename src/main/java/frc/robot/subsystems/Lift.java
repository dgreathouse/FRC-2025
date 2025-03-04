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
  double m_maxUpSpeed_volts = 12;
  double m_maxDownSpeed_volts = -10;
  VoltageOut m_voltageOut;
  /** Creates a new ScissorLift. */
  public Lift() {
    m_motor = new TalonFX(10, g.CAN_IDS_ROBORIO.NAME); // Creates a new TalonFX.
    m_voltageOut = new VoltageOut(0).withEnableFOC(true); // Creates a new VoltageOut.
    m_pid = new PIDController(1.5, 0.5, 0); // Creates a new PID Controller.
    m_pid.setIZone(20); // Sets the IZone range.
    m_pid.setIntegratorRange(-.1, .1); // Sets the Integrator range.
    m_pid.setTolerance(0.1); // Sets the tolerance
    
    
    MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs(); // Creates new MotorOutputConfigs.
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
      moveToPosition(390);
        break;
      case ALGAE_HIGH:
        moveToPosition(400);
        break;
      case ALGAE_LOW:
        moveToPosition(100);
        break;
      case START:
        moveToPosition(0);
        break;
        case LIFT_CLIMB_UP:
        moveToPosition(350);  
        break;
        case LIFT_CLIMB_DOWN:
        moveToPosition(100);    
        break;
      default:
        break;
    }
  }

  public void moveToPosition(double _pos_mm) {
    double volts = m_pid.calculate(getPosition_mm(), _pos_mm);
    volts = MathUtil.clamp(volts, m_maxDownSpeed_volts, m_maxUpSpeed_volts);
    // if(getPosition_mm() < 20 && volts < 0) {
    //   volts = 0;
    // }
    moveWithVoltage(m_kG + volts);
  }
  public double getPosition_mm(){
    return m_motor.getPosition().getValueAsDouble() / g.LIFT.MOTOR_ROTATIONS_TO_LIFT_DISTANCE_rotPmm;
  } 
  public void moveWithVoltage(double _volts){
    m_motor.setControl(m_voltageOut.withOutput(_volts));

  }
  public boolean isAtState(){
    return m_pid.atSetpoint();
  }
  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Lift/Distance_mm", getPosition_mm());
    SmartDashboard.putNumber("Lift/Volts", m_voltageOut.Output);
    
  }
}
