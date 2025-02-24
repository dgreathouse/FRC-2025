package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inch;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.CoralArmState;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.g;

public class Coral extends SubsystemBase implements IUpdateDashboard{
  TalonFXS m_leftMotor;
  TalonFXS m_rightMotor;
  TalonFX m_rotateMotor;
  CANrange m_rangeSensor;
  PIDController m_rotatePID;
  ArmFeedforward m_rotateFF;
  VoltageOut m_leftVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  VoltageOut m_rightVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  VoltageOut m_rotateVoltageOut = new VoltageOut(0.0).withEnableFOC(true);

  /** Creates a new CoralIntake. */
  public Coral() {
    m_leftMotor = new TalonFXS(g.CAN_IDS_ROBORIO.CORAL_LEFT_MOTOR);
    m_rightMotor = new TalonFXS(g.CAN_IDS_ROBORIO.CORAL_RIGHT_MOTOR);
    m_rotateMotor = new TalonFX(g.CAN_IDS_ROBORIO.CORAL_ROTATE_MOTOR);
    m_rangeSensor = new CANrange(g.CAN_IDS_ROBORIO.CORAL_RANGE_SENSOR);

    m_rotatePID = new PIDController(0, 0, 0);
    m_rotateFF = new ArmFeedforward(0, 0, 0);

    MotorOutputConfigs spinnerConfig = new MotorOutputConfigs();
    spinnerConfig.NeutralMode = NeutralModeValue.Brake;
    m_leftMotor.getConfigurator().apply(spinnerConfig);
    m_rightMotor.getConfigurator().apply(spinnerConfig);
    
    m_rotateMotor.setPosition(0.0);  // TODO: set offset angle for start position.


    
    FovParamsConfigs fovConfig = new FovParamsConfigs();
    fovConfig.FOVCenterX = 1.0;
    m_rangeSensor.getConfigurator().apply(fovConfig);
     g.DASHBOARD.updates.add(this);

  }
  public void spinIn(){

    m_leftMotor.setControl(m_leftVoltageOut.withOutput(g.CORAL.spinSpeed * g.ROBOT.MAX_BATTERY_SUPPLY_volts));
    m_rightMotor.setControl(m_rightVoltageOut.withOutput(-g.CORAL.spinSpeed * g.ROBOT.MAX_BATTERY_SUPPLY_volts));
  }
  public void spinOut(){
    m_leftMotor.setControl(m_leftVoltageOut.withOutput(-g.CORAL.spinSpeed * g.ROBOT.MAX_BATTERY_SUPPLY_volts));
    m_rightMotor.setControl(m_rightVoltageOut.withOutput(g.CORAL.spinSpeed * g.ROBOT.MAX_BATTERY_SUPPLY_volts));
  }
  public void spin(double _speed){
    m_leftMotor.setControl(m_leftVoltageOut.withOutput(_speed * g.ROBOT.MAX_BATTERY_SUPPLY_volts));
    m_rightMotor.setControl(m_rightVoltageOut.withOutput(-_speed * g.ROBOT.MAX_BATTERY_SUPPLY_volts));

  }
  public void rotate(CoralArmState _state){
    switch (_state) { // TODO: adjust the angles for the levels
      case L1:
        rotateToAngle(45);
        break;
      case L2:
      rotateToAngle(45);
        break;
      case L3:
      rotateToAngle(45);
        break;
      case L4:
      rotateToAngle(45);
        break;
      case START:
      rotateToAngle(45);
        break;
      default:
        break;
    }
  }
  public void rotateToAngle(double _angle_deg){
    double ff = m_rotateFF.calculate(Math.toRadians(_angle_deg), 0.1);
    double pid = m_rotatePID.calculate(Math.toRadians(getRotateAngle_deg()), Math.toRadians(_angle_deg));
    m_rotateMotor.setControl(m_rotateVoltageOut.withOutput(pid + ff));
  }
  public double getRotateAngle_deg(){

    return m_rotateMotor.getPosition().getValueAsDouble() * 360 * g.CORAL.ROTATE_GEAR_RATIO;
  }
  public double getRange(){

    return m_rangeSensor.getDistance().getValue().in(Inch);
  }
  @Override
  public void periodic() {

  }
  public boolean isAtState(){

    return m_rotatePID.atSetpoint();
  }
  @Override
  public void updateDashboard() {
    SmartDashboard.putString("Coral/Claw Arm State", g.CORAL.armState.toString());
    SmartDashboard.putNumber("Coral/Arm Angle", getRotateAngle_deg());
  //  SmartDashboard.putNumber("Coral/Arm Rotations",m_rotateMotor.getPosition().getValueAsDouble() / g.CORAL.ROTATE_GEAR_RATIO);
  }
}
