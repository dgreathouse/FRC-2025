package frc.robot.subsystems;

//import static edu.wpi.first.units.Units.Millimeter;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
//import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.CoralArmState;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.g;

public class Coral extends SubsystemBase implements IUpdateDashboard{
  TalonFXS m_leftMotor;
  TalonFXS m_rightMotor;
  SparkMax m_rotateMotor;
  //CANrange m_rangeSensor;
  PIDController m_rotatePID;
  PIDController m_spinnerPid;
  SimpleMotorFeedforward m_spinnerFF;
  ArmFeedforward m_rotateFF;
  VoltageOut m_leftVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  VoltageOut m_rightVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  VoltageOut m_rotateVoltageOut = new VoltageOut(0.0).withEnableFOC(true);

  /** Creates a new CoralIntake. */
  public Coral() {
    m_leftMotor = new TalonFXS(g.CAN_IDS_ROBORIO.CORAL_LEFT_MOTOR);
    m_rightMotor = new TalonFXS(g.CAN_IDS_ROBORIO.CORAL_RIGHT_MOTOR);
    m_rotateMotor = new SparkMax(g.CAN_IDS_ROBORIO.CORAL_ROTATE_MOTOR, MotorType.kBrushless);
    //m_rangeSensor = new CANrange(g.CAN_IDS_ROBORIO.CORAL_RANGE_SENSOR);

    m_rotatePID = new PIDController(7, 2, 0);
    m_rotatePID.setIZone(Math.toRadians(7.5));

    m_rotateFF = new ArmFeedforward(0, 0, 0);

    MotorOutputConfigs spinnerConfig = new MotorOutputConfigs();
    spinnerConfig.NeutralMode = NeutralModeValue.Brake;
    m_leftMotor.getConfigurator().apply(spinnerConfig);
    m_rightMotor.getConfigurator().apply(spinnerConfig);
    
    TalonFXSConfiguration toConfigure = new TalonFXSConfiguration();
    toConfigure.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    m_leftMotor.getConfigurator().apply(toConfigure);
    m_rightMotor.getConfigurator().apply(toConfigure);

    SparkMaxConfig maxConfig = new SparkMaxConfig();
    maxConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
    m_rotateMotor.configure(maxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    m_spinnerPid = new PIDController(.1, 0, 0);
    m_spinnerPid.setIZone(0.1);

    m_spinnerFF = new SimpleMotorFeedforward(0, g.ROBOT.MAX_BATTERY_SUPPLY_volts/g.CORAL.SPINNER_MOTOR_MAX_VELOCITY_rotPsec, 0);


    
    // FovParamsConfigs fovConfig = new FovParamsConfigs();
    // fovConfig.FOVCenterX = 1.0;
    // m_rangeSensor.getConfigurator().apply(fovConfig);
    g.DASHBOARD.updates.add(this);

  }
  /**
   * Spins the coral in at the desired speed
   * Does not spin if the coral is not detected
   * This uses a PID and Feedforward controller to spin the coral in at the desired speed
   * Hopefulle this will allow the coral to spin in and maintain it's position when a 0 speed is given.
   * 
   * @param _speed The speed to spin the coral in (-1.0 to 1.0)
   */
  public void spinIn(double _speed) {
    double volts = _speed * g.ROBOT.MAX_BATTERY_SUPPLY_volts;
    m_leftMotor.setControl(m_leftVoltageOut.withOutput(volts));
    m_rightMotor.setControl(m_rightVoltageOut.withOutput(-volts));
  }

  /**
   * Spins the coral out at the desired speed
   * 
   * @param _speed The speed to spin the coral out (-1.0 to 1.0)
   */
  public void spinOut ( CoralArmState _state) {
    double speed = 0;
    switch (_state){
      case ALGAE_HIGH:
      speed = 0.45;
        break;
      case ALGAE_LOW:
      speed = 0.45;
        break;
      case L1:
      speed = 0.15;
        break;
      case L2:
      speed = 0.25;
        break;
      case L3:
      speed = 0.15;
        break;
      case START:
        break;
      default:
        break;
      
    }
    m_leftMotor.setControl(m_leftVoltageOut.withOutput(speed * g.ROBOT.MAX_BATTERY_SUPPLY_volts));
    m_rightMotor.setControl(m_rightVoltageOut.withOutput(-speed * g.ROBOT.MAX_BATTERY_SUPPLY_volts));
  
  }
  /**
   * Rotates the arm to the desired state
   * @param _state The CoralArmState to rotate to as L1, L2, L3, ALGAE_HIGH, ALGAE_LOW, START
   */
  public void rotate(CoralArmState _state){
    switch (_state) { // TODO: adjust the angles for the levels
      case L1:
        rotateToAngle(55);
        break;
      case L2:
      rotateToAngle(45);
        break;
      case L3:
        rotateToAngle(20);
        break;
      case ALGAE_HIGH:
        rotateToAngle(55);
        break;
      case ALGAE_LOW:
        rotateToAngle(55);
        break;
      case START:
        rotateToAngle(-73);
        break;
      case LIFT_CLIMB_UP:
        rotateToAngle(55);
        break;
      case LIFT_CLIMB_DOWN:
        rotateToAngle(55);
        break;
      default:
        break;
    }
  }

/**
 * Rotates the arm to the desired angle
 * @param _angle_deg 
 *  */
  public void rotateToAngle(double _angle_deg){
    //double ff = m_rotateFF.calculate(Math.toRadians(_angle_deg), 0.1);
    double pid = m_rotatePID.calculate(Math.toRadians(getRotateAngle_deg()), Math.toRadians(_angle_deg));
    SmartDashboard.putNumber("Coral/pid", pid);
    pid = MathUtil.clamp(pid, -8, 8);
    m_rotateMotor.setVoltage(pid);
  }

  public double getRotateAngle_deg(){
    return m_rotateMotor.getEncoder().getPosition() * 360 / g.CORAL.ROTATE_GEAR_RATIO;
  }
/**
 *  Returns true if the coral is detected by the range sensor
 * @return boolean
 */
  // public boolean getIsDetected(){
  //   return m_rangeSensor.getIsDetected().getValue();
  // }
  /**
   * Returns the distance in mm from the range sensor
   * @return double
   */
  // public double getRange(){
  //   boolean status = m_rangeSensor.getIsDetected().getValue();
  //   return status ? m_rangeSensor.getDistance().getValue().in(Millimeter) : -1;
  // }

  @Override
  public void periodic() {

  }
  /**
   * Returns true if the arm is at the desired angle
   * @return boolean
   */
  public boolean isAtState(){
    return m_rotatePID.atSetpoint();
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putString("Coral/Coral Arm State", g.CORAL.armState.toString());
    SmartDashboard.putNumber("Coral/Coral Arm Angle", getRotateAngle_deg());
    // SmartDashboard.putNumber("Coral/RangeSensor_mm", getRange());
    // SmartDashboard.putBoolean("Coral/IsCoralDetected", getIsDetected());
    SmartDashboard.putNumber("Coral/Spinner Velocity", m_leftMotor.getVelocity().getValueAsDouble());
   
  }
}
