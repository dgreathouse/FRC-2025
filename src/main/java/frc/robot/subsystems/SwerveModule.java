package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.SimplerMotorFeedforward;
import frc.robot.lib.g;

/** Add your docs here. */
public class SwerveModule implements IUpdateDashboard {
  // SwerveModuleConstants m_k;
  public String m_name = "";
  public Translation2d m_location;
  private TalonFX m_driveMotor;
  private TalonFX m_steerMotor;
  private CANcoder m_canCoder;
  private SwerveModulePosition m_position = new SwerveModulePosition();
  // TODO Tune Steer PID kP, kI, kD
  private PIDController m_steerPID = new PIDController(g.SWERVE.STEER.PID_KP, g.SWERVE.STEER.PID_KI, 0);
  // TODO: Tune Drive PID kP, kI, kD
  private PIDController m_drivePID = new PIDController(g.SWERVE.DRIVE.PID_KP, g.SWERVE.DRIVE.PID_KI, 0);
  // TODO Tune Drive FF kV, kS, and kA
  private SimplerMotorFeedforward m_driveFF = new SimplerMotorFeedforward(g.SWERVE.DRIVE.PID_KS, g.SWERVE.DRIVE.PID_KV, 0.0);
  private VoltageOut m_steerVoltageOut = new VoltageOut(0.0).withEnableFOC(true).withOverrideBrakeDurNeutral(true);
  private VoltageOut m_driveVoltageOut = new VoltageOut(0.0).withEnableFOC(true).withOverrideBrakeDurNeutral(true);
  private StatusSignal<Angle> m_drivePosition;
  private StatusSignal<AngularVelocity> m_driveVelocity;
  private StatusSignal<Angle> m_steerPosition;
  private StatusSignal<AngularVelocity> m_steerVelocity;


  public SwerveModule(
      String _name,
      int _driveCanId,
      boolean _driveIsReversed,
      int _steerCanId,
      boolean _steerIsReversed,
      int _canCoderId,
      double _canCoderOffset_rot,
      double _locationX_m,
      double _locationY_m) {
    StatusCode status;
    m_name = _name;
    m_location = new Translation2d(_locationX_m, _locationY_m);
    m_driveMotor = new TalonFX(_driveCanId, g.CAN_IDS_CANIVORE.NAME);
    m_steerMotor = new TalonFX(_steerCanId, g.CAN_IDS_CANIVORE.NAME);
    m_canCoder = new CANcoder(_canCoderId, g.CAN_IDS_CANIVORE.NAME);

    MotorOutputConfigs driveMotorOutputConfig = new MotorOutputConfigs();
    driveMotorOutputConfig.NeutralMode = NeutralModeValue.Brake;
    driveMotorOutputConfig.Inverted = _driveIsReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    status = m_driveMotor.getConfigurator().apply(driveMotorOutputConfig);
    System.out.println(m_name + " Drive Motor Output Config Status =" + status.toString());


    OpenLoopRampsConfigs driveOpenLoopRampsConfig = new OpenLoopRampsConfigs();
    driveOpenLoopRampsConfig.VoltageOpenLoopRampPeriod = 0.01;
    m_driveMotor.getConfigurator().apply(driveOpenLoopRampsConfig);
    System.out.println(m_name + " Drive Motor OpenLoopRamps Config Status =" + status.toString());

    CurrentLimitsConfigs driveCurrentConfig = new CurrentLimitsConfigs();
    driveCurrentConfig.StatorCurrentLimitEnable = true;
    driveCurrentConfig.StatorCurrentLimit = g.SWERVE.DRIVE.STATOR_CURRENT_LIMIT_amps;
    driveCurrentConfig.SupplyCurrentLimitEnable = true;
    driveCurrentConfig.SupplyCurrentLimit = g.SWERVE.DRIVE.SUPPLY_CURRENT_LIMIT_amps;   
    status = m_driveMotor.getConfigurator().apply(driveCurrentConfig);
    System.out.println(m_name + " Drive Motor Current Config Status =" + status.toString());

    m_drivePosition = m_driveMotor.getPosition();
    m_drivePosition.setUpdateFrequency(g.SWERVE.CAN_UPDATE_FREQ_hz);
    m_driveVelocity = m_driveMotor.getVelocity();
    m_driveVelocity.setUpdateFrequency(g.SWERVE.CAN_UPDATE_FREQ_hz);


    // Configure Steer Motor
    MotorOutputConfigs steerMotorOutputConfigs = new MotorOutputConfigs();
    steerMotorOutputConfigs.Inverted = _steerIsReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    status = m_steerMotor.getConfigurator().apply(steerMotorOutputConfigs);
    System.out.println(m_name + " Steer Motor Output Config Status =" + status.toString());

    OpenLoopRampsConfigs steerOpenLoopRampsConfigs = new OpenLoopRampsConfigs();
    steerOpenLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.01;
    status = m_steerMotor.getConfigurator().apply(steerOpenLoopRampsConfigs);
    System.out.println(m_name + " Steer Motor Open Loop Ramp Config Status =" + status.toString());

    CurrentLimitsConfigs steerCurrentLimitConfig = new CurrentLimitsConfigs();
    steerCurrentLimitConfig.StatorCurrentLimitEnable = true;
    steerCurrentLimitConfig.StatorCurrentLimit = g.SWERVE.STEER.STATOR_CURRENT_LIMIT_amps;
    steerCurrentLimitConfig.SupplyCurrentLimitEnable = true;
    steerCurrentLimitConfig.SupplyCurrentLimit = g.SWERVE.STEER.SUPPLY_CURRENT_LIMIT_amps;
    status = m_steerMotor.getConfigurator().apply(steerCurrentLimitConfig);
    System.out.println(m_name + " Steer Motor Current Limit Config Status =" + status.toString());

    // Configure CANCoder
    MagnetSensorConfigs magnetSensorConfig = new MagnetSensorConfigs();
    magnetSensorConfig.MagnetOffset = _canCoderOffset_rot;
    m_canCoder.getConfigurator().apply(magnetSensorConfig);
    status = m_canCoder.getConfigurator().apply(magnetSensorConfig);
    System.out.println(m_name + " CANCOder Magnet Config Status =" + status.toString());

    // Set the offset position of the steer motor based on the CANCoder
    m_steerMotor.setPosition(m_canCoder.getPosition().getValueAsDouble() * g.SWERVE.STEER.GEAR_RATIO);

    m_steerPosition = m_steerMotor.getPosition();
    m_steerPosition.setUpdateFrequency(g.SWERVE.CAN_UPDATE_FREQ_hz);
    m_steerVelocity = m_steerMotor.getVelocity();
    m_steerVelocity.setUpdateFrequency(g.SWERVE.CAN_UPDATE_FREQ_hz);

    m_steerPID.enableContinuousInput(-180.0, 180.0);
    m_steerPID.setIZone(20);
    m_steerPID.setTolerance(0.1);
    m_steerPID.setIntegratorRange(-5, 5);

 
  }



  public SwerveModulePosition updatePosition() {
    double drive_rot = m_driveMotor.getPosition().getValueAsDouble();
    double angle_rot = m_steerMotor.getPosition().getValueAsDouble();
    // anagle_rot is the Motor rotations. Apply the gear ratio to get wheel
    // rotations for steer
    angle_rot = angle_rot / g.SWERVE.STEER.GEAR_RATIO;
    /* And push them into a SwerveModuleState object to return */
    // WHEEL_MotRotPerMeter
    m_position.distanceMeters = drive_rot / g.SWERVE.DRIVE.MOTOR_ROTATIONS_TO_WHEEL_DISTANCE_rotPm;
    /* Angle is already in terms of steer rotations */
    m_position.angle = Rotation2d.fromRotations(angle_rot);

    return m_position;
  }

  public double getSteerActualAngle_deg() {
    return m_position.angle.getDegrees();
  }

  public void setDesiredState(SwerveModuleState _state) {
    _state.optimize(m_position.angle);
    // TODO: Cosine compensation needs testing
    // _state.speedMetersPerSecond *=
    // _state.angle.minus(g.ROBOT.angleActual_Rot2d).getCos();
    
    /*-------------------- Steer---------------------*/
    if (g.SWERVE.isEnabled) {
      double steerVolts = m_steerPID.calculate(m_position.angle.getDegrees(), _state.angle.getDegrees());
      
      m_steerMotor.setControl(m_steerVoltageOut.withOutput(steerVolts));
      SmartDashboard.putNumber("Swerve/SteerVolts "+ m_name, steerVolts);
      SmartDashboard.putNumber("Swerve/SteerActualAng " + m_name, m_position.angle.getDegrees() % 180);
      SmartDashboard.putNumber("Swerve/SteerSetAng " + m_name,_state.angle.getDegrees());
    } else {
      m_steerMotor.setControl(m_steerVoltageOut.withOutput(0));
    }
    /*-------------------- Drive---------------------*/
    if (g.SWERVE.isEnabled) {

      double driveSetVelocity_mps = _state.speedMetersPerSecond;
      double driveErrorVolts = m_drivePID.calculate(m_driveMotor.getVelocity().getValueAsDouble() / g.SWERVE.DRIVE.MOTOR_ROTATIONS_TO_WHEEL_DISTANCE_rotPm, driveSetVelocity_mps);
      driveErrorVolts = MathUtil.clamp(driveErrorVolts, -6, 6); // Limit the amount the PID can contribute

      double driveVolts = driveErrorVolts + m_driveFF.calculate(driveSetVelocity_mps, 0.0);
      //driveVolts = m_driveFF.calculate(driveSetVelocity_mps, 0.0);
      SmartDashboard.putNumber("Swerve/DriveVolts "+this.m_name, driveVolts);
      SmartDashboard.putNumber("Swerve/DriveActualVelocity" + m_name, m_driveMotor.getVelocity().getValueAsDouble() / g.SWERVE.DRIVE.MOTOR_ROTATIONS_TO_WHEEL_DISTANCE_rotPm);
      SmartDashboard.putNumber("Swerve/DriveSetVelocity" + m_name, driveSetVelocity_mps);
      m_driveMotor.setControl(m_driveVoltageOut.withOutput(driveVolts));
    } else {
      m_driveMotor.setControl(m_driveVoltageOut.withOutput(0));
    }

  }

  public double getDriveCurrent() {
    return m_driveMotor.getTorqueCurrent().getValueAsDouble();
  }

  public double getSteerCurrent() {
    return m_steerMotor.getTorqueCurrent().getValueAsDouble();
  }
  /**
   * 
   * @return Drive speed in MPS
   */
  public double getDriveSpeed(){
    return Math.abs(m_driveMotor.getVelocity().getValueAsDouble() / g.SWERVE.DRIVE.MOTOR_ROTATIONS_TO_WHEEL_DISTANCE_rotPm);
  }
    /**
   * Called by separate thread to put stuff to the dashboard at a slower rate than the main periodic
   */
  public void updateDashboard() {
    // SmartDashboard.putNumber(
    //     "Swerve/" + this.m_name + "/Vel(ftPsec)",
    //     (m_driveMotor.getVelocity().getValueAsDouble()
    //             / // Rot/sec /
    //             g.SWERVE.DRIVE.MOTOR_ROTATIONS_TO_WHEEL_DISTANCE_rotPm) // Rot/m
    //         * g.CV.MPS_TO_FEETPERSEC); // ft/s
    // SmartDashboard.putNumber("Swerve/" + this.m_name + "/Drive Current", getDriveCurrent());
    // SmartDashboard.putNumber("Swerve/" + this.m_name + "/Steer Current", getSteerCurrent());
    // SmartDashboard.putData("Swerve/" + this.m_name + "/Steer PID", m_steerPID);
    // SmartDashboard.putData("Swerve/" + this.m_name + "/Drive PID", m_drivePID);
  }
}
