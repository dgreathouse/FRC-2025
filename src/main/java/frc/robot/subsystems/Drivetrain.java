package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.g;

public class Drivetrain extends SubsystemBase implements IUpdateDashboard {
  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_odometry;
  private OdometryThread m_odometryThread;
  private StatusSignal<Angle> m_yawStatusPrimary;
  private StatusSignal<AngularVelocity> m_angularVelocityZStatusPrimary;
  private double m_yawPrimary;
  private double m_angularVelocityZPrimary;
  private double m_yawSecondary = 0;
  private double m_angularVelocityZSecondary = 0;
  // TODO: Tune KP,KI,KD max output should be +/-1 Start around 1/3.14 for Kp
  private PIDController m_turnPID = new PIDController(g.DRIVETRAIN.TURN_KP, g.DRIVETRAIN.TURN_KI, g.DRIVETRAIN.TURN_KD);

  private ChassisSpeeds m_speeds = new ChassisSpeeds();

  /** Creates a new Drivetrain. */
  @SuppressWarnings("unused")
  public Drivetrain() {
    m_yawStatusPrimary = g.ROBOT.gyro_pigeon2.getYaw();
    m_yawStatusPrimary.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_hz);
    m_angularVelocityZStatusPrimary = g.ROBOT.gyro_pigeon2.getAngularVelocityZDevice();
    m_angularVelocityZStatusPrimary.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_hz);

    g.SWERVE.modules[0] = new SwerveModule(
        "BR",
        12,
        true,
        22,
        true,
        2,
        0.425,
        g.CHASSIS.BACK_RIGHT_SWERVE_X_POSITION_m,
        g.CHASSIS.BACK_RIGHT_SWERVE_Y_POSITION_m);
    g.SWERVE.modules[1] = new SwerveModule(
        "BL",
        13,
        false,
        23,
        true,
        3,
        -0.160,
        g.CHASSIS.BACK_LEFT_SWERVE_X_POSITION_m,
        g.CHASSIS.BACK_LEFT_SWERVE_Y_POSITION_m);
    g.SWERVE.modules[2] = new SwerveModule(
        "F",
        11,
        true,
        21,
        true,
        1,
        0.0344,
        g.CHASSIS.FRONT_SWERVE_X_POSITION_m,
        g.CHASSIS.FRONT_SWERVE_Y_POSITION_m);
    if (g.SWERVE.COUNT == 4) {
      g.SWERVE.modules[3] = new SwerveModule("BL", 13, false, 23, false, 4, 0, 0, 0);
    }

    for (int i = 0; i < g.SWERVE.COUNT; i++) {
      g.SWERVE.positions[i] = new SwerveModulePosition();
      g.DASHBOARD.updates.add(g.SWERVE.modules[i]);
    }
    updatePositions();

    m_kinematics = new SwerveDriveKinematics(
        g.SWERVE.modules[0].m_location,
        g.SWERVE.modules[1].m_location,
        g.SWERVE.modules[2].m_location);
    m_odometry = new SwerveDriveOdometry(m_kinematics, g.ROBOT.angleActual_Rot2d, g.SWERVE.positions);

    m_turnPID.enableContinuousInput(-Math.PI, Math.PI);
    // TODO set Derivative tolerance so atSetPoint only returns true at low speeds
    m_turnPID.setTolerance(Math.toRadians(1.0), Double.POSITIVE_INFINITY);
    m_turnPID.setIZone(Math.toRadians(45));
    m_turnPID.setIntegratorRange(-0.5, 0.5);

    m_odometryThread = new OdometryThread();
    m_odometryThread.start();
    SmartDashboard.putBoolean("Robot/IsGyroPrimaryActive", true);
    resetYaw(0);
    g.DASHBOARD.updates.add(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Drive in old fashion mode. Forward on thumb stick makes the robot go forward with reference to the front of the robot.
   * 
   * @param _xSpeed The X speed in +/- 1.0 Forward +
   * @param _ySpeed The Y speed in +/- 1.0 Left +
   * @param _rotate The rotational speed in +/- 1.0
   * @param _centerOfRotation_m The center of rotation for the robot. To be the ceter of the robot use g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m.
   */
  public void driveRobotCentric(double _xSpeed, double _ySpeed, double _rotate, Translation2d _centerOfRotation_m) {
    m_speeds.vxMetersPerSecond = _xSpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec;
    m_speeds.vyMetersPerSecond = _ySpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec;
    m_speeds.omegaRadiansPerSecond = _rotate * g.SWERVE.DRIVE.MAX_ANGULAR_VELOCITY_radPsec;
    
    setSwerveModuleStates(m_speeds, _centerOfRotation_m);
  }

  /** Drive in field centric mode. Forward on thumb stick is always forward on field no matter what way the robot is facing
   * 
   * @param _xSpeed The X speed in +/- 1.0 Forward +
   * @param _ySpeed The Y speed in +/- 1.0 Left +
   * @param _rotate The rotational speed in +/- 1.0
   * @param _robotAngle_deg The current robot angle 
   * @param _centerOfRotation_m The center of rotation for the robot. To be the ceter of the robot use g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m.
   */
  public void driveFieldCentric(double _xSpeed, double _ySpeed, double _rotate, double _robotAngle_deg, Translation2d _centerOfRotation_m) {
    
    m_speeds.vxMetersPerSecond = _xSpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec;
    m_speeds.vyMetersPerSecond = _ySpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec;
    m_speeds.omegaRadiansPerSecond = _rotate * g.SWERVE.DRIVE.MAX_ANGULAR_VELOCITY_radPsec;

    m_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_speeds, new Rotation2d(Math.toRadians(_robotAngle_deg)));
    SmartDashboard.putNumber("COR", _centerOfRotation_m.getX());
    setSwerveModuleStates(m_speeds, _centerOfRotation_m);
  }

  /** Use the X and Y values that usually come from a joystick and drive with a robot target angle.
   * This maintains the robot facing direction to the target angle.
   * 
   * @param _xSpeed The X speed in +/- 1.0
   * @param _ySpeed The Y speed in +/- 1.0
   * @param _robotAngle_deg The current robot angle 
   * @param _targetAngle_deg The desired angle for the front of the robot to face.
   * @param _centerOfRotation_m The center of rotation for the robot. To be the ceter of the robot use g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m.
   */
  public void driveAngleFieldCentric(double _xSpeed, double _ySpeed, double _robotAngle_deg, double _targetAngle_deg, Translation2d _centerOfRotation_m) {
    m_speeds.vxMetersPerSecond = _xSpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec;
    m_speeds.vyMetersPerSecond = _ySpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec;
    double rotate = m_turnPID.calculate(Math.toRadians(_robotAngle_deg), Math.toRadians(_targetAngle_deg));
    rotate = MathUtil.applyDeadband(rotate, g.DRIVETRAIN.TURN_DEADBAND);
    m_speeds.omegaRadiansPerSecond = rotate * g.SWERVE.DRIVE.MAX_ANGULAR_VELOCITY_radPsec;

    m_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_speeds, new Rotation2d(Math.toRadians(_robotAngle_deg)));

    setSwerveModuleStates(m_speeds, _centerOfRotation_m);
  }

  /** Get the X and Y values from the Drive Angle and call {@link #driveAngleFieldCentric(double, double, double, double)}
   * 
   * 
   * @param _speed The speed to drive at as +/- 1.0
   * @param _robotAngle_deg The angle of the robot which usually is g.ROBOT.angleActual_deg
   * @param _targetAngle_deg The angle you want the robot front to point to.
   * @param _driveAngle_deg The drive angle you want the robot to drive at
   * @param _centerOfRotation_m The center of rotation for the robot. To be the ceter of the robot use g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m.
  
   */
  public void drivePolarFieldCentric(double _speed, double _robotAngle_deg, double _targetAngle_deg, double _driveAngle_deg, Translation2d _centerOfRotation_m) {
    double y = Math.sin(Units.degreesToRadians(_driveAngle_deg)) * _speed;
    double x = Math.cos(Units.degreesToRadians(_driveAngle_deg)) * _speed;
    driveAngleFieldCentric(x, y, _robotAngle_deg, _targetAngle_deg, _centerOfRotation_m);
  }
  /** This is a compensated AngleFieldCentric that takes the speed from the x,y and generates a new X,Y from driveAngle
   * 
   * @param _xSpeed The X speed in +/- 1.0
   * @param _ySpeed The Y speed in +/- 1.0
   * @param _robotAngle_deg The current robot angle 
   * @param _targetAngle_deg The desired angle for the front of the robot to face.
   * @param _driveAngle_deg The angle to drive the robot at on the field
   * @param _centerOfRotation_m The center of rotation for the robot. To be the ceter of the robot use g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m.
   */
  public void driveAngleFieldCentric(double _xSpeed, double _ySpeed, double _robotAngle_deg, double _targetAngle_deg, double _driveAngle_deg, Translation2d _centerOfRotation_m){

    double speed = Math.max(Math.abs(_xSpeed), Math.abs(_ySpeed));
    double y = Math.sin(Units.degreesToRadians(_driveAngle_deg)) * speed;
    double x = Math.cos(Units.degreesToRadians(_driveAngle_deg)) * speed;
    driveAngleFieldCentric(x, y, _robotAngle_deg, _targetAngle_deg, _centerOfRotation_m);
  }

  public void setSwerveModuleStates(ChassisSpeeds _speeds, Translation2d _centerOfRotation_m){
    _centerOfRotation_m = _centerOfRotation_m == null ? g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m : _centerOfRotation_m;

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_speeds, _centerOfRotation_m);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MetersPerSecond.of(g.SWERVE.DRIVE.MAX_VELOCITY_mPsec));

    for (int i = 0; i < g.SWERVE.COUNT; i++) {
      g.SWERVE.modules[i].setDesiredState(states[i]);
    }
  }


  // /**
  //  * This is the final method of all drive methods that sends the array of 
  //  * Swerve Module angles and speeds to the SwerveModule classes
  //  * @param _states
  //  */
  // public void setSwerveModules(SwerveModuleState[] _states) {
  //   for (int i = 0; i < g.SWERVE.COUNT; i++) {
  //     g.SWERVE.modules[i].setDesiredState(_states[i]);
  //   }
  // }
  /** Set the drive angle for AngleFieldCentric mode if the  
   * Hypotenus of the x,y is greater that a threashold
   * 
   * @param _x The X value
   * @param _y The Y value
   */
  public void setTargetRobotAngle(double _x, double _y) {
    double x = _x; // -g.OI.driverController.getRightX();
    double y = _y; // -g.OI.driverController.getRightY();
    double hyp = Math.hypot(x, y);
    double joystickAngle = Math.toDegrees(Math.atan2(x, y));

    if (Math.abs(hyp) > g.OI.ANGLE_TARGET_DEADBAND) {
      if (joystickAngle >= -22.5 && joystickAngle <= 22.5) { // North
        g.ROBOT.angleRobotTarget_deg = 0.0;
      } else if (joystickAngle >= -67.5 && joystickAngle < -22.5) { // North East
        g.ROBOT.angleRobotTarget_deg = -45.0;
      } else if (joystickAngle >= -112.5 && joystickAngle < -67.5) { // East
        g.ROBOT.angleRobotTarget_deg = -90.0;
      } else if (joystickAngle >= -157.5 && joystickAngle < -112.5) { // South East
        g.ROBOT.angleRobotTarget_deg = -135.0;
      } else if ((joystickAngle >= 157.5 && joystickAngle <= 180.0)
          || (joystickAngle <= -157.5 && joystickAngle > -179.99)) { // South
        g.ROBOT.angleRobotTarget_deg = 180.0;
      } else if (joystickAngle <= 67.5 && joystickAngle > 22.5) { // North West
        g.ROBOT.angleRobotTarget_deg = 45.0;
      } else if (joystickAngle <= 112.5 && joystickAngle > 67.5) { // West
        g.ROBOT.angleRobotTarget_deg = 90.0;
      } else if (joystickAngle <= 157.5 && joystickAngle > 112.5) { // South West
        g.ROBOT.angleRobotTarget_deg = 135.0;
      }
    }
  }

  /**
   * Directly set the g.ROBOT.angleTarget_deg to a angle. Generally this is used
   * in autonomous or by a button.
   *
   * @param _angle_deg The target angle the robot should PID to in
   *                   AngleFieldCentric Mode.
   */
  public void setAngleTarget(double _angle_deg) {
    g.ROBOT.angleRobotTarget_deg = _angle_deg;
  }

    /**
   * Is the robot rotational on target for g.ROBOT.AngleTarget_deg
   *
   * @return if the rotation of the robot is on target
   */
  public boolean isRotateAtTarget() {
    return m_turnPID.atSetpoint();
  }

  public void resetYaw(double _angle) {
    g.ROBOT.gyro_pigeon2.setYaw(_angle);
    g.ROBOT.gyro_navx.reset();
  }

  public void updateDashboard() {
    g.SWERVE.totalSwerveCurrent_amps = 0;
    for (SwerveModule swerveModule : g.SWERVE.modules) {
      g.SWERVE.totalSwerveCurrent_amps += Math.abs(swerveModule.getDriveCurrent())
          + Math.abs(swerveModule.getSteerCurrent());
    }
    SmartDashboard.putNumber("Swerve/totalSwerveCurrent_amps", g.SWERVE.totalSwerveCurrent_amps);
    SmartDashboard.putData("Robot/Field2d", g.ROBOT.field2d);
    SmartDashboard.putNumber("Robot/angleTarget_deg", g.ROBOT.angleRobotTarget_deg);
    SmartDashboard.putNumber("Robot/angleActual_deg", g.ROBOT.angleActual_deg);

    SmartDashboard.putNumber("Robot/GyroPrimary_deg", m_yawPrimary);
    SmartDashboard.putNumber("Robot/GyroSecondary_deg", m_yawSecondary);
    SmartDashboard.putNumber("Robot/GyroYaw_deg", getYaw());

    // Get from Dashboard
    g.ROBOT.isPrimaryGyroActive = SmartDashboard.getBoolean("Robot/IsGyroPrimaryActive", true);
    
  }
  private void updatePositions() {
    for (int i = 0; i < g.SWERVE.COUNT; i++) {
      g.SWERVE.positions[i] = g.SWERVE.modules[i].updatePosition();
    }
  }
  public double getYaw(){
    return g.ROBOT.isPrimaryGyroActive ? m_yawPrimary : m_yawSecondary;
  }
  public double getAngularVelocityZ(){
    return g.ROBOT.isPrimaryGyroActive ? m_angularVelocityZPrimary : m_angularVelocityZSecondary;
  }
  private class OdometryThread extends Thread {
    public OdometryThread() {
      super();
    }

    @Override
    public void run() {
      
      while (true) {
        /* Now update odometry */
        updatePositions();
        m_yawStatusPrimary = g.ROBOT.gyro_pigeon2.getYaw();
        m_angularVelocityZStatusPrimary = g.ROBOT.gyro_pigeon2.getAngularVelocityZDevice();
        m_yawPrimary = m_yawStatusPrimary.getValueAsDouble();
        m_angularVelocityZPrimary = m_angularVelocityZStatusPrimary.getValueAsDouble();
        
        m_yawSecondary = -g.ROBOT.gyro_navx.getAngle();
        m_angularVelocityZSecondary = -g.ROBOT.gyro_navx.getVelocityZ();

        g.ROBOT.angleActual_deg = getYaw();
        g.ROBOT.angleActual_Rot2d = Rotation2d.fromDegrees(g.ROBOT.angleActual_deg);

        g.ROBOT.pose2d = m_odometry.update(g.ROBOT.angleActual_Rot2d, g.SWERVE.positions);
        g.ROBOT.pose3d = new Pose3d(g.ROBOT.pose2d);
        g.ROBOT.field2d.setRobotPose(g.ROBOT.pose2d);
        g.ROBOT.vision.calculate();
        try {
          Thread.sleep(g.ROBOT.ODOMETRY_RATE_ms);
        } catch (InterruptedException e) {
          System.out.println(e.getMessage());
        }
      }
    }
  }
}
