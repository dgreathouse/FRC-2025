package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.AprilTagAlignState;
import frc.robot.lib.DriveMode;
import frc.robot.lib.RobotAlignStates;
import frc.robot.lib.g;


public class AutoDriveToPose extends Command {
  Pose2d m_desiredPose;
  double m_speed;
  double m_timeOut_sec;
  double m_driveDistance_m = 0;
  double m_driveAngle_deg = 0;
  double m_robotTargetAngle_deg = 0;
  double m_rampuUpTime_sec = 0.15;
  Rotation2d m_zeroRotation = new Rotation2d();
  PIDController m_drivePID = new PIDController(.25, 0.0  , 0);
  Timer m_timer = new Timer();
  AprilTagAlignState m_apriltagAlignState = AprilTagAlignState.NONE;
  private PIDController m_turnPID = new PIDController(.075, 0.0, 0.0);

  private ChassisSpeeds m_speeds = new ChassisSpeeds();
  /** Drive to a pose on the field. Pose must be relative to starting pose or the starting pose must be set based on field pose.
   * 
   * @param _desiredPose The Pose to drive to with X,Y are reference to blue and angle is Red/Blue aligned
   * @param _speed The max speed on a scale from 0.0 to 1.0. This is always positive
   * @param _timeOut_sec The time to end if pose not reached
   */
  public AutoDriveToPose(Pose2d _desiredPose,  double _speed, double _timeOut_sec) {
    addRequirements(g.ROBOT.drive);
    m_desiredPose = _desiredPose;
    m_speed = _speed;
    m_timeOut_sec = _timeOut_sec;
   // m_drivePID.setTolerance(g.DRIVETRAIN.AUTO_DRIVE_POSE_DISTANCE_TOLERANCE_m);
    m_drivePID.setTolerance(.005);
    //m_drivePID.setIZone(0.5);
    //m_drivePID.setIntegratorRange(-0.35, 0.35);
    //m_alignState = RobotAlignStates.UNKNOWN;
    m_apriltagAlignState = AprilTagAlignState.NONE;
    m_robotTargetAngle_deg = _desiredPose.getRotation().getDegrees();
    m_turnPID.setTolerance(Math.toRadians(1));
    m_turnPID.setIntegratorRange(-0.15, 0.15);
    m_turnPID.setIZone(0.15);

  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    g.ROBOT.angleRobotTarget_deg = m_robotTargetAngle_deg;
    g.DRIVETRAIN.driveMode = DriveMode.FIELD_CENTRIC;
  }

  // TODO: Test this class. Possible issues.
  //  [x] Starting Pose, CAN vision reset pose at beginning before match. OrangePI is on and working. Just reset Gyro based on yaw to tag
  //  [ ] Tolerance
  //  [ ] PIDs
  //  [ ] Speeds

  @Override
  public void execute() {
    m_driveAngle_deg = m_desiredPose.getTranslation().minus(g.ROBOT.pose2d.getTranslation()).getAngle().getDegrees();
    m_driveDistance_m = g.ROBOT.pose2d.getTranslation().getDistance(m_desiredPose.getTranslation());
    SmartDashboard.putNumber("Auto/m_driveDistance_m", m_driveDistance_m);
    double speed = Math.abs(m_drivePID.calculate(m_driveDistance_m,0));
    speed = rampUpValue(speed, m_rampuUpTime_sec);
    speed = MathUtil.clamp(speed, -m_speed, m_speed);
    double y = Math.sin(Math.toRadians(m_driveAngle_deg)) * speed;
    double x = Math.cos(Math.toRadians(m_driveAngle_deg)) * speed;
    m_speeds.vxMetersPerSecond = x * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec;
    m_speeds.vyMetersPerSecond = y * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec;
    double rotate = m_turnPID.calculate(Math.toRadians(g.ROBOT.angleActual_deg), Math.toRadians(m_robotTargetAngle_deg));
    m_speeds.omegaRadiansPerSecond = rotate * g.SWERVE.DRIVE.MAX_ANGULAR_VELOCITY_radPsec;
    SmartDashboard.putNumber("Rotate", rotate);
    m_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_speeds, new Rotation2d(Math.toRadians(-g.ROBOT.angleActual_deg)));
    g.ROBOT.drive.setSwerveModuleStates(m_speeds, g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m);

    //g.ROBOT.drive.driveAngleFieldCentric(x,y, g.ROBOT.angleActual_deg, m_robotTargetAngle_deg,  g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m);
    // Drive the robot in Polar mode since we have a speed and angle.
    //g.ROBOT.drive.drivePolarFieldCentric(speed, g.ROBOT.angleActual_deg, m_robotTargetAngle_deg, m_driveAngle_deg, g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m);
  }

  private double rampUpValue(double _val, double _rampTime_sec) {
    double currentTime_sec = m_timer.get();
    if (currentTime_sec < _rampTime_sec) {
      _val = _val * currentTime_sec / _rampTime_sec;
    }
    return _val;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_drivePID.atSetpoint() || m_timer.hasElapsed(m_timeOut_sec)){
      g.VISION.aprilTagAlignState = AprilTagAlignState.NONE;
      return true;
    }
    return false;
  }
}
