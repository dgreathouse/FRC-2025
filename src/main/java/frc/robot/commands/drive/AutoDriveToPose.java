package frc.robot.commands.drive;

import com.ctre.phoenix6.configs.GyroTrimConfigs;

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
import frc.robot.lib.g;


public class AutoDriveToPose extends Command {
  Pose2d m_desiredPose;
  double m_speed;
  double m_timeOut_sec;
  double m_driveDistance_m = 0;
  double m_driveAngle_deg = 0;
  double m_robotTargetAngle_deg = 0;
  double m_rampuUpTime_sec = 0.25;
  PIDController m_drivePID = new PIDController(0.45, 0.015, 0);
  PIDController m_turnPID = new PIDController(0.4, 0.001, 0.0);
  Timer m_timer = new Timer();

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
    m_drivePID.setTolerance(.0154);
    m_drivePID.setIZone(0.15);
    //m_drivePID.setIntegratorRange(-0.35, 0.35);

    m_robotTargetAngle_deg = _desiredPose.getRotation().getDegrees();
    m_turnPID.enableContinuousInput(-Math.PI, Math.PI);
    m_turnPID.setTolerance(Math.toRadians(.2));
    m_turnPID.setIntegratorRange(-0.05, 0.05);
    m_turnPID.setIZone(0.15);

  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    g.ROBOT.angleRobotTarget_deg = m_robotTargetAngle_deg;
    g.DRIVETRAIN.driveMode = DriveMode.FIELD_CENTRIC;
    SmartDashboard.putData("Auto/TurnPID", m_turnPID);
    SmartDashboard.putData("Auto/DrivePID", m_drivePID);
  }


  @Override
  public void execute() {
    m_driveAngle_deg = m_desiredPose.getTranslation().minus(g.ROBOT.pose2d.getTranslation()).getAngle().getDegrees();
    m_driveDistance_m = g.ROBOT.pose2d.getTranslation().getDistance(m_desiredPose.getTranslation());
    

    double speed = Math.abs(m_drivePID.calculate(m_driveDistance_m,0)); // Calculate the speed based on the distance to the target
    speed = MathUtil.clamp(speed, 0, m_speed); // Clamp the speed to the max speed
    speed = rampUpValue(speed, m_rampuUpTime_sec);  // Ramp up the speed
    

    double y = Math.sin(Math.toRadians(m_driveAngle_deg)); // Calculate the x and y components of the speed
    double x = Math.cos(Math.toRadians(m_driveAngle_deg));
    m_speeds.vxMetersPerSecond = x * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec * speed; // Set the speed of the robot
    m_speeds.vyMetersPerSecond = y * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec * speed; // Set the speed of the robot

    double rotate = m_turnPID.calculate(Math.toRadians(g.ROBOT.angleActual_deg), Math.toRadians(m_robotTargetAngle_deg)); // Calculate the rotation speed
    m_speeds.omegaRadiansPerSecond = rotate * g.SWERVE.DRIVE.MAX_ANGULAR_VELOCITY_radPsec; // Set the rotation speed
    m_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_speeds, new Rotation2d(Math.toRadians(-g.ROBOT.angleActual_deg))); // Convert the speed to robot relative speeds

    g.ROBOT.drive.setSwerveModuleStates(m_speeds, g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m); // Set the speeds of the swerve modules
    
    SmartDashboard.putNumber("Auto/m_driveAngle_deg", m_driveAngle_deg);
    SmartDashboard.putNumber("Auto/m_driveDistance_m", m_driveDistance_m);
    SmartDashboard.putNumber("Auto/rotate", rotate);
    SmartDashboard.putNumber("Auto/Pose2d_X", g.ROBOT.pose2d.getX());
    SmartDashboard.putNumber("Auto/Pose2d_Y", g.ROBOT.pose2d.getY());
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
    if(m_drivePID.atSetpoint() && m_turnPID.atSetpoint() || m_timer.hasElapsed(m_timeOut_sec)){
      g.VISION.aprilTagAlignState = AprilTagAlignState.NONE;
      return true;
    }
    return false;
  }
}
