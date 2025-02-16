package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.AprilTagAlignState;
import frc.robot.lib.RobotAlignStates;
import frc.robot.lib.g;


public class AutoDriveToPose extends Command {
  Pose2d m_desiredPose;
  double m_speed;
  double m_timeOut_sec;
  double m_driveDistance_m = 0;
  double m_driveAngle_deg = 0;
  double m_rampuUpTime_sec = 0.25;
  Rotation2d m_zeroRotation = new Rotation2d();
  PIDController m_drivePID = new PIDController(1, 0.5  , 0);
  Timer m_timer = new Timer();
  RobotAlignStates m_alignState = RobotAlignStates.UNKNOWN;
  AprilTagAlignState m_apriltagAlignState = AprilTagAlignState.NONE;
  /** Drive to a pose on the field. Pose must be relative to starting pose or the starting pose must be set based on field pose.
   * 
   * @param _desiredPose The Pose to drive to
   * @param _startPose The field releative pose position
   * @param _speed The max speed on a scale from 0.0 to 1.0. This is always positive
   * @param _timeOut_sec The time to end if pose not reached
   */
  public AutoDriveToPose(Pose2d _desiredPose,  double _speed, double _timeOut_sec) {
    addRequirements(g.ROBOT.drive);
    m_desiredPose = _desiredPose;
    m_speed = _speed;
    m_timeOut_sec = _timeOut_sec;
    m_drivePID.setTolerance(g.DRIVETRAIN.AUTO_DRIVE_POSE_DISTANCE_TOLERANCE_m);
    m_drivePID.setIZone(0.5);
    m_drivePID.setIntegratorRange(-m_speed/2, m_speed/2);
    m_alignState = RobotAlignStates.UNKNOWN;
    m_apriltagAlignState = AprilTagAlignState.NONE;


  }
public AutoDriveToPose(Pose2d _desiredPose,  double _speed, double _robotTargetAngle, double _timeOut_sec) {
  this(_desiredPose, _speed, _timeOut_sec);
  g.ROBOT.angleRobotTarget_deg = _robotTargetAngle;
  
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    // Set the global varables so the vision processor works on them
    //g.ROBOT.alignmentState = m_alignState;
    //g.VISION.aprilTagAlignState = m_apriltagAlignState;
   // SmartDashboard.putData("Drive/AutoDrivePID", m_drivePID);
  }

  // TODO: Test this class. Possible issues.
  //  [x] Starting Pose, CAN vision reset pose at beginning before match. OrangePI is on and working. Just reset Gyro based on yaw to tag
  //  [ ] Tolerance
  //  [ ] PIDs
  //  [ ] Speeds

  @Override
  public void execute() {
    // Calculate the angle and distance to the pose
    
    Pose2d trajectory = m_desiredPose.relativeTo(new Pose2d(g.ROBOT.pose2d.getX(), g.ROBOT.pose2d.getY(), m_zeroRotation));

    m_driveAngle_deg = trajectory.getTranslation().getAngle().getDegrees();
    if(DriverStation.getAlliance().isPresent()){
      if(DriverStation.getAlliance().get() == Alliance.Red){
        m_driveAngle_deg = m_driveAngle_deg + 180;
      }
    }
    m_driveDistance_m = g.ROBOT.pose2d.getTranslation().getDistance(m_desiredPose.getTranslation());

    SmartDashboard.putNumber("Drive/AutoDrive Distance m", m_driveDistance_m);
    SmartDashboard.putNumber("Drive/AutoDrive Angle", m_driveAngle_deg);
    SmartDashboard.putNumber("Drive/Robot Angle", g.ROBOT.angleRobotTarget_deg);
    // PID the speed based on distance
    double speed = Math.abs(m_drivePID.calculate(m_driveDistance_m,0));
    speed = rampUpValue(speed, m_rampuUpTime_sec);
    speed = MathUtil.clamp(speed, 0, m_speed);
    // Drive the robot in Polar mode since we have a speed and angle.
    g.ROBOT.drive.drivePolarFieldCentric(speed, g.ROBOT.angleActual_deg, g.ROBOT.angleRobotTarget_deg, m_driveAngle_deg, g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m);
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
    if(m_drivePID.atSetpoint()){
      g.VISION.aprilTagAlignState = AprilTagAlignState.NONE;
      return true;
    //}
    // if(Math.abs(m_driveDistance_m) < g.DRIVETRAIN.AUTO_DRIVE_POSE_DISTANCE_TOLERANCE_m
    //     // && Math.abs((g.ROBOT.pose2dDrive.getX()) - m_desiredPose.getX()) < g.DRIVETRAIN.AUTO_DRIVE_POSE_DISTANCE_TOLERANCE_m
    //     // && Math.abs((g.ROBOT.pose2dDrive.getY()) - m_desiredPose.getY()) < g.DRIVETRAIN.AUTO_DRIVE_POSE_DISTANCE_TOLERANCE_m
    //    && g.ROBOT.drive.isRotateAtTarget())
    // {
     // g.DRIVETRAIN.isAutoToAprilTagDone = true;
      
    }
    if(m_timer.hasElapsed(m_timeOut_sec)){
      return true;
    }
    return false;
  }
}
