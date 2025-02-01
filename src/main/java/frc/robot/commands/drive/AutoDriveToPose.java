package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  double m_rampuUpTime_sec = 0.25;
  Rotation2d m_zeroRotation = new Rotation2d();
  PIDController m_drivePID = new PIDController(1, 0  , 0);
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
    m_drivePID.setTolerance(0.05);
    m_drivePID.setIZone(0.5);
    m_drivePID.setIntegratorRange(-m_speed/2, m_speed/2);
    m_alignState = RobotAlignStates.UNKNOWN;
    m_apriltagAlignState = AprilTagAlignState.NONE;

  }
  // /** Drive to a pose on the field. Pose must be relative to starting pose or the starting pose must be set based on field pose.
  //  * 
  //  * @param _newPose The Pose to drive to
  //  * @param _startPose The field releative pose position
  //  * @param _speed The max speed on a scale from 0.0 to 1.0. This is always positive
  //  * @param _timeOut_sec The time to end if pose not reached
  //  * @param _alingState The alignment state of the robot
  //   */
  // public AutoDriveToPose(Pose2d _newPose, double _speed, double _timeOut_sec, RobotAlignStates _alingState, AprilTagAlignState _apriltagLocation) {
  //   this(_newPose, _speed, _timeOut_sec);
  //   m_alignState = _alingState;
  //   m_apriltagAlignState = _apriltagLocation;

  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    // Set the global varables so the vision processor works on them
    g.ROBOT.alignmentState = m_alignState;
    g.VISION.aprilTagAlignState = m_apriltagAlignState;
    
  }

  // TODO: Test this class. Possible issues.
  //  [x] Starting Pose
  //  [ ] Tolerance
  //  [ ] PIDs

  @Override
  public void execute() {
    // Calculate the angle and distance to the pose
    Pose2d trajectory = m_desiredPose.relativeTo(new Pose2d(g.ROBOT.pose2d.getX(), g.ROBOT.pose2d.getY(), m_zeroRotation));
    double driveAngle_deg = getAngleBasedOnAlliance(trajectory);
    
    m_driveDistance_m = g.ROBOT.pose2d.getTranslation().getDistance(m_desiredPose.getTranslation());
    // PID the speed based on distance
    double speed = m_drivePID.calculate(0,m_driveDistance_m);
    speed = rampUpValue(speed, m_rampuUpTime_sec);
    speed = MathUtil.clamp(speed, 0, m_speed);
    // Drive the robot in Polar mode since we have a speed and angle.
    g.ROBOT.drive.drivePolarFieldCentric(speed, g.ROBOT.angleActual_deg, g.ROBOT.angleRobotTarget_deg,driveAngle_deg, g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m);
  }

  private double  getAngleBasedOnAlliance(Pose2d _pose){
    double angle = _pose.getTranslation().getAngle().getDegrees();
    if(g.MATCH.alliance == Alliance.Blue){
      return angle;
    }else {
      return angle + 180;
    }
    
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
    if(Math.abs(m_driveDistance_m) < g.DRIVETRAIN.AUTO_DRIVE_POSE_DISTANCE_TOLERANCE_m
      //  && Math.abs((g.ROBOT.pose2d.getX()) - m_desiredPose.getX()) < g.DRIVETRAIN.AUTO_DRIVE_POSE_DISTANCE_TOLERANCE_m
      //  && Math.abs((g.ROBOT.pose2d.getY()) - m_desiredPose.getY()) < g.DRIVETRAIN.AUTO_DRIVE_POSE_DISTANCE_TOLERANCE_m
       && g.ROBOT.drive.isRotateAtTarget())
    {
     // g.DRIVETRAIN.isAutoToAprilTagDone = true;
      g.VISION.aprilTagAlignState = AprilTagAlignState.NONE;
      return true;
    }
    if(m_timer.hasElapsed(m_timeOut_sec)){
      return true;
    }
    return false;
  }
}
