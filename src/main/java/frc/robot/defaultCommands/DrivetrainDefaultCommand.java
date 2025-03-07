package frc.robot.defaultCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.AutoDriveToPose;
import frc.robot.lib.DriveMode;
import frc.robot.lib.g;

public class DrivetrainDefaultCommand extends Command {
  SlewRateLimiter m_stickLimiterLX = new SlewRateLimiter(3);
  SlewRateLimiter m_stickLimiterLY = new SlewRateLimiter(3);
  SlewRateLimiter m_stickLimiterRX = new SlewRateLimiter(3);
  SlewRateLimiter m_stickLimiterRY = new SlewRateLimiter(3);

  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand() {
    addRequirements(g.ROBOT.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //kLeftX(0),kLeftY(1),
    //kRightX(2),kRightY(5),

    double leftYRaw_Driver = -g.OI.driverController.getLeftX(); // 0
    double leftXRaw_Driver = -g.OI.driverController.getLeftY(); // 1
    double rightYRaw_Driver = -g.OI.driverController.getRightX(); // 2
    double rightXRaw_Driver = -g.OI.driverController.getRightY(); // 5
    
    double rightYRaw_Operator = -g.OI.operatorController.getRightX(); // 2
    double rightXRaw_Operator = -g.OI.operatorController.getRightY(); // 5

    // Limit the inputs for a deadband related to the joystick
    double leftYFiltered_Driver = MathUtil.applyDeadband(leftYRaw_Driver, 0.1, 1.0);
    double leftXFiltered_Driver = MathUtil.applyDeadband(leftXRaw_Driver, 0.1, 1.0);
    double rightXFiltered_Driver = MathUtil.applyDeadband(rightXRaw_Driver, 0.1, 1.0);
    double rightYFiltered_Driver = MathUtil.applyDeadband(rightYRaw_Driver, 0.1, 1.0);

    // Limit the speed of change to reduce the acceleration
    leftXFiltered_Driver = m_stickLimiterLX.calculate(leftXFiltered_Driver);
    leftYFiltered_Driver = m_stickLimiterLY.calculate(leftYFiltered_Driver);
    rightXFiltered_Driver = m_stickLimiterRX.calculate(rightXFiltered_Driver);
    rightYFiltered_Driver = m_stickLimiterRY.calculate(rightYFiltered_Driver);

    double redInvert = 1;
    if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
      redInvert = -1;
    }

    if(g.ROBOT.vision.getIsAutoAprilTagActive()){
      g.VISION.aprilTagRequestedPose = g.ROBOT.vision.getRobotPoseForAprilTag(g.VISION.aprilTagRequestedID, g.VISION.aprilTagAlignState);
      AutoDriveToPose autoPose = new AutoDriveToPose(g.VISION.aprilTagRequestedPose, 1, 5);
      autoPose.schedule();
    }else {
      switch (g.DRIVETRAIN.driveMode) {
        case FIELD_CENTRIC:
          g.ROBOT.drive.driveFieldCentric(leftXFiltered_Driver*redInvert, leftYFiltered_Driver*redInvert, rightYFiltered_Driver, g.ROBOT.angleActual_deg, g.DRIVETRAIN.centerOfRotation_m);

          break;
        case ANGLE_FIELD_CENTRIC:
          g.ROBOT.drive.setTargetRobotAngle(rightXFiltered_Driver*redInvert, rightYFiltered_Driver*redInvert);
          g.ROBOT.drive.setTargetRobotAngle(rightXRaw_Operator*redInvert,rightYRaw_Operator*redInvert);
          g.ROBOT.drive.driveAngleFieldCentric( leftXFiltered_Driver*redInvert, leftYFiltered_Driver*redInvert, g.ROBOT.angleActual_deg, g.ROBOT.angleRobotTarget_deg, g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m);
          break;
        case POLAR_CENTRIC:
          // This mode is not used by the operator. It is intented for autonomous or teleOp commands. 
          //g.ROBOT.drive.drivePolarFieldCentric(g.ROBOT.speedDriveTarget_mPsec, g.ROBOT.angleActual_deg, g.ROBOT.angleRobotTarget_deg, g.ROBOT.angleDriveTarget_deg, g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m);
          break;
        case ROBOT_CENTRIC:
          g.ROBOT.drive.driveRobotCentric(leftXFiltered_Driver*redInvert, leftYFiltered_Driver*redInvert, rightYFiltered_Driver, g.DRIVETRAIN.ZERO_CENTER_OF_ROTATION_m);
          break;
        case FAST_STOP:
          // g.ROBOT.drive.fastStop();
          break;
        default:
          break;
      }
    }
    
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
