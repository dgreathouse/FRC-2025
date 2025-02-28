package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoDriveToPose;
import frc.robot.lib.AprilTagAlignState;
import frc.robot.lib.g;

public class AutoDriveToPoseTest extends SequentialCommandGroup {
  /** Creates a new AutoDriveRotateTest. */
  public AutoDriveToPoseTest() {
    // Pose X,Y are in Meters, X positive is forward, Y Positive is to the left
    
addCommands(

       new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(22, AprilTagAlignState.RIGHT), 0.6, 6),
       new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(12, AprilTagAlignState.RIGHT), 0.6, 6),
       new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(17, AprilTagAlignState.RIGHT), 0.6, 6),
       new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(12, AprilTagAlignState.RIGHT), 0.6, 6),
       new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(17, AprilTagAlignState.LEFT), 0.6, 6)
      
    );
  }
}
