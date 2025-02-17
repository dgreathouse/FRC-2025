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

       new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(20, AprilTagAlignState.RIGHT), 0.7, 6),
       new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(13, AprilTagAlignState.CENTER), 0.7, 6),
       new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(19, AprilTagAlignState.LEFT), 0.7, 6),
       new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(13, AprilTagAlignState.CENTER), 0.7, 6),
       new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(19, AprilTagAlignState.RIGHT), 0.7, 6)
      
    );
  }
}
