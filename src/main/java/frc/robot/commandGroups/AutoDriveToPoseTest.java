package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoDriveToPose;
import frc.robot.lib.AprilTagAlignState;
import frc.robot.lib.g;

public class AutoDriveToPoseTest extends SequentialCommandGroup {
  /** Creates a new AutoDriveRotateTest. */
  public AutoDriveToPoseTest() {
    // Pose X,Y are in Meters, X positive is forward, Y Positive is to the left
    addCommands(
      new SequentialCommandGroup(
        new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(20, AprilTagAlignState.RIGHT), 0.7, 6)
        ),
      new ParallelRaceGroup(
        new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(20, AprilTagAlignState.RIGHT), 0.7, 6)
      ),
      new ParallelDeadlineGroup( 
        new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(20, AprilTagAlignState.RIGHT), 0.7, 6)
      ),
      new ParallelCommandGroup(
        // Move Coral to position L2
        // Spin the wheels to put the coral on the reef
       new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(20, AprilTagAlignState.RIGHT), 0.7, 6)
      ).withTimeout(3),
      new ParallelCommandGroup(
        // Move Coral to start position
        // Spin the wheels to take in the coral
       new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(13, AprilTagAlignState.CENTER), 0.7, 6)
      ),
      new ParallelCommandGroup(
        // Move Coral to position L4
        // Spin the wheels to put the coral on the reef
       new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(19, AprilTagAlignState.LEFT), 0.7, 6)
      ),
      new ParallelCommandGroup(
        // Move Coral to Start position
        // Spin the wheels to take in the coral
       new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(13, AprilTagAlignState.CENTER), 0.7, 6)
      ),
      new ParallelCommandGroup(
        // Move Coral to position L4
        // Spin the wheels to put the coral on the reef
       new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(19, AprilTagAlignState.RIGHT), 0.7, 6)
      )
      
    );
  }
}
