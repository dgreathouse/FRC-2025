package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoDriveToPose;

public class AutoDriveToPoseTest extends SequentialCommandGroup {
  /** Creates a new AutoDriveRotateTest. */
  public AutoDriveToPoseTest() {
    // Pose X,Y are in Meters, X positive is forward, Y Positive is to the left
    addCommands(
      new AutoDriveToPose(new Pose2d(5, 0, new Rotation2d(Math.toRadians(0))), 0.2, 10)
      //new AutoDriveToPose(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), 0, 0, RobotAlignStates.BACK, AprilTagAlignState.RIGHT)
    );
  }
}
