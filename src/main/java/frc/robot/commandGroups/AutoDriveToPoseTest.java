package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoDriveToPose;
import frc.robot.lib.StartLocation;
import frc.robot.lib.g;

public class AutoDriveToPoseTest extends SequentialCommandGroup {
  /** Creates a new AutoDriveRotateTest. */
  public AutoDriveToPoseTest() {
    // Pose X,Y are in Meters, X positive is forward, Y Positive is to the left
    addCommands(
      new InstantCommand( ()-> {g.ROBOT.drive.setOdometry(StartLocation.LEFT);}, g.ROBOT.drive),
      new AutoDriveToPose(new Pose2d(2.51,g.ROBOT.POSE_START_LEFT.getY(),new Rotation2d(0)), 0.25, 8),  // Drives straight to the back left of the field
      new AutoDriveToPose(new Pose2d(2.51,4.492,new Rotation2d(0)), 0.25, 8),   // Drives behind the reef
      new AutoDriveToPose(new Pose2d(3,4.492,new Rotation2d(0)), 0.25, 8)    // Drives up to the reef on the back side
      
    );
  }
}
