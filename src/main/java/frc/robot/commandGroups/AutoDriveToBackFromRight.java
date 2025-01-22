package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoDriveToPose;
import frc.robot.commands.drive.AutoRotateToAngle;


public class AutoDriveToBackFromRight extends SequentialCommandGroup {
  /** Creates a new AutoDriveToBackFromLeft. */
  public AutoDriveToBackFromRight() {

    addCommands(
      new AutoDriveToPose(new Pose2d(2.51,1.2192,new Rotation2d(0)), 0.5, 5),  // Drives straight to the back right of the field
      new AutoDriveToPose(new Pose2d(2.51,4.492,new Rotation2d(0)), 0.5, 5),   // Drives behind the reef
      new AutoDriveToPose(new Pose2d(3,4.492,new Rotation2d(0)), 0.5, 5),      // Drives up to the reef on the back side
      new AutoRotateToAngle(-185, new Translation2d(-0.6, 0), 0),                             // Rotates on an offset center on rotation to pick up coral from the ground
      new AutoDriveToPose(new Pose2d(1.77,4,new Rotation2d(0)), 0.5, 5),       // Drives forward to pick up coral
      new AutoRotateToAngle(0, new Translation2d(0, 0), 0),                      // Rotates towards the reef
      new AutoDriveToPose(new Pose2d(3,4.492,new Rotation2d(0)), 0.5, 5)       // Drives to the reef on the back side
      
      // Drive backwards
      // straff to behind the reef
      // Drive to aprilTag
      // Raise lift
      // Place coral and lower lift
      // Turn around and get coral with algae on top
      // Place coral
      // Get algae out.
      
    );
  }
}
