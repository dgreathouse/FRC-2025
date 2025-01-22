// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoDriveToPose;
import frc.robot.commands.drive.AutoRotateToAngle;


public class AutoDriveToFront extends SequentialCommandGroup {
  /** Creates a new AutoDriveToBackFromLeft. */
  public AutoDriveToFront() {

    addCommands(
      new AutoRotateToAngle(-180, new Translation2d(0, 0), 0),                       // Rotates toward the reef
      new AutoDriveToPose(new Pose2d(1.84,4,new Rotation2d(0)), 0.5, 5) // Drives straight to the front of the reef
      
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
