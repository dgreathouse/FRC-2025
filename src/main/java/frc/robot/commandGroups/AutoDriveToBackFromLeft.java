// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoDriveToPose;
import frc.robot.commands.drive.AutoRotateToAngle;
import frc.robot.lib.StartLocation;
import frc.robot.lib.g;


public class AutoDriveToBackFromLeft extends SequentialCommandGroup {
  /** Creates a new AutoDriveToBackFromLeft. */
  public AutoDriveToBackFromLeft() {

    addCommands(
      new InstantCommand( ()-> {g.ROBOT.drive.setOdometry(StartLocation.LEFT);}, g.ROBOT.drive),
      new AutoDriveToPose(new Pose2d(2.51,g.ROBOT.POSE_START_LEFT.getY(),new Rotation2d(0)), 0.5, 5),  // Drives straight to the back left of the field
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
