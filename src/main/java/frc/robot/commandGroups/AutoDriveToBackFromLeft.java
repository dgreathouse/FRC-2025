// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AutoDriveToBackFromLeft extends SequentialCommandGroup {
  /** Creates a new AutoDriveToBackFromLeft. */
  public AutoDriveToBackFromLeft() {

    addCommands(
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
