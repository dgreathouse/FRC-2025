// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoDriveToPose;
import frc.robot.lib.AprilTagAlignState;
import frc.robot.lib.g;

public class AutoRedRight244 extends SequentialCommandGroup {
  /** Creates a new AutoRedRight244. */
  public AutoRedRight244() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(9, AprilTagAlignState.LEFT), 0.6, 3)
    );
  }
}
