// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coral.CoralMoveToStateCommand;
import frc.robot.commands.coral.CoralSpinInCommand;
import frc.robot.commands.coral.CoralSpinOutCommand;
import frc.robot.commands.drive.AutoDriveToPose;
import frc.robot.commands.drive.AutoRotateToPose;
import frc.robot.commands.drive.AutoDriveDelay;
import frc.robot.lib.AprilTagAlignState;
import frc.robot.lib.CoralArmState;
import frc.robot.lib.g;

public class AutoBlueRight2 extends SequentialCommandGroup {
  public double m_delay;
  /** Creates a new AutoBlueRight2. */
  public AutoBlueRight2(double _delay) {
    m_delay = _delay;

    addCommands(
      new AutoDriveDelay(m_delay),
      new AutoRotateToPose(g.ROBOT.vision.getRobotPoseForAprilTag(22, AprilTagAlignState.LEFT), .3, 1),
      new ParallelDeadlineGroup(
        new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(22, AprilTagAlignState.LEFT), 0.5, 3),
        new CoralMoveToStateCommand(CoralArmState.L2)
      ),
      new CoralSpinOutCommand(CoralArmState.L2, 1.5)
      // new AutoRotateToPose(g.ROBOT.vision.getRobotPoseForAprilTag(12, AprilTagAlignState.RIGHT), .3, 1),
      //  new ParallelDeadlineGroup(
      //   new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(12, AprilTagAlignState.RIGHT), 0.5, 7),
      //   new CoralMoveToStateCommand(CoralArmState.START)
      // ),
      // new CoralSpinInCommand(CoralArmState.START, 2),
      // new AutoDriveToPose(new Pose2d(2,4, new Rotation2d(0)),0.3, 3)
    );
  }
}
