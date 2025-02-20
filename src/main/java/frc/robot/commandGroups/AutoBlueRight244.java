// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Lift.LiftMoveCommand;
import frc.robot.commands.coral.CoralMoveToStateCommand;
import frc.robot.commands.coral.CoralSpinCommand;
import frc.robot.commands.drive.AutoDriveToPose;
import frc.robot.lib.AprilTagAlignState;
import frc.robot.lib.CoralArmState;
import frc.robot.lib.CoralIntakeStates;
import frc.robot.lib.g;

/**
 * 
 */
public class AutoBlueRight244 extends SequentialCommandGroup {
  /** Creates a new AutoBlueRight244. */
  public AutoBlueRight244() {

    addCommands(
      // Drive to first coral for L2
      new ParallelDeadlineGroup(
        new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(22, AprilTagAlignState.LEFT), 0.6, 3)
       // new CoralMoveToStateCommand(CoralArmState.L2)
      ),
      new CoralSpinCommand(CoralIntakeStates.OUT, 1),
      // Drive to Station on the left
      new ParallelDeadlineGroup(
        new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(12, AprilTagAlignState.LEFT), 0.6, 4)
    //    new CoralMoveToStateCommand(CoralArmState.START)
      ),
      new CoralSpinCommand(CoralIntakeStates.IN, 2),
      // Drive to reef for L4 on the right
      new ParallelDeadlineGroup(
        new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(17, AprilTagAlignState.RIGHT), 0.6, 4)
     //   new CoralMoveToStateCommand(CoralArmState.L4),
      //  new LiftMoveCommand(CoralArmState.L4, 2)
      ),
      new CoralSpinCommand(CoralIntakeStates.OUT, 1),
      // Drive to Station on the left
      new ParallelDeadlineGroup(
        new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(12, AprilTagAlignState.LEFT), 0.6, 4)
     //   new CoralMoveToStateCommand(CoralArmState.START),
      //  new LiftMoveCommand(CoralArmState.START, 0)
      ),
      new CoralSpinCommand(CoralIntakeStates.IN, 2),
      // Drive to reef for L4 on the LEFT
      new ParallelDeadlineGroup(
        new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(17, AprilTagAlignState.LEFT), 0.6, 4)
    //    new CoralMoveToStateCommand(CoralArmState.L4),
      //  new LiftMoveCommand(CoralArmState.L4, 2)
      ),
      new CoralSpinCommand(CoralIntakeStates.OUT, 1)
    //  new LiftMoveCommand(CoralArmState.START, 0),
    //  new CoralMoveToStateCommand(CoralArmState.START)
    );
  }
}
