// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.CoralArmState;
import frc.robot.lib.g;


public class LiftMoveCommand extends Command {
  CoralArmState m_state;
  Timer m_timer = new Timer();
  double m_startDelayTime_sec = 0;
  /** Creates a new LiftMoveCommand. */
  public LiftMoveCommand(CoralArmState _state, double _startDelayTime_sec) {
    addRequirements(g.ROBOT.lift);
    m_state = _state;
    m_startDelayTime_sec = _startDelayTime_sec;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_timer.hasElapsed(m_startDelayTime_sec)){
      g.ROBOT.lift.moveToPosition(m_state);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
