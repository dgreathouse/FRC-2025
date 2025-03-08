// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coral;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;

public class CoralReverseCommand extends Command {
  Timer m_timer = new Timer();
  double m_timeOut_sec = 0.1;
  public CoralReverseCommand() {
    addRequirements(g.ROBOT.coral);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    g.ROBOT.coral.spin(-0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_timeOut_sec);
  }
}
