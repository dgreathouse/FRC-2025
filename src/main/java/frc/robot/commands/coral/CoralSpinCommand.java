// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coral;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.CoralIntakeStates;
import frc.robot.lib.g;

public class CoralSpinCommand extends Command {
  /** Creates a new CoralSpinCommand. */
  CoralIntakeStates m_state;
  double m_timeout = 0.0;
  Timer m_timer = new Timer();

  public CoralSpinCommand(CoralIntakeStates _state, double _timeout) { 
    addRequirements(g.ROBOT.coral);
    m_state = _state;

    m_timeout = _timeout;
    
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    g.CORAL.intakeState = m_state;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_state){
      case IN:
      // TODO: handle range sensor
        g.ROBOT.coral.spinIn();
        break;
      case OUT:
      // TODO: Handle timer
        g.ROBOT.coral.spinOut();
        break;
      case OFF:
        g.ROBOT.coral.spin(0.0);
        break;
    }
    g.ROBOT.coral.rotate(g.CORAL.armState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_state == CoralIntakeStates.OUT){
      return m_timer.hasElapsed(m_timeout);
    }else if(m_state == CoralIntakeStates.IN){
      return g.ROBOT.coral.getRange() < g.CORAL.INTAKE_RANGE_MIN_mm;
    }
    return false;
  }
}
