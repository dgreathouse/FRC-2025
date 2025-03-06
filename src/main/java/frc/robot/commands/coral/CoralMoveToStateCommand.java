package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.CoralArmState;
import frc.robot.lib.g;


public class CoralMoveToStateCommand extends Command {
  CoralArmState m_state;
  /** Creates a new CoralMoveToStateCommand. */
  public CoralMoveToStateCommand(CoralArmState _state) {
    m_state = _state;
    addRequirements(g.ROBOT.coral); 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    g.ROBOT.coral.rotate(m_state);
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
