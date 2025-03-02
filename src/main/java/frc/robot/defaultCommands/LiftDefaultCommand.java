package frc.robot.defaultCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.CoralArmState;
import frc.robot.lib.g;

public class LiftDefaultCommand extends Command {

  /**
   * Creates a new LiftDefaultCommand.
   */
  public LiftDefaultCommand() {
    addRequirements(g.ROBOT.lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(g.OI.driverController.getR2Axis() > 0){
    g.ROBOT.lift.moveToPosition(g.CORAL.armState);
    }else {
        g.ROBOT.lift.moveToPosition(CoralArmState.START);
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
