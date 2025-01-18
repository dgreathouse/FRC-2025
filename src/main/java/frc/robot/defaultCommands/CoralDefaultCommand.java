package frc.robot.defaultCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;

public class CoralDefaultCommand extends Command {
  /** Creates a new CoralIntakeDefaultCommand. */
  public CoralDefaultCommand() {
    addRequirements(g.ROBOT.coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int val = (int)SmartDashboard.getNumber("Servo", 2500);
    g.ROBOT.coral.setServo(val);
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
