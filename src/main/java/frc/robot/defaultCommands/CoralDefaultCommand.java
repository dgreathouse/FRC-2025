package frc.robot.defaultCommands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;

public class CoralDefaultCommand extends Command {
  /** Creates a new CoralIntakeDefaultCommand. */
  public CoralDefaultCommand() {
    addRequirements(g.ROBOT.coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(g.OI.DRIVER_CORAL_IN.getAsBoolean()){
      if(g.ROBOT.coral.getRange() < 70){ // TODO: adjust range sensor
        g.ROBOT.coral.spin(0.25);
      }else {
        g.ROBOT.coral.spin(0.0);
      }
    }else if(g.OI.DRIVER_CORAL_OUT.getAsBoolean()){
      g.ROBOT.coral.spin(0.25);
    }else { // OFF
      g.ROBOT.coral.spin(0.0);
    }
    g.ROBOT.coral.rotate(g.CORAL.armState);
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
