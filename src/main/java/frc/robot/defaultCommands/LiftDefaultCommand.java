package frc.robot.defaultCommands;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;

public class LiftDefaultCommand extends Command {
  // PID in this default command only lowers the lift and maintains 0
  PIDController m_pid = new PIDController(0, 0, 0);
  // An elevator FF may not work since the kg changes as the lift is lowered and raised.
  // An interpolation table will probably be needed
  ElevatorFeedforward m_ff = new ElevatorFeedforward(0, 0, 0);
  TalonFX m_motor = new TalonFX(60);
  VoltageOut m_voltageOut = new VoltageOut(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true);
  /** Creates a new ScissorLiftDefaultCommand. 
   * While default running the lift should being maintaining 0 position.
   * Maintaing the position is done through a PID and FF from current position to expected.
   * During the movement a button trigger take over the movement to the state of coral or algae
   * When button trigger is released the state should be reset to 0 and the PID/FF should take over and go back to 0. Gently
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
