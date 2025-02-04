package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.g;

public class Algae extends SubsystemBase implements IUpdateDashboard{
  /** Creates a new AlgaeArm. */
  public Algae() {

    //g.DASHBOARD.updates.add(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void updateDashboard() {
   // SmartDashboard.putString("Algae/Intake State", g.ALGAE.intakeState.toString());

  }
}
