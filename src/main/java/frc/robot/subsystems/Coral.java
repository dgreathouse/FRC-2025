package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.g;

public class Coral extends SubsystemBase implements IUpdateDashboard{
  /** Creates a new CoralIntake. */
  public Coral() {
    g.DASHBOARD.updates.add(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  @Override
  public void updateDashboard() {
    SmartDashboard.putString("Coral/Coral Claw State", g.CORAL.clawState.toString());
  }
}
