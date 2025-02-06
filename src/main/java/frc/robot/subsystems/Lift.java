package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.g;

public class Lift extends SubsystemBase implements IUpdateDashboard{
  /** Creates a new ScissorLift. */
  public Lift() {
    g.DASHBOARD.updates.add(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * This will move to a position based on the Coral and Algae arm states.
   * Coral is higher priority at the beginning.
   * 
   */
  public void moveToPosition(){
    switch (g.CORAL.armState) {
      case FLOOR_HORZ:
        break;
      case FLOOR_VERT:
        break;
      case L1:
      // Hopefully do nothing
        break;
      case L2:
      // Hopefully do nothing
        break;
      case L3:
      // Raise to L3
        break;
      case L4:
      // Raise to L4
        break;
      case START:
        // Set to 0 or do nothing
        break;
      default:
        break;

    }
  }
  @Override
  public void updateDashboard() {
  }
}
