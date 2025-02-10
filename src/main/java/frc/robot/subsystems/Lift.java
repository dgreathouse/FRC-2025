package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.g;

public class Lift extends SubsystemBase implements IUpdateDashboard{
  TalonFX m_motor;
  /** Creates a new ScissorLift. */
  public Lift() {
    g.DASHBOARD.updates.add(this);
    m_motor = new TalonFX(10, g.CAN_IDS_ROBORIO.NAME);
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
