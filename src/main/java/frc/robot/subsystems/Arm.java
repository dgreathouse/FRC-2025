package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;

public class Arm extends SubsystemBase {
  ArmFeedforward ff;
  public Arm() {
    ff = new ArmFeedforward(0, 0, 0);
    Angle x = Degrees.of(45);
    x.in(Radians);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
