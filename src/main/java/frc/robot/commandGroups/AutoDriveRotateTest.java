package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoRotateToAngle;



public class AutoDriveRotateTest extends SequentialCommandGroup {
  /** Creates a new AutoDriveRotateTest. */
  public AutoDriveRotateTest() {

    addCommands(
      new AutoRotateToAngle(90, new Translation2d(0, 0), 2)

    );
  }
}
