package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoDriveToPose;
import frc.robot.lib.AprilTagAlignState;
import frc.robot.lib.RobotAlignStates;
import frc.robot.lib.StartLocation;
import frc.robot.lib.g;

public class AutoDriveToPoseTest extends SequentialCommandGroup {
  /** Creates a new AutoDriveRotateTest. */
  public AutoDriveToPoseTest() {
    // Pose X,Y are in Meters, X positive is forward, Y Positive is to the left
    addCommands(
      //new InstantCommand( ()-> {g.ROBOT.drive.setOdometry(StartLocation.RIGHT);}, g.ROBOT.drive),
      //new AutoDriveToPose(new Pose2d(2.51,g.ROBOT. POSE_START_RIGHT.getY(),new Rotation2d(0)), 0.35, 8), // Drives straight to the back left of the field
      //new AutoDriveToPose(new Pose2d(2.51,4.492,new Rotation2d(0)), 0.35, 8) // Drives behind the reef
      //new AutoDriveToPose(new Pose2d(3,4.492,new Rotation2d(0)), 0.35, 8)    // Drives up to the reef on the back side
      new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(20, AprilTagAlignState.RIGHT), 0.3,g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.BACK_LEFT), 3),
      new AutoDriveToPose(g.ROBOT.vision.getRobotPoseForAprilTag(13, AprilTagAlignState.CENTER), 0.3,g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.STATION_LEFT), 5)
      
    );
  }
}
