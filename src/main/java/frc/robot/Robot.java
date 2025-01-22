package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commandGroups.AutoDoNothing;
import frc.robot.commandGroups.AutoDriveRotateTest;
import frc.robot.defaultCommands.CoralDefaultCommand;
import frc.robot.defaultCommands.DrivetrainDefaultCommand;
import frc.robot.lib.DriveMode;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.RobotAlignStates;
import frc.robot.lib.g;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private SendableChooser<Command> m_startPoseChooser = new SendableChooser<>();
  private Notifier m_telemetry = new Notifier(this::updateDashboard);

  private DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand();
  private CoralDefaultCommand m_coralDefaultCommand = new CoralDefaultCommand();
 

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Set the default commands for the subsystems
    g.ROBOT.drive.setDefaultCommand(m_drivetrainDefaultCommand);
    g.ROBOT.coral.setDefaultCommand(m_coralDefaultCommand);
    // Configure/Map all the controller buttons to commands
    configureBindings();

    // Setup the autonomous play default and send to dashboard for selection
    m_autoChooser.setDefaultOption("Do Nothing", new AutoDoNothing());
    m_autoChooser.addOption("Drive Rotate Test", new AutoDriveRotateTest());
    SmartDashboard.putData("Autonomouse Play", m_autoChooser);

    m_startPoseChooser.setDefaultOption("Left", new InstantCommand(()->{g.ROBOT.drive.updateOdometry(g.ROBOT.poseStartLeft);}));
    m_autoChooser.addOption("Right", new InstantCommand(()->{g.ROBOT.drive.updateOdometry(g.ROBOT.poseStartRight);}));
    m_autoChooser.addOption("Center", new InstantCommand(()->{g.ROBOT.drive.updateOdometry(g.ROBOT.poseStartCenter);}));
    m_autoChooser.addOption("Zero", new InstantCommand(()->{g.ROBOT.drive.updateOdometry(g.ROBOT.poseStartZero);}));
    // Start telemetry in a slower rate than the main loop
    m_telemetry.startPeriodic(g.ROBOT.TELEMETRY_RATE_sec);
  }

  /**
   * This method is called by m_telemetry which is a Notifier. A Notifier will be executed at a
   * specified rate. The typical rate is 0.1 seconds which is slower that our base rate or 0.02
   * seconds.
   */
  private void updateDashboard() {
    for (IUpdateDashboard updates : g.DASHBOARD.updates) {
      updates.updateDashboard();
    }
    g.MATCH.alliance = DriverStation.getAlliance().get();
  }

  /** */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_autoChooser.getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void configureBindings() {
    // Driver controls
    g.OI.DRIVER_RESET_YAW.onTrue( new InstantCommand(() -> g.ROBOT.drive.resetYaw(0.0), g.ROBOT.drive));
    g.OI.DRIVER_MODE_ANGLEFIELDCENTRIC.onTrue( new InstantCommand(() -> { g.DRIVETRAIN.driveMode = DriveMode.ANGLE_FIELD_CENTRIC;}, g.ROBOT.drive));
    g.OI.DRIVER_MODE_FIELDCENTRIC.onTrue( new InstantCommand(() -> { g.DRIVETRAIN.driveMode = DriveMode.FIELD_CENTRIC; }, g.ROBOT.drive));
    g.OI.DRIVER_MODE_ROBOTCENTRIC.onTrue( new InstantCommand(() -> { g.DRIVETRAIN.driveMode = DriveMode.ROBOT_CENTRIC;}, g.ROBOT.drive));
    g.OI.DRIVER_MODE_SPEED_HI.onTrue( new InstantCommand(() -> { g.DRIVETRAIN.speedMultiplier = 1.0; }, g.ROBOT.drive));
    g.OI.DRIVER_MODE_SPEED_LOW.onTrue( new InstantCommand( () -> { g.DRIVETRAIN.speedMultiplier = 0.5; }, g.ROBOT.drive));
    g.OI.DRIVER_TOGGLE_DRIVETRAIN_ENABLE.onTrue( new InstantCommand( () -> { g.SWERVE.isEnabled = !g.SWERVE.isEnabled; }, g.ROBOT.drive));
    g.OI.DRIVER_DISABLE_YAW.onTrue(new InstantCommand(() -> { g.SIM.IS_GYRO_DISABLED = !g.SIM.IS_GYRO_DISABLED; }, g.ROBOT.drive));

    // Test driver controls
    g.OI.DRIVER_TEST_BB_FRONT.onTrue(new InstantCommand(() -> { g.ROBOT.alignmentState = RobotAlignStates.FRONT; }, g.ROBOT.drive));
    g.OI.DRIVER_TEST_BB_BACK.onTrue(new InstantCommand(() -> { g.ROBOT.alignmentState = RobotAlignStates.BACK; }, g.ROBOT.drive));

    g.OI.DRIVER_TEST_COR_BACK.onTrue(new InstantCommand(() -> {g.DRIVETRAIN.centerOfRotation_m = new Translation2d(-1.0,0.0);}, g.ROBOT.drive));
    g.OI.DRIVER_TEST_COR_RESET.onTrue(new InstantCommand(() -> {g.DRIVETRAIN.centerOfRotation_m = new Translation2d(0,0.0);}, g.ROBOT.drive));
    //Button board
    // g.OI.BB_ALGAE_BARGE.onTrue(new InstantCommand(() ->{ g.ALGAE.armState = AlgaeArmState.BARGE; }, g.ROBOT.algae ));
  
    // g.OI.BB_CORAL_L4.onTrue(new InstantCommand(() ->{ g.CORAL.armState = CoralArmState.L4; }, g.ROBOT.coral ));

    // g.OI.BB_ROBOT_BACK.onTrue(new InstantCommand(() ->{ g.ROBOT.alignmentState = RobotAlignStates.BACK; }, g.ROBOT.drive ));

    // g.OI.BB_APRIL_LEFT.onTrue(new InstantCommand(() ->{ g.VISION.aprilTagButtonState = AprilTagButtonState.LEFT; }, g.ROBOT.drive));
    // g.OI.BB_APRIL_RIGHT.onTrue(new InstantCommand(() ->{ g.VISION.aprilTagButtonState = AprilTagButtonState.RIGHT; }, g.ROBOT.drive));
    // g.OI.BB_APRIL_CENTER.onTrue(new InstantCommand(() ->{ g.VISION.aprilTagButtonState = AprilTagButtonState.CENTER; }, g.ROBOT.drive));

    // g.OI.BB_APRIL_LEFT.onFalse(new InstantCommand(() ->{ g.VISION.aprilTagButtonState = AprilTagButtonState.NONE; }, g.ROBOT.drive));
    // g.OI.BB_APRIL_RIGHT.onFalse(new InstantCommand(() ->{ g.VISION.aprilTagButtonState = AprilTagButtonState.NONE; }, g.ROBOT.drive));
    // g.OI.BB_APRIL_CENTER.onFalse(new InstantCommand(() ->{ g.VISION.aprilTagButtonState = AprilTagButtonState.NONE; }, g.ROBOT.drive));
  }
}
