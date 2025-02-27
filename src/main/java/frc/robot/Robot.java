package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commandGroups.AutoBlueCenter4;
import frc.robot.commandGroups.AutoBlueLeft244;
import frc.robot.commandGroups.AutoBlueRight244;
import frc.robot.commandGroups.AutoDoNothing;

import frc.robot.commandGroups.AutoDriveToPoseTest;
import frc.robot.commandGroups.AutoRedCenter4;
import frc.robot.commandGroups.AutoRedLeft244;
import frc.robot.commandGroups.AutoRedRight244;
import frc.robot.defaultCommands.CoralDefaultCommand;
import frc.robot.defaultCommands.DrivetrainDefaultCommand;
import frc.robot.defaultCommands.LiftDefaultCommand;
import frc.robot.lib.AlgaeArmState;
import frc.robot.lib.AprilTagAlignState;
import frc.robot.lib.CoralArmState;
import frc.robot.lib.DriveMode;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.RobotAlignStates;
import frc.robot.lib.g;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private Notifier m_telemetry = new Notifier(this::updateDashboard);

  private DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand();
  private CoralDefaultCommand m_coralDefaultCommand = new CoralDefaultCommand();
  private LiftDefaultCommand m_LiftDefaultCommand  = new LiftDefaultCommand();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Set the default commands for the subsystems
    g.ROBOT.drive.setDefaultCommand(m_drivetrainDefaultCommand);
    g.ROBOT.coral.setDefaultCommand(m_coralDefaultCommand);
    g.ROBOT.lift.setDefaultCommand(m_LiftDefaultCommand); 
    

    // Configure/Map all the controller buttons to commands
    configureBindings();

    // Setup the autonomous play default and send to dashboard for selection
    m_autoChooser.setDefaultOption("Do Nothing", new AutoDoNothing());
    m_autoChooser.addOption("Blue Left 244", new AutoBlueLeft244());
    m_autoChooser.addOption("Blue Right 244", new AutoBlueRight244());
    m_autoChooser.addOption("Red Right 244", new AutoRedRight244());
    m_autoChooser.addOption("Red Right 244", new AutoRedLeft244());
    m_autoChooser.addOption("Red Center 4", new AutoRedCenter4());
    m_autoChooser.addOption("Blue Center 4", new AutoBlueCenter4());
    m_autoChooser.addOption("Drive To Pose Test", new AutoDriveToPoseTest());
    SmartDashboard.putData("Autonomouse Play", m_autoChooser);

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
    
  }

  /** */
  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}
 //boolean flag = false;
  @Override
  public void disabledPeriodic() {

  }

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
    //g.ROBOT.drive.resetYaw(g.VISION.pose2d.getRotation().getDegrees());
    // g.ROBOT.drive.setOdometry(StartLocation.RIGHT);
    // g.ROBOT.vision.setOdometry(StartLocation.RIGHT);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

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
    g.OI.DRIVER_TOGGLE_DRIVETRAIN_ENABLE.onTrue( new InstantCommand( () -> { g.SWERVE.isEnabled = !g.SWERVE.isEnabled; }, g.ROBOT.drive));
    g.OI.DRIVER_STATION_RIGHT.onTrue( new InstantCommand(() -> {g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.STATION_RIGHT); }, g.ROBOT.drive));
    g.OI.DRIVER_STATION_LEFT.onTrue( new InstantCommand(() -> {g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.STATION_LEFT);}, g.ROBOT.drive));
    g.OI.DRIVER_TOGGLE_AUTO_DRIVE.onTrue(new InstantCommand(() -> {g.DRIVETRAIN.isAutoDriveEnabled = !g.DRIVETRAIN.isAutoDriveEnabled;}));
    g.OI.DRIVER_MODE_SPEED_TOGGLE.onTrue(new InstantCommand(() -> {g.ROBOT.drive.toggleSpeed();}));

    // Operator controls
    g.OI.OPERATOR_ALIGN_CENTER.onTrue(new InstantCommand(()-> {g.ROBOT.drive.setAprilTagAlignment(AprilTagAlignState.CENTER);}, g.ROBOT.drive));
    g.OI.OPERATOR_ALIGN_LEFT.onTrue(new InstantCommand(()-> {g.ROBOT.drive.setAprilTagAlignment(AprilTagAlignState.LEFT);}, g.ROBOT.drive));
    g.OI.OPERATOR_ALIGN_RIGHT.onTrue(new InstantCommand(()-> {g.ROBOT.drive.setAprilTagAlignment(AprilTagAlignState.RIGHT);}, g.ROBOT.drive));


    //Button board
    //  g.OI.BB_ALGAE_BARGE.onTrue(new InstantCommand(() ->{ g.ALGAE.armState = AlgaeArmState.BARGE; }, g.ROBOT.algae ));
    //  g.OI.BB_ALGAE_HIGH.onTrue(new InstantCommand(() ->{ g.ALGAE.armState = AlgaeArmState.HIGH; }, g.ROBOT.algae ));
    //  g.OI.BB_ALGAE_LOW.onTrue(new InstantCommand(() ->{ g.ALGAE.armState = AlgaeArmState.LOW; }, g.ROBOT.algae ));
    //  g.OI.BB_ALGAE_PROCESSOR.onTrue(new InstantCommand(() ->{ g.ALGAE.armState = AlgaeArmState.PROCESSOR; }, g.ROBOT.algae ));
    //  g.OI.BB_ALGAE_FLORAL.onTrue(new InstantCommand(() ->{ g.ALGAE.armState = AlgaeArmState.FLORAL; }, g.ROBOT.algae ));
    //  g.OI.BB_ALGAE_FLOOR.onTrue(new InstantCommand(() ->{ g.ALGAE.armState = AlgaeArmState.FLOOR; }, g.ROBOT.algae ));
    //  g.OI.BB_ALGAE_START.onTrue(new InstantCommand(() ->{ g.ALGAE.armState = AlgaeArmState.START; }, g.ROBOT.algae ));

     //g.OI.BB_CORAL_L4.onTrue(new InstantCommand(() ->{ g.CORAL.armState = CoralArmState.ALGAE_HIGH; }, g.ROBOT.coral ));
     g.OI.BB_CORAL_L3.onTrue(new InstantCommand(() ->{ g.CORAL.armState = CoralArmState.L3; }, g.ROBOT.coral ));
     g.OI.BB_CORAL_L2.onTrue(new InstantCommand(() ->{ g.CORAL.armState = CoralArmState.L2; }, g.ROBOT.coral ));
     g.OI.BB_CORAL_L1.onTrue(new InstantCommand(() ->{ g.CORAL.armState = CoralArmState.L1; }, g.ROBOT.coral ));
    //  g.OI.BB_CORAL_FLOOR_HORZ.onTrue(new InstantCommand(() ->{ g.CORAL.armState = CoralArmState.FLOOR_HORZ; }, g.ROBOT.coral ));
    //  g.OI.BB_CORAL_FLOOR_VERT.onTrue(new InstantCommand(() ->{ g.CORAL.armState = CoralArmState.FLOOR_VERT; }, g.ROBOT.coral ));
     g.OI.BB_CORAL_START.onTrue(new InstantCommand(() ->{ g.CORAL.armState = CoralArmState.START; }, g.ROBOT.coral ));

     g.OI.BB_ROBOT_BACK.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.BACK); }, g.ROBOT.drive ));
     g.OI.BB_ROBOT_BACK_RIGHT.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.BACK_RIGHT); }, g.ROBOT.drive ));
     g.OI.BB_ROBOT_BACK_LEFT.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.BACK_LEFT); }, g.ROBOT.drive ));
     g.OI.BB_ROBOT_FRONT.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.FRONT); }, g.ROBOT.drive ));
     g.OI.BB_ROBOT_FRONT_RIGHT.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.FRONT_RIGHT); }, g.ROBOT.drive ));
     g.OI.BB_ROBOT_FRONT_LEFT.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.FRONT_LEFT); }, g.ROBOT.drive ));

     g.OI.BB_ROBOT_STATION_LEFT.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.STATION_LEFT); }, g.ROBOT.drive ));
     g.OI.BB_ROBOT_STATION_RIGHT.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.STATION_RIGHT); }, g.ROBOT.drive ));

     g.OI.BB_ROBOT_RIGHT.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.RIGHT); }, g.ROBOT.drive ));
     g.OI.BB_ROBOT_LEFT.onTrue(new InstantCommand(() ->{ g.ROBOT.drive.setTargetRobotAngle(RobotAlignStates.LEFT); }, g.ROBOT.drive ));

     g.OI.BB_APRIL_LEFT.onTrue(new InstantCommand(() ->{ g.VISION.aprilTagAlignState = AprilTagAlignState.LEFT; }, g.ROBOT.drive));
     g.OI.BB_APRIL_RIGHT.onTrue(new InstantCommand(() ->{ g.VISION.aprilTagAlignState = AprilTagAlignState.RIGHT; }, g.ROBOT.drive));
     g.OI.BB_APRIL_CENTER.onTrue(new InstantCommand(() ->{ g.VISION.aprilTagAlignState = AprilTagAlignState.CENTER; }, g.ROBOT.drive));

  }
}
