// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coral;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralReverseCommand extends Command {
  TalonFXS m_leftMotor;
  TalonFXS m_rightMotor;
  PIDController spinnerPid;
  VoltageOut m_leftVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  VoltageOut m_rightVoltageOut = new VoltageOut(0.0).withEnableFOC(true);

  public CoralReverseCommand() {
    addRequirements(g.ROBOT.coral);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
