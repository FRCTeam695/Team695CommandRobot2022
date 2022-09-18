// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeLiftSubsystem;

public class ActuateClimberTwo extends CommandBase {

  private final ClimberSubsystem m_ClimberSubsystem;
  private final IntakeLiftSubsystem m_IntakeLiftSubsystem;

  private double climberPercent;
  private double intakeLiftPosition;

  public ActuateClimberTwo(ClimberSubsystem climberSubsystem, IntakeLiftSubsystem intakeLiftSubsystem, double climberSpeed) {
    m_ClimberSubsystem = climberSubsystem;
    m_IntakeLiftSubsystem = intakeLiftSubsystem;
    addRequirements(m_ClimberSubsystem);
    addRequirements(m_IntakeLiftSubsystem);

    climberPercent = climberSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeLiftPosition = m_IntakeLiftSubsystem.intakeLiftPosition;
    if(intakeLiftPosition <= 0.9){
      m_ClimberSubsystem.setClimber2Percentage(climberPercent);
    }
    else{
      m_ClimberSubsystem.setClimber2Percentage(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ClimberSubsystem.setClimber2Percentage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
