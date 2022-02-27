// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeLiftSubsystem;

public class LowerIntakeToBottom extends CommandBase {

  private final IntakeLiftSubsystem m_IntakeLiftSubsystem;

  public LowerIntakeToBottom(IntakeLiftSubsystem intakeLift) {

    m_IntakeLiftSubsystem = intakeLift;
    addRequirements(m_IntakeLiftSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeLiftSubsystem.setArmPercent(-.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeLiftSubsystem.setArmPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_IntakeLiftSubsystem.getArmPosition() <= 0;
  }
}