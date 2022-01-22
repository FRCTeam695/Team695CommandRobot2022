// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class TurnToHeading extends CommandBase {
  private final DoubleSupplier m_desiredHeading;
  private final DriveSubsystem m_subsystem;
  private final double kP = 0.015;
  
  /** Creates a new TurnToHeading. */
  public TurnToHeading(DriveSubsystem subsystem, DoubleSupplier desiredHeading) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_desiredHeading = desiredHeading;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = m_desiredHeading.getAsDouble() - m_subsystem.getHeading();
    
    m_subsystem.tankDriveVolts(-kP * 11 * error, kP * 11 * error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
