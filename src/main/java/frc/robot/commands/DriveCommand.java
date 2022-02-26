// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  private final DriveSubsystem m_drivetrain;

  private final DoubleSupplier m_xSpeed, m_zRotation;

  /**
   * Creates a new ExampleCommand.
   *
   * @param drivetrain The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem drivetrain, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    m_drivetrain = drivetrain;
    m_xSpeed = xSpeed;
    m_zRotation = zRotation;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_xSpeed.getAsDouble()*.6, m_zRotation.getAsDouble()*.6);
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
