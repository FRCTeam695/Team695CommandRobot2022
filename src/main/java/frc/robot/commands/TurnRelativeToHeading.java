// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnRelativeToHeading extends CommandBase {
  /** Creates a new TurnRelativeToHeading. */
  private static final int kMaxErrorForCompletionInDegrees = 1;
  private final DriveSubsystem m_drivetrain;
  private final double m_differenceInAngle;
  private double m_desiredAngle;
  private final double kP = 0.015;
  private double m_error;

  public TurnRelativeToHeading(DriveSubsystem drivetrain, double differenceInAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_differenceInAngle = differenceInAngle;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_desiredAngle = m_drivetrain.getHeading() + m_differenceInAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desiredAngle = m_desiredAngle;
    double currentAngle = m_drivetrain.getHeading();

    m_error = desiredAngle - currentAngle;

    double driveVolts = kP * DriveConstants.kMaxDrivetrainVolts * m_error;
    double maxDriveVolts = 3.0;
    if (driveVolts > maxDriveVolts){
      driveVolts = maxDriveVolts;
    }
    else if (driveVolts < -maxDriveVolts){
      driveVolts = -maxDriveVolts;
    }
    m_drivetrain.tankDriveVolts(
      -driveVolts,
      driveVolts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_error) < kMaxErrorForCompletionInDegrees;
  }
}
