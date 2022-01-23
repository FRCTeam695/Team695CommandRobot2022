// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.*;


public class TurnToHeading extends CommandBase {
  private final DoubleSupplier m_desiredHeading;
  private final DriveSubsystem m_drivetrain;
  private final double kP = 0.015;
  
  /** Creates a new TurnToHeading. */
  public TurnToHeading(DriveSubsystem drivetrain, DoubleSupplier desiredHeading) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_desiredHeading = desiredHeading;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double halfwayPoint = 180.0;
    double desiredHeading = m_desiredHeading.getAsDouble();
    double adjustedDesiredHeading = desiredHeading;
    double currentHeading = m_drivetrain.getHeading();
    double differenceInHeading = currentHeading - desiredHeading;
    double adjustedDifferenceInHeading = differenceInHeading;

    double circleValue = 360.0;

    

    while (adjustedDifferenceInHeading < -180){
      adjustedDesiredHeading = adjustedDesiredHeading - 360;
      adjustedDifferenceInHeading = currentHeading - adjustedDesiredHeading;
    }
    while (adjustedDifferenceInHeading > 180){
      adjustedDesiredHeading = adjustedDesiredHeading + 360;
      adjustedDifferenceInHeading = currentHeading - adjustedDesiredHeading;
    }

    double error = adjustedDesiredHeading - currentHeading;
    System.out.printf("%.3f           %.3f           %.3f", currentHeading, desiredHeading, adjustedDesiredHeading);
    System.out.println();

    double driveVolts = kP * DriveConstants.kMaxDrivetrainVolts * error;
    double maxDriveVolts = 5.0;
    if (driveVolts > maxDriveVolts){
      driveVolts = maxDriveVolts;
    }
    else if (driveVolts < -maxDriveVolts){
      driveVolts = -maxDriveVolts;
    }
    m_drivetrain.tankDriveVolts(
      -driveVolts,
      driveVolts);

    //System.out.println("turning to: " + m_drivetrain.getHeading());
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
