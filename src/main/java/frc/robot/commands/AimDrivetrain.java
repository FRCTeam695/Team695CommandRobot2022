// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class AimDrivetrain extends CommandBase {
  private LimelightSubsystem Limelight_Inst;
  /** Creates a new AimDrivetrain. */
 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AimDrivetrain(LimelightSubsystem Limelight_Inst) {
    System.out.println(Limelight_Inst.getAzimuth());
    System.out.print("");

    System.out.println(Limelight_Inst.getCoPolar());
    System.out.print("");

    System.out.println(Limelight_Inst.getContourArea());
    System.out.print("");
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
