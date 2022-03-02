// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private final WPI_TalonFX m_ClimberMotor1 = new WPI_TalonFX(5);
  private final WPI_TalonFX m_ClimberMotor2 = new WPI_TalonFX(6);


  public ClimberSubsystem() {
    m_ClimberMotor1.setNeutralMode(NeutralMode.Brake);
    m_ClimberMotor2.setNeutralMode(NeutralMode.Brake);
  }

  public void setClimber1Percentage(double percentVBus){
    m_ClimberMotor1.set(percentVBus);
  }

  public void setClimber2Percentage(double percentVBus){
    m_ClimberMotor2.set(percentVBus);
  }

  public void turnClimberOff(){
    m_ClimberMotor1.set(0);
    m_ClimberMotor2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
