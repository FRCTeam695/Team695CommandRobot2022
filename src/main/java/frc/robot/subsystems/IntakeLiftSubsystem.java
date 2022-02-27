// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeLiftSubsystem extends SubsystemBase {
  /** Creates a new IntakeLiftSubsystem. */
  private final WPI_TalonFX m_IntakeLiftMotor = new WPI_TalonFX(2);

  private static final double ENCODER_POSITION_WHEN_UP = 51000; //Empirically Determined
  public IntakeLiftSubsystem() {
    m_IntakeLiftMotor.setNeutralMode(NeutralMode.Brake);
    resetIntakeLiftPositionToUp();
    SmartDashboard.putData(this);
  }
  //0 is stopped
  //0 to 1 is upward
  //0 to -1 is downward
  public void setArmPercent(double percentVBus){
    m_IntakeLiftMotor.set(percentVBus);
  }
  //1 is up position
  //0 is down position
  public double getArmPosition(){
    return m_IntakeLiftMotor.getSelectedSensorPosition() / ENCODER_POSITION_WHEN_UP;
  }

  public void resetIntakeLiftPositionToUp () {
    m_IntakeLiftMotor.setSelectedSensorPosition(ENCODER_POSITION_WHEN_UP);
  }

  public void resetIntakeLiftPositionToDown () {
    m_IntakeLiftMotor.setSelectedSensorPosition(0);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeLiftPosition", getArmPosition());
  }
}
