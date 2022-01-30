// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final WPI_VictorSPX m_IntakeMotor = new WPI_VictorSPX(6);     
  public final static WPI_TalonFX m_IntakeLiftMotor = new WPI_TalonFX(2); 

  public IntakeSubsystem() {
  }

  public void setIntakeSpeed(double speed) {
      //speed = (speed + 1)/2;
      m_IntakeMotor.set(speed);
  }

  public void setIntakeLiftPosition(boolean liftDown) {
    if (liftDown == true){
      if (m_IntakeLiftMotor.getSelectedSensorPosition() < 10000000)        //1000000000 is a placeholder
      {
         m_IntakeLiftMotor.set(0.25);   //Value of 0.25 subject to change
        //System.out.println(m_IntakeLiftMotor.getSelectedSensorPosition());
      }    
      else{
        m_IntakeLiftMotor.set(0);
      }
  } 
  else{
    m_IntakeLiftMotor.set(0);
  }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
