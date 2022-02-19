// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeLiftSubsystem extends SubsystemBase {
  /** Creates a new IntakeLiftSubsystem. */
  public final static WPI_TalonFX m_IntakeLiftMotor = new WPI_TalonFX(2);

  public IntakeLiftSubsystem() {}

  boolean liftDown = true;
  boolean liftUp = false;

  public void moveIntakeLiftUp(boolean liftDown) {
    if (liftDown == true){
      if (m_IntakeLiftMotor.getSelectedSensorPosition() < 58508)
      {
         m_IntakeLiftMotor.set(0.5);   //Value of 0.5 subject to change
        //System.out.println(m_IntakeLiftMotor.getSelectedSensorPosition());
      }    
      else{
        m_IntakeLiftMotor.set(0);
        liftUp = true;
      }
  } 
  else{
    m_IntakeLiftMotor.set(0);
  }
}

  public void moveIntakeLiftDown(boolean liftUp)  {
    if (liftUp == true){
      if (m_IntakeLiftMotor.getSelectedSensorPosition() > 0)
      {
        m_IntakeLiftMotor.set(-0.5);
      }
      else{
        m_IntakeLiftMotor.set(0);
        liftDown = true;
      }
    }
  }

  public void setIntakeLiftPositionToZero () {
    m_IntakeLiftMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
