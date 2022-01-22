// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Limelight Imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new Limelight. */
  private NetworkTable LimeLight;
  private NetworkTableEntry LimeLightAzimuth;
  private NetworkTableEntry LimeLightCoPolar;
  private NetworkTableEntry LimeLightContourArea;

  public LimelightSubsystem(NetworkTableInstance RobotMainNetworkTableInstance, int driverNum) {
    this.LimeLight = RobotMainNetworkTableInstance.getTable("limelight");
    this.LimeLightAzimuth = LimeLight.getEntry("tx");
    this.LimeLightCoPolar = LimeLight.getEntry("ty");
    this.LimeLightContourArea = LimeLight.getEntry("ta");
  }

  public double getAzimuth(){
    return LimeLightAzimuth.getDouble(0.0);
  }

  public double getCoPolar(){
    return LimeLightCoPolar.getDouble(0.0);
  }

  public double getContourArea(){
    return LimeLightContourArea.getDouble(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
