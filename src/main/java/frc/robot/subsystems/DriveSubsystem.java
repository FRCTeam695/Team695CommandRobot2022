// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_leftmotor1 = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
  private final WPI_TalonFX m_leftmotor2 = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);

  private final WPI_TalonFX m_rightmotor1 = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
  private final WPI_TalonFX m_rightmotor2 = new WPI_TalonFX(DriveConstants.kRightMotor2Port);

  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors;

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors;

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    setNeutralMode(NeutralMode.Brake);
    m_leftMotors = new MotorControllerGroup(m_leftmotor1, m_leftmotor2);
    m_rightMotors = new MotorControllerGroup(m_rightmotor1, m_rightmotor2);
    m_rightMotors.setInverted(true);
    resetEncoders();

    resetGyro();

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0);
  }

  public void resetGyro() {
    m_gyro.calibrate();
    m_gyro.reset();
  }

  public void setNeutralMode(NeutralMode neutralMode) {
    m_leftmotor1.setNeutralMode(neutralMode);
    m_leftmotor2.setNeutralMode(neutralMode);
    m_rightmotor1.setNeutralMode(neutralMode);
    m_rightmotor2.setNeutralMode(neutralMode);
  }

    @Override
    public void periodic() {
      // Update the odometry in the periodic block
      // JPK
      m_odometry.update(m_gyro.getRotation2d(),
                        m_leftmotor1.getSelectedSensorPosition() * DriveConstants.kMetersRobotTravelPerEncoderCount,
                        -1* m_rightmotor1.getSelectedSensorPosition() * DriveConstants.kMetersRobotTravelPerEncoderCount);   
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
      return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(
        m_leftmotor1.getSelectedSensorVelocity() * DriveConstants.kMetersRobotTravelPerSecondPerTalonUnits,
        -1* m_rightmotor1.getSelectedSensorVelocity() * DriveConstants.kMetersRobotTravelPerSecondPerTalonUnits
        );
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
      resetEncoders();
      m_odometry.resetPosition(m_gyro.getRotation2d(), 0, 0, pose);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
      m_leftMotors.setVoltage(leftVolts);
      m_rightMotors.setVoltage(rightVolts);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
      m_leftmotor1.setSelectedSensorPosition(0, 0, 20);
      m_rightmotor1.setSelectedSensorPosition(0, 0, 20);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
      return  (m_leftmotor1.getSelectedSensorPosition() * DriveConstants.kMetersRobotTravelPerEncoderCount
       + (-1) * m_rightmotor1.getSelectedSensorPosition() * DriveConstants.kMetersRobotTravelPerEncoderCount)
      / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    
    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
      return m_gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
      return -m_gyro.getRate();
    }
}
