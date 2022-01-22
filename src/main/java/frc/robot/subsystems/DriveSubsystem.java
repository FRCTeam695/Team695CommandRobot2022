// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

//import edu.wpi.first.wpilibj.examples.ramsetecommand.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_leftmotor1 = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
  private final WPI_TalonFX m_leftmotor2 = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);

  private final WPI_TalonFX m_rightmotor1 = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
  private final WPI_TalonFX m_rightmotor2 = new WPI_TalonFX(DriveConstants.kRightMotor2Port);

  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(m_leftmotor1, m_leftmotor2);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(m_rightmotor1, m_rightmotor2);



/*
  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(new WPI_TalonFX(DriveConstants.kLeftMotor1Port),
                               new WPI_TalonFX(DriveConstants.kLeftMotor2Port));

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(new WPI_TalonFX(DriveConstants.kRightMotor1Port),
                               new WPI_TalonFX(DriveConstants.kRightMotor2Port));
*/
  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
//  private final Encoder m_leftEncoder =
//      new Encoder(11, 12, false);

  // The right-side drive encoder
//  private final Encoder m_rightEncoder =
//      new Encoder(13, 14, false);

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
 


  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
//    m_leftEncoder.setDistancePerPulse(.15*3.14/(2048*8));
//    m_rightEncoder.setDistancePerPulse(.15*3.14/(2048*8));

   resetEncoders();

    m_gyro.calibrate();
    m_gyro.reset();

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

  }

    @Override
    public void periodic() {
      // Update the odometry in the periodic block
//      m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(),
//                        m_rightEncoder.getDistance());

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
 //     return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
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
      m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
      m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
      m_leftMotors.setVoltage(leftVolts);
      m_rightMotors.setVoltage(-rightVolts);
      m_drive.feed();
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
      m_leftmotor1.setSelectedSensorPosition(0, 0, 20);
      m_rightmotor1.setSelectedSensorPosition(0, 0, 20);
      //m_leftEncoder.reset();
      //m_rightEncoder.reset();
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
    /*public Encoder getLeftEncoder() {
      return m_leftEncoder;
    }
*/
    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    /*public Encoder getRightEncoder() {
      return m_rightEncoder;
    }
*/
    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
      m_drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    /*public void zeroHeading() {
      m_gyro.reset();
    }*/

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
