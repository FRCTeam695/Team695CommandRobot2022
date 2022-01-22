// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.constraint.*;
import edu.wpi.first.wpilibj.controller.*;

import frc.robot.Constants.*;
import frc.robot.commands.*;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.*;
/*import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;*/
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase.*;
import edu.wpi.first.wpilibj2.command.RamseteCommand.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
/*import edu.wpi.first.wpilibj.examples.ramsetecommand.Constants.DriveConstants;
import edu.wpi.first.wpilibj.examples.ramsetecommand.Constants.OIConstants;
import edu.wpi.first.wpilibj.examples.ramsetecommand.subsystems.DriveSubsystem;*/
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick m_Logitech_F310 = new Joystick(0);
  private final DoubleSupplier m_LStickYAxis = () -> (-m_Logitech_F310.getY()); //Java Lambda Expression; In a Logitech F310 pushing forward yields a negative value
  private final DoubleSupplier m_LStickXAxis = () -> (m_Logitech_F310.getX());
  private final DoubleSupplier m_RStickXAxis = () -> (m_Logitech_F310.getRawAxis(4));
  private final NetworkTableInstance RobotMainNetworkTableInstance = NetworkTableInstance.getDefault();
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem(RobotMainNetworkTableInstance, 0);

  private final DriveCommand m_F310_ArcadeDrive = new DriveCommand(m_DriveSubsystem, m_LStickYAxis, m_RStickXAxis);
  private final AimDrivetrain m_AimDrivetrain = new AimDrivetrain(m_LimelightSubsystem);
  private NetworkTable LimeLight;


  private final Command tankDriveSame = new RunCommand(
      () -> {m_DriveSubsystem.tankDriveVolts(
                DriveConstants.kMaxDrivetrainVolts * m_LStickYAxis.getAsDouble(),
                DriveConstants.kMaxDrivetrainVolts * m_LStickYAxis.getAsDouble());},
      m_DriveSubsystem);

  private final Command turnDrive = new RunCommand(
    () -> {m_DriveSubsystem.tankDriveVolts(
                DriveConstants.kMaxDrivetrainVolts * m_RStickXAxis.getAsDouble(),
                -DriveConstants.kMaxDrivetrainVolts * m_RStickXAxis.getAsDouble());},
    m_DriveSubsystem);

  private OptionalDouble lastHeadingWithVision = OptionalDouble.empty();

  private final Command acquireHeadingForTarget = new RunCommand(
    () -> {
          if (LimeLight.getEntry("tv").getNumber(0).intValue() == 1){
            lastHeadingWithVision = OptionalDouble.of(m_DriveSubsystem.getHeading());
          }
    }
  );


  private final Command aimDrivetrain = new RunCommand(
    () -> {
            double Kp = -0.015;         //-0.015 on concrete
            double min_command = 0.075; //0.075 on concrete

            LimeLight = RobotMainNetworkTableInstance.getTable("limelight");

            double forwardValue = (0.25)*applyDeadband(m_LStickYAxis.getAsDouble(),.1);
            double steering_adjust = 0;

            if (LimeLight.getEntry("tv").getNumber(0).intValue() == 1){
                double tx = LimeLight.getEntry("tx").getDouble(0);
                double adjustedMinCommand = Math.max(min_command - Math.abs(forwardValue), 0);
                double heading_error = -tx;
                steering_adjust = Kp * tx;

                if (heading_error > 0){
                  steering_adjust = Kp * heading_error - adjustedMinCommand;
                }
                else if (heading_error < 0){
                  steering_adjust = Kp * heading_error + adjustedMinCommand;
                }
                
              }
                double left_command = steering_adjust + forwardValue;
                double right_command = -steering_adjust + forwardValue;
                m_DriveSubsystem.tankDriveVolts(
                  DriveConstants.kMaxDrivetrainVolts * left_command,
                  DriveConstants.kMaxDrivetrainVolts * right_command);
    },
    m_DriveSubsystem);

    private final JoystickButton B = new JoystickButton(m_Logitech_F310,2);
    private final JoystickButton A = new JoystickButton(m_Logitech_F310,1);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_DriveSubsystem.setDefaultCommand(m_F310_ArcadeDrive);
    //m_DriveSubsystem.setDefaultCommand(turnDrive);
    B.whileHeld(aimDrivetrain);
    A.whileHeld(new TurnToHeading(m_DriveSubsystem, () -> 0));
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond * 0.7,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared * 0.5)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
          
          /*
          //new Translation2d(1, 0),
          //new Translation2d(2, 0)
            new Translation2d(1, 1), //Inverted both y coordinates
            new Translation2d(2, -1)
          //new Translation2d(3,0)
        */

            /*new Translation2d(1,-1),
            new Translation2d(-2,-1.5),
            new Translation2d(-6,-1.5),
            new Translation2d(-7,-1),
            new Translation2d(-5,-.5)*/
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        //new Pose2d(0, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_DriveSubsystem::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_DriveSubsystem::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_DriveSubsystem::tankDriveVolts,
        m_DriveSubsystem
    );

    // Reset odometry to the starting pose of the trajectory.
   m_DriveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
  return ramseteCommand.andThen(() -> m_DriveSubsystem.tankDriveVolts(0, 0));
  }


  //Limelight:
 
 /* private NetworkTableEntry LimeLightAzimuth;
  private NetworkTableEntry LimeLightCoPolar;
  private NetworkTableEntry LimeLightContourArea;*/

  public void printLimelightVars() {
  //Limelight Code
    
      LimeLight = RobotMainNetworkTableInstance.getTable("limelight");
      //this.LimeLightAzimuth = LimeLight.getEntry("tx");
      //this.LimeLightCoPolar = LimeLight.getEntry("ty");
      //this.LimeLightContourArea = LimeLight.getEntry("ta");

    System.out.printf("%.3f%n",LimeLight.getEntry("tx").getDouble(0));
    //System.out.print("");
    //System.out.println(m_Logitech_F310.getRawAxis(4));


/*
    System.out.println(LimeLightCoPolar);
    System.out.print("");

    System.out.println(LimeLightContourArea);
    System.out.print("");
*/
    //System.out.println("TeleopCommand");
     //return m_AimDrivetrain;
  }
  public static double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

}
