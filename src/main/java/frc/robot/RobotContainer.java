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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeLiftSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.constraint.*;

import frc.robot.Constants.*;
import frc.robot.commands.*;

import static edu.wpi.first.wpilibj.XboxController.Button;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
/*import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;*/
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase.*;
import edu.wpi.first.wpilibj2.command.RamseteCommand.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
/*import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;*/
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;
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
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final IntakeLiftSubsystem m_IntakeLiftSubsystem = new IntakeLiftSubsystem();

  private final Joystick m_Logitech_F310 = new Joystick(0);
  private final Joystick m_Extreme_3D_Pro_1 = new Joystick(2);
  private final Joystick m_Extreme_3D_Pro_2 = new Joystick(3);
  private final DoubleSupplier m_LStickYAxis = () -> (-m_Logitech_F310.getY()); //Java Lambda Expression; In a Logitech F310 pushing forward yields a negative value
  //private final DoubleSupplier m_LStickXAxis = () -> (m_Logitech_F310.getX());
  private final DoubleSupplier m_RStickXAxis = () -> (m_Logitech_F310.getRawAxis(4));

  private final DoubleSupplier m_LeftStickYAxis = () -> (-applyDeadband(m_Extreme_3D_Pro_1.getRawAxis(1), Constants.OIConstants.kExtreme3DProDeadband));
  private final DoubleSupplier m_LeftStickXAxis = () -> (applyDeadband(m_Extreme_3D_Pro_1.getRawAxis(0),  Constants.OIConstants.kExtreme3DProDeadband));
  private final DoubleSupplier m_RightStickXAxis = () -> (applyDeadband(m_Extreme_3D_Pro_2.getRawAxis(0),  Constants.OIConstants.kExtreme3DProDeadband));
  private final DoubleSupplier m_LeftStickSlider = () -> (m_Extreme_3D_Pro_1.getRawAxis(3));

  private final DoubleSupplier m_LeftStickTwistValue = () -> (m_Extreme_3D_Pro_1.getRawAxis(2));

  private final NetworkTableInstance RobotMainNetworkTableInstance = NetworkTableInstance.getDefault();
  private final DriveSubsystem m_drivetrain = new DriveSubsystem();
  private final Limelight m_LimelightSubsystem = new Limelight(RobotMainNetworkTableInstance, 0);

  private final DriveCommand m_F310_ArcadeDrive = new DriveCommand(m_drivetrain, m_LStickYAxis, m_RStickXAxis);
  private NetworkTable LimeLight;

  private final Command m_IntakeMotorRunCommand = new RunCommand(
     () -> {
        m_IntakeSubsystem.setIntakeSpeed(
          m_LeftStickTwistValue.getAsDouble()
        );
        System.out.println(m_LeftStickTwistValue.getAsDouble());
     },
  m_IntakeSubsystem);

  private final Command m_IntakeMotorLiftRunCommand = new RunCommand(
     () -> {
       m_IntakeLiftSubsystem.moveIntakeLiftUp(
         true
       );
       System.out.println(IntakeLiftSubsystem.m_IntakeLiftMotor.getSelectedSensorPosition());
     },
  m_IntakeSubsystem);

  private final Command m_IntakeMotorLiftRunCommandDisengageCommand = new RunCommand(
    () -> {
      m_IntakeLiftSubsystem.moveIntakeLiftUp(
        false
      );
    },
  m_IntakeSubsystem);

  /*private final Command m_F310_CurvatureDrive = new RunCommand(
      () -> {
              m_drivetrain.curvatureDrive
              (
                m_LStickYAxis.getAsDouble(), 
                m_RStickXAxis.getAsDouble()
               );
      },
      m_drivetrain);*/

  private final Command m_Extreme_3D_Pro_CurvatureDrive = new RunCommand(
      () -> {
        double fwd = m_LeftStickYAxis.getAsDouble();
        double rot = m_RightStickXAxis.getAsDouble();
        //double deadband = DifferentialDrive.kDefaultDeadband;
        double adjustedRot = rot;
        double deadband = 0.2;
        //deadband = (deadband + 1)/2;
        //System.out.println(deadband);

        boolean isQuickTurn =  fwd < deadband && fwd > -deadband;
        if (isQuickTurn){
          adjustedRot = rot * Math.abs(rot);
        }

        //m_drive.curvatureDrive(fwd, adjustedRot, isQuickTurn);
    
        double xSpeed = MathUtil.clamp(fwd, -1.0, 1.0);
        double zRotation = MathUtil.clamp(adjustedRot, -1.0, 1.0);
    
        double leftSpeed;
        double rightSpeed;
    
        if (isQuickTurn) {
          leftSpeed = xSpeed + zRotation;
          rightSpeed = xSpeed - zRotation;
        } else {
          leftSpeed = xSpeed + Math.abs(xSpeed) * zRotation;
          rightSpeed = xSpeed - Math.abs(xSpeed) * zRotation;
        }
    
        // Normalize wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (maxMagnitude > 1.0) {
          leftSpeed /= maxMagnitude;
          rightSpeed /= maxMagnitude;
        }
        m_drivetrain.tankDriveVolts(
          DriveConstants.kMaxDrivetrainVolts * leftSpeed,
          DriveConstants.kMaxDrivetrainVolts * rightSpeed);
      },
      m_drivetrain);
    /*() -> {
            m_drivetrain.curvatureDrive
            (
              m_LeftStickYAxis.getAsDouble(), 
              m_RightStickXAxis.getAsDouble()
              );
    },
    m_drivetrain);*/

  private final Command tankDriveSame = new RunCommand(
      () -> {m_drivetrain.tankDriveVolts(
                DriveConstants.kMaxDrivetrainVolts * m_LStickYAxis.getAsDouble(),
                DriveConstants.kMaxDrivetrainVolts * m_LStickYAxis.getAsDouble());},
      m_drivetrain);

      /*private final Command testCurvatureDrive = new RunCommand(
        () -> {
                m_drivetrain.curvatureDrive
                (
                  0.1, 
                  0
                  );
        },
        m_drivetrain);*/    

  private final Command turnDrive = new RunCommand(
    () -> {m_drivetrain.tankDriveVolts(
                DriveConstants.kMaxDrivetrainVolts * m_RStickXAxis.getAsDouble(),
                -DriveConstants.kMaxDrivetrainVolts * m_RStickXAxis.getAsDouble());},
    m_drivetrain);

  private OptionalDouble lastHeadingWithVision = OptionalDouble.empty();

  public boolean targetInView() {
    return LimeLight.getEntry("tv").getNumber(0).intValue() == 1;
  }

  private final Command acquireHeadingForTarget = new RunCommand(
    () -> {
          if (targetInView()){
            double tx = LimeLight.getEntry("tx").getDouble(0);
            lastHeadingWithVision = OptionalDouble.of(m_drivetrain.getHeading()-tx);
          }
    }
  );


  private final Command aimDrivetrainAtHub = new RunCommand(
    () -> {
            double Kp = -0.015;         //-0.015 on concrete
            double min_command = 0.075; //0.075 on concrete

            LimeLight = RobotMainNetworkTableInstance.getTable("limelight");

            double forwardValue = (0.25)*applyDeadband( m_LeftStickYAxis.getAsDouble(),.1);
            double steering_adjust = 0;

            if (targetInView()){
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
                m_drivetrain.tankDriveVolts(
                  DriveConstants.kMaxDrivetrainVolts * left_command,
                  DriveConstants.kMaxDrivetrainVolts * right_command);
    },
    m_drivetrain);

    private final JoystickButton B = new JoystickButton(m_Logitech_F310,2);
    private final JoystickButton A = new JoystickButton(m_Logitech_F310,1);
    private final JoystickButton X = new JoystickButton(m_Logitech_F310,3);
    private final JoystickButton LeftStickButton1 = new JoystickButton(m_Extreme_3D_Pro_1,1);
    private final JoystickButton LeftStickButton2 = new JoystickButton(m_Extreme_3D_Pro_1,2);
    private final JoystickButton LeftStickButton8 = new JoystickButton(m_Extreme_3D_Pro_1,8);
    private final JoystickButton LeftStickButton3 = new JoystickButton(m_Extreme_3D_Pro_1,3);

    private Trajectory HubToMiddleLeftBlueCargoTrajectory = importTrajectory("paths/output/HubToMiddleLeftBlueCargo.wpilib.json");
    private Trajectory MiddleLeftBlueCargoToHubTrajectory = importTrajectory("paths/output/MiddleLeftBlueCargoToHub.wpilib.json");
    private Trajectory HubToBottomLeftBlueCargo1Trajectory = importTrajectory("paths/output/HubToBottomLeftBlueCargo1.wpilib.json");
    private Trajectory HubToBottomLeftBlueCargo2Trajectory = importTrajectory("paths/output/HubToBottomLeftBlueCargo2.wpilib.json");
    private Trajectory BottomLeftBlueCargoToHubTrajectory = importTrajectory("paths/output/BottomLeftBlueCargoToHub.wpilib.json");
    
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_drivetrain.setDefaultCommand(m_Extreme_3D_Pro_CurvatureDrive);
    m_IntakeSubsystem.setDefaultCommand(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(0);}, m_IntakeSubsystem).withName("defaultStop"));
    TurnToHeading gyroPointRobotAtHub = new TurnToHeading(m_drivetrain, () -> {return lastHeadingWithVision.orElse(0);});
    ConditionalCommand gyroPointRobotAtHubIfHubAngleKnown = new ConditionalCommand(gyroPointRobotAtHub, new InstantCommand(() -> {}), () -> {return lastHeadingWithVision.isPresent();});
    //A.whileHeld(new ConditionalCommand(aimDrivetrainAtHub, gyroPointRobotAtHubIfHubAngleKnown, () -> {return targetInView();}));
    LeftStickButton1.whileHeld(new ConditionalCommand(aimDrivetrainAtHub, gyroPointRobotAtHubIfHubAngleKnown, () -> {return targetInView();}));
    LeftStickButton2.whileHeld(m_IntakeMotorRunCommand);
    LeftStickButton8.whenActive(m_IntakeMotorLiftRunCommand);
    LeftStickButton8.whenInactive(m_IntakeMotorLiftRunCommandDisengageCommand);
    //LeftStickButton3.whileHeld(testCurvatureDrive);
    configureButtonBindings();
  }

  private Trajectory importTrajectory(String trajectoryPathString) {
    Trajectory toReturn = new Trajectory();
    try
    {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryPathString);
      toReturn = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      System.out.println("Trajectory loaded: " + trajectoryPathString);
   }
   catch (IOException ex)
   {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryPathString, ex.getStackTrace());
   }
   return toReturn;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  public Command getAcquireHeadingForTarget(){
    return acquireHeadingForTarget;
  }

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return new InstantCommand(()-> {m_drivetrain.resetOdometry(HubToMiddleLeftBlueCargoTrajectory.getInitialPose());}, m_drivetrain)
    .andThen
      (
        generateRamseteCommand(HubToMiddleLeftBlueCargoTrajectory)
        .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(1);}, m_IntakeSubsystem))
      )
    .andThen
      (
        generateRamseteCommand(MiddleLeftBlueCargoToHubTrajectory)
        .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(0);}, m_IntakeSubsystem))
        .raceWith(new RunCommand(()-> {m_IntakeLiftSubsystem.moveIntakeLiftUp(true);}, m_IntakeLiftSubsystem))
      )
    .andThen
      (
        generateRamseteCommand(HubToBottomLeftBlueCargo1Trajectory)
        .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(1);}, m_IntakeSubsystem))
        .raceWith(new RunCommand(()-> {m_IntakeLiftSubsystem.moveIntakeLiftDown(true);}, m_IntakeLiftSubsystem))
      )
    .andThen(generateRamseteCommand(HubToBottomLeftBlueCargo2Trajectory))
    .andThen
      (
        generateRamseteCommand(BottomLeftBlueCargoToHubTrajectory)
        .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(0);}, m_IntakeSubsystem))
        .raceWith(new RunCommand(()-> {m_IntakeLiftSubsystem.moveIntakeLiftUp(true);}, m_IntakeLiftSubsystem))
      )
    .andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
  }

  private Command generateRamseteCommand(Trajectory traj) {
    RamseteCommand ramseteCommand = new RamseteCommand(
        traj,
        m_drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVelMetersPerSec, 0, 0),
        new PIDController(DriveConstants.kPDriveVelMetersPerSec, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivetrain::tankDriveVolts,
        m_drivetrain
    );

    

    // Run path following command, then stop at the end.
  return ramseteCommand;
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

    //System.out.printf("%.3f%n",LimeLight.getEntry("tx").getDouble(0));
    //System.out.println(lastHeadingWithVision);

    //System.out.print("");
    //System.out.println(m_Logitech_F310.getRawAxis(4));


/*
    System.out.println(LimeLightCoPolar);
    System.out.print("");

    System.out.println(LimeLightContourArea);
    System.out.print("");
*/
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
