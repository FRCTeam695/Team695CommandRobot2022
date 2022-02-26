// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeLiftSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import frc.robot.Constants.*;
import frc.robot.commands.*;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;
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
  private final Joystick m_Extreme_3D_Pro_Left = new Joystick(2);
  private final Joystick m_Extreme_3D_Pro_Right = new Joystick(3);
  private final DoubleSupplier m_F310_LStickYAxis = () -> (-m_Logitech_F310.getY()); //Java Lambda Expression; In a Logitech F310 pushing forward yields a negative value
  private final DoubleSupplier m_F310_LStickXAxis = () -> (m_Logitech_F310.getX());
  private final DoubleSupplier m_F310_RStickXAxis = () -> (m_Logitech_F310.getRawAxis(4));

  private final DoubleSupplier m_LeftStickYAxis = () -> (-applyDeadband(m_Extreme_3D_Pro_Left.getRawAxis(1), Constants.OIConstants.kExtreme3DProDeadband));
  private final DoubleSupplier m_LeftStickXAxis = () -> (applyDeadband(m_Extreme_3D_Pro_Left.getRawAxis(0),  Constants.OIConstants.kExtreme3DProDeadband));
  private final DoubleSupplier m_RightStickXAxis = () -> (applyDeadband(m_Extreme_3D_Pro_Right.getRawAxis(0),  Constants.OIConstants.kExtreme3DProDeadband));
  private final DoubleSupplier m_LeftStickSlider = () -> (m_Extreme_3D_Pro_Left.getRawAxis(3));

  private final DoubleSupplier m_LeftStickTwistValue = () -> (m_Extreme_3D_Pro_Left.getRawAxis(2));

  private final NetworkTableInstance RobotMainNetworkTableInstance = NetworkTableInstance.getDefault();
  private final DriveSubsystem m_drivetrain = new DriveSubsystem();
  private final Limelight m_LimelightSubsystem = new Limelight(RobotMainNetworkTableInstance, 0);

  private final NetworkTable LimeLight = RobotMainNetworkTableInstance.getTable("limelight");

  private final Command m_IntakeInward = new RunCommand(
     () -> {
        m_IntakeSubsystem.setIntakeSpeed(1);
        System.out.println(m_LeftStickTwistValue.getAsDouble());
     },
  m_IntakeSubsystem);

  private final Command m_IntakeOutward = new RunCommand(
     () -> {
        m_IntakeSubsystem.setIntakeSpeed(-1);
        System.out.println(m_LeftStickTwistValue.getAsDouble());
     },
  m_IntakeSubsystem);

  private final Command m_Extreme_3D_Pro_CurvatureDrive = curvatureDrive(m_LeftStickYAxis, m_RightStickXAxis, m_drivetrain);
  private final Command m_F310_CurvatureDrive = curvatureDrive(m_F310_LStickYAxis, m_F310_RStickXAxis, m_drivetrain);

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

    private final Button[] LeftStickButtons = createStickButtons(m_Extreme_3D_Pro_Left, 12);
    private final Button[] RightStickButtons = createStickButtons(m_Extreme_3D_Pro_Right, 12);

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
    m_IntakeLiftSubsystem.setDefaultCommand(new IntakeLiftStop(m_IntakeLiftSubsystem));
    TurnToHeading gyroPointRobotAtHub = new TurnToHeading(m_drivetrain, () -> {return lastHeadingWithVision.orElse(0);});
    ConditionalCommand gyroPointRobotAtHubIfHubAngleKnown = new ConditionalCommand(gyroPointRobotAtHub, new InstantCommand(() -> {}), () -> {return lastHeadingWithVision.isPresent();});
    LeftStickButtons[1].whileHeld(new ConditionalCommand(aimDrivetrainAtHub, gyroPointRobotAtHubIfHubAngleKnown, () -> {return targetInView();}));
    LeftStickButtons[2].whileHeld(m_IntakeInward);
    RightStickButtons[1].whileHeld(m_IntakeInward);
    RightStickButtons[4].whileHeld(m_IntakeOutward);
    LeftStickButtons[3].whenPressed(new RaiseIntake(m_IntakeLiftSubsystem));
    LeftStickButtons[4].whenPressed(new LowerIntake(m_IntakeLiftSubsystem));
    LeftStickButtons[8].whenPressed(new EnableBrakeMode(m_drivetrain));
    LeftStickButtons[7].whenPressed(new EnableCoastMode(m_drivetrain));
  }

  private static Button[] createStickButtons(Joystick joystick, int buttonCount) {
    Button[] toReturn = new Button[buttonCount + 1];
    int i;
    for(i = 1; i < joystick.getButtonCount() + 1; i++)
      toReturn[i] = new JoystickButton(joystick, i);

    for(; i < toReturn.length; i++)
      toReturn[i] = new Button();
      return toReturn;
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
        .alongWith(new LowerIntake(m_IntakeLiftSubsystem))
      )
    .andThen
      (
        generateRamseteCommand(MiddleLeftBlueCargoToHubTrajectory)
        .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(0);}, m_IntakeSubsystem))
        .alongWith(new RaiseIntake(m_IntakeLiftSubsystem))
      )
    .andThen
      (
        generateRamseteCommand(HubToBottomLeftBlueCargo1Trajectory)
        .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(1);}, m_IntakeSubsystem))
        .alongWith(new LowerIntake(m_IntakeLiftSubsystem))
      )
    .andThen(generateRamseteCommand(HubToBottomLeftBlueCargo2Trajectory))
    .andThen
      (
        generateRamseteCommand(BottomLeftBlueCargoToHubTrajectory)
        .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(0);}, m_IntakeSubsystem))
        .alongWith(new RaiseIntake(m_IntakeLiftSubsystem))
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

  public static Command curvatureDrive(DoubleSupplier forwardSupplier, DoubleSupplier rotationSupplier, DriveSubsystem driveSubsystem)
  {
    return new RunCommand(
      () -> {
        double fwd = forwardSupplier.getAsDouble();
        double rot = rotationSupplier.getAsDouble();
        double adjustedRot = rot;
        double deadband = 0.2;
        boolean isQuickTurn =  fwd < deadband && fwd > -deadband;
        if (isQuickTurn){
          adjustedRot = rot * Math.abs(rot);
        }
    
        double xSpeed = MathUtil.clamp(fwd, -1.0, 1.0);
        double zRotation = MathUtil.clamp(adjustedRot, -1.0, 1.0);
    
        double leftSpeed;
        double rightSpeed;
    
        if (isQuickTurn) {
          leftSpeed = xSpeed + Math.pow(zRotation, 3);
          rightSpeed = xSpeed - Math.pow(zRotation, 3);
        } else {
          leftSpeed = xSpeed + Math.abs(xSpeed) * Math.pow(zRotation, 3);
          rightSpeed = xSpeed - Math.abs(xSpeed) * Math.pow(zRotation, 3);
        }
    
        // Normalize wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (maxMagnitude > 1.0) {
          leftSpeed /= maxMagnitude;
          rightSpeed /= maxMagnitude;
        }
        driveSubsystem.tankDriveVolts(
          DriveConstants.kMaxDrivetrainVolts * leftSpeed,
          DriveConstants.kMaxDrivetrainVolts * rightSpeed);
      },
      driveSubsystem);
  }

}
