// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeLiftSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drivetrain = new DriveSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final IntakeLiftSubsystem m_IntakeLiftSubsystem = new IntakeLiftSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();


  private final XboxController m_Logitech_F310 = new XboxController(0);
  private final DoubleSupplier m_F310_LStickYAxis = () -> (-m_Logitech_F310.getRawAxis(1)); //Java Lambda Expression; In a Logitech F310 pushing forward yields a negative value
  private final DoubleSupplier m_F310_LStickXAxis = () -> (m_Logitech_F310.getRawAxis(0));
  private final DoubleSupplier m_F310_RStickXAxis = () -> (m_Logitech_F310.getRawAxis(4));
  private final JoystickButton B = new JoystickButton(m_Logitech_F310,2);
  private final JoystickButton A = new JoystickButton(m_Logitech_F310,1);
  private final JoystickButton X = new JoystickButton(m_Logitech_F310,3);
  private final JoystickButton Y = new JoystickButton(m_Logitech_F310,4);
  private final Command m_F310_CurvatureDrive = curvatureDrive(m_F310_LStickYAxis, m_F310_RStickXAxis, ()-> (1), m_drivetrain);


  private final Joystick m_Extreme_3D_Pro_Left = new Joystick(2);
  private final DoubleSupplier m_LeftStickYAxis = () -> (-applyDeadband(m_Extreme_3D_Pro_Left.getRawAxis(1), Constants.OIConstants.kExtreme3DProDeadband));
  private final DoubleSupplier m_LeftStickXAxis = () -> (applyDeadband(m_Extreme_3D_Pro_Left.getRawAxis(0),  Constants.OIConstants.kExtreme3DProDeadband));
  private final DoubleSupplier m_LeftStickSlider = () -> ((-m_Extreme_3D_Pro_Left.getRawAxis(3)+1)/2);
  private final DoubleSupplier m_LeftStickTwistValue = () -> (m_Extreme_3D_Pro_Left.getRawAxis(2));
  private final Button[] LeftStickButtons = createStickButtons(m_Extreme_3D_Pro_Left);

  private final Joystick m_Extreme_3D_Pro_Right = new Joystick(3);
  private final DoubleSupplier m_RightStickXAxis = () -> (applyDeadband(m_Extreme_3D_Pro_Right.getRawAxis(0),  Constants.OIConstants.kExtreme3DProDeadband));
  private final Button[] RightStickButtons = createStickButtons(m_Extreme_3D_Pro_Right);  

  private final Command m_Extreme_3D_Pro_CurvatureDrive = curvatureDrive(m_LeftStickYAxis, m_RightStickXAxis, ()-> (0.55), m_drivetrain);


  private Trajectory HubToMiddleLeftBlueCargoTrajectory = importTrajectory("paths/output/HubToMiddleLeftBlueCargo.wpilib.json");
  private Trajectory MiddleLeftBlueCargoToHubTrajectory = importTrajectory("paths/output/MiddleLeftBlueCargoToHub.wpilib.json");
  private Trajectory HubToBottomLeftBlueCargo1Trajectory = importTrajectory("paths/output/HubToBottomLeftBlueCargo1.wpilib.json");
  private Trajectory HubToBottomLeftBlueCargo2Trajectory = importTrajectory("paths/output/HubToBottomLeftBlueCargo2.wpilib.json");
  private Trajectory BottomLeftBlueCargoToHubTrajectory = importTrajectory("paths/output/BottomLeftBlueCargoToHub.wpilib.json");
  private Trajectory HubToTopLeftBlueCargoTrajectory = importTrajectory("paths/output/HubToTopLeftBlueCargo.wpilib.json");
  private Trajectory TopLeftBlueCargoToHubTrajectory = importTrajectory("paths/output/TopLeftBlueCargoToHub.wpilib.json");
  private Trajectory HubToOutOfTarmacTrajectory = importTrajectory("paths/output/HubToOutOfTarmac.wpilib.json");
  private Trajectory GoStraightTestTrajectory = importTrajectory("paths/output/GoStraight.wpilib.json");
  private Trajectory OneTurnTestTrajectory = importTrajectory("paths/output/OneTurn.wpilib.json");
  private Trajectory CosineTestTrajectory = importTrajectory("paths/output/Cosine.wpilib.json");
  private Trajectory SimpleCurveTestTrajectory = importTrajectory("paths/output/SimpleCurve.wpilib.json");
  private Trajectory GoStraightBackTestTrajectory = importTrajectory("paths/output/GoStraightBack.wpilib.json");
  private Trajectory SimpleCurve2TestTrajectory = importTrajectory("paths/output/SimpleCurve2.wpilib.json");
  private Trajectory SimpleCurve2BackTestTrajectory = importTrajectory("paths/output/SimpleCurve2Back.wpilib.json");


  private final Command m_IntakeInward = new RunCommand(
     () -> {
        m_IntakeSubsystem.setIntakeSpeed(1);
     },
  m_IntakeSubsystem);

  private final Command m_IntakeOutward = new RunCommand(
     () -> {
        m_IntakeSubsystem.setIntakeSpeed(-1);
     },
  m_IntakeSubsystem);

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<Double> m_secChooser = new SendableChooser<>();
    
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_drivetrain.setDefaultCommand(m_Extreme_3D_Pro_CurvatureDrive);
    m_IntakeSubsystem.setDefaultCommand(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(0);}, m_IntakeSubsystem).withName("defaultStop"));
    m_IntakeLiftSubsystem.setDefaultCommand(new IntakeLiftStop(m_IntakeLiftSubsystem));
    m_ClimberSubsystem.setDefaultCommand(new RunCommand(()-> {m_ClimberSubsystem.turnClimberOff();}, m_ClimberSubsystem));


    LeftStickButtons[2].whenPressed(new TurnRelativeToHeading(m_drivetrain, 45).withTimeout(1));
    RightStickButtons[2].whenPressed(new TurnRelativeToHeading(m_drivetrain, -45).withTimeout(1));

    RightStickButtons[1].whileHeld(m_IntakeInward);
    RightStickButtons[4].whileHeld(m_IntakeOutward);

    LeftStickButtons[9].whileHeld(new RunCommand(()->{m_IntakeLiftSubsystem.setArmPercent(-0.15);}, m_IntakeLiftSubsystem));
    LeftStickButtons[10].whileHeld(new RunCommand(()->{m_IntakeLiftSubsystem.setArmPercent(0.15);}, m_IntakeLiftSubsystem));
    
    LeftStickButtons[11].whenPressed(new InstantCommand(()-> {m_IntakeLiftSubsystem.resetIntakeLiftPositionToDown();}, m_IntakeLiftSubsystem));
    LeftStickButtons[12].whenPressed(new InstantCommand(()-> {m_IntakeLiftSubsystem.resetIntakeLiftPositionToUp();}, m_IntakeLiftSubsystem));
    
    LeftStickButtons[3].whenPressed(new RaiseIntakeToTop(m_IntakeLiftSubsystem, IntakeLiftConstants.kFastIntakeRaise));
    LeftStickButtons[4].whenPressed(new LowerIntakeToBottom(m_IntakeLiftSubsystem));

    LeftStickButtons[8].whenPressed(new EnableBrakeMode(m_drivetrain));
    LeftStickButtons[7].whenPressed(new EnableCoastMode(m_drivetrain));
    
    RightStickButtons[8].whileHeld(new RunCommand(()->{m_ClimberSubsystem.setClimber1Percentage(-0.25);},m_ClimberSubsystem));
    RightStickButtons[7].whileHeld(new RunCommand(()->{m_ClimberSubsystem.setClimber1Percentage(0.25);},m_ClimberSubsystem));
    
    RightStickButtons[10].whileHeld(new RunCommand(()->{m_ClimberSubsystem.setClimber2Percentage(-0.25);},m_ClimberSubsystem));
    RightStickButtons[9].whileHeld(new RunCommand(()->{m_ClimberSubsystem.setClimber2Percentage(0.25);},m_ClimberSubsystem));

    LeftStickButtons[1].whileHeld(new RunCommand(()->{m_ClimberSubsystem.setBothClimberPercentage(-1, -1);},m_ClimberSubsystem));
    RightStickButtons[11].whileHeld(new RunCommand(()->{m_ClimberSubsystem.setBothClimberPercentage(1, 1);},m_ClimberSubsystem));

    m_chooser.setDefaultOption("middleBottomCargoScore", middleBottomCargoScore());
    m_chooser.addOption("middleCargoAndScore", middleCargoAndScore());
    m_chooser.addOption("bottomCargoAndScore", bottomCargoAndScore());
    m_chooser.addOption("topCargoAndScore", topCargoAndScore());
    m_chooser.addOption("scoreAndMoveOutOfTarmac", scoreAndMoveOutOfTarmac());
    m_chooser.addOption("goStraightTest", goStraightTest());
    m_chooser.addOption("oneTurn", oneTurnTest());
    m_chooser.addOption("cosineTest", cosineTest());
    m_chooser.addOption("simpleCurve", simpleCurveTest());
    m_chooser.addOption("goStraightandBackTest", goStraightandBackTest());
    m_chooser.addOption("simpleCuve2andBackTest", simpleCurve2andBackTest());


    m_secChooser.setDefaultOption("0", 0.0);
    m_secChooser.addOption("1", 1.0);
    m_secChooser.addOption("2", 2.0);
    m_secChooser.addOption("3", 3.0);
    m_secChooser.addOption("4", 4.0);
    m_secChooser.addOption("5", 5.0);

    SmartDashboard.putData(m_chooser);
    SmartDashboard.putData(m_secChooser);

    CameraServer.startAutomaticCapture();

    SmartDashboard.putData(new resetGyro(m_drivetrain));
  }


  private static Button[] createStickButtons(Joystick joystick) {
    Button[] toReturn = new Button[joystick.getButtonCount() + 1];
    int i;
    for(i = 1; i < toReturn.length; i++)
      toReturn[i] = new JoystickButton(joystick, i);
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


   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selected = m_chooser.getSelected();
    CommandGroupBase.clearGroupedCommand(selected);

    return new WaitCommand(m_secChooser.getSelected())
    .andThen(selected); 
    }

  public Command middleBottomCargoScore(){
    return new InstantCommand(()-> {m_drivetrain.resetOdometry(HubToMiddleLeftBlueCargoTrajectory.getInitialPose());}, m_drivetrain)
    .andThen
      (
        generateRamseteCommand(HubToMiddleLeftBlueCargoTrajectory)
        .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(1);}, m_IntakeSubsystem))
        .alongWith(new LowerIntakeToBottom(m_IntakeLiftSubsystem))
      )
    .andThen
      (
        generateRamseteCommand(MiddleLeftBlueCargoToHubTrajectory)
        .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(0);}, m_IntakeSubsystem))
        .alongWith(new RaiseIntakeToTop(m_IntakeLiftSubsystem, IntakeLiftConstants.kFastIntakeRaise))
      )
    .andThen(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(-1);}, m_IntakeSubsystem).withTimeout(0.5))
    .andThen
      (
        generateRamseteCommand(HubToBottomLeftBlueCargo1Trajectory)
        .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(1);}, m_IntakeSubsystem))
        .alongWith(new LowerIntakeToBottom(m_IntakeLiftSubsystem))
      )
    .andThen(new TurnRelativeToHeading(m_drivetrain, -180).withTimeout(2))
    .andThen(generateRamseteCommand(HubToBottomLeftBlueCargo2Trajectory))
    .andThen
      (
        generateRamseteCommand(BottomLeftBlueCargoToHubTrajectory)
        .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(0);}, m_IntakeSubsystem))
        .alongWith(new RaiseIntakeToTop(m_IntakeLiftSubsystem, IntakeLiftConstants.kFastIntakeRaise))
      )
    .andThen
      (
      new RunCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain)
      .alongWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(-1);}, m_IntakeSubsystem))
      );
  }

  public Command middleCargoAndScore(){
    return new InstantCommand(()-> {m_drivetrain.resetOdometry(HubToMiddleLeftBlueCargoTrajectory.getInitialPose());}, m_drivetrain)
    .andThen
      (
        generateRamseteCommand(HubToMiddleLeftBlueCargoTrajectory)
        .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(1);}, m_IntakeSubsystem))
        .alongWith(new LowerIntakeToBottom(m_IntakeLiftSubsystem))
      )
    .andThen
      (
        generateRamseteCommand(MiddleLeftBlueCargoToHubTrajectory)
        .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(0);}, m_IntakeSubsystem))
        .alongWith(new RaiseIntakeToTop(m_IntakeLiftSubsystem, IntakeLiftConstants.kFastIntakeRaise))
      )
    .andThen
      (
      new RunCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain)
      .alongWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(-1);}, m_IntakeSubsystem))
      );
  }

  public Command bottomCargoAndScore(){
    return new InstantCommand(()-> {m_drivetrain.resetOdometry(HubToBottomLeftBlueCargo1Trajectory.getInitialPose());}, m_drivetrain)
    .andThen(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(-1);}, m_IntakeSubsystem).withTimeout(0.5))
    .andThen
      (
        generateRamseteCommand(HubToBottomLeftBlueCargo1Trajectory)
        .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(1);}, m_IntakeSubsystem))
        .alongWith(new LowerIntakeToBottom(m_IntakeLiftSubsystem))
      )
    .andThen(new TurnRelativeToHeading(m_drivetrain, -180).withTimeout(2))
    .andThen(generateRamseteCommand(HubToBottomLeftBlueCargo2Trajectory))
    .andThen
      (
        generateRamseteCommand(BottomLeftBlueCargoToHubTrajectory)
        .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(0);}, m_IntakeSubsystem))
        .alongWith(new RaiseIntakeToTop(m_IntakeLiftSubsystem, IntakeLiftConstants.kFastIntakeRaise))
      )
      .andThen
      (
      new RunCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain)
      .alongWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(-1);}, m_IntakeSubsystem))
      );
  }

  public Command topCargoAndScore(){
    return new InstantCommand(()-> {m_drivetrain.resetOdometry(HubToTopLeftBlueCargoTrajectory.getInitialPose());}, m_drivetrain)
    .andThen
    (
      generateRamseteCommand(HubToTopLeftBlueCargoTrajectory)
      .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(1);}, m_IntakeSubsystem))
      .alongWith(new LowerIntakeToBottom(m_IntakeLiftSubsystem))
    )
    .andThen(new TurnRelativeToHeading(m_drivetrain, -180).withTimeout(2))
    .andThen
    (
      generateRamseteCommand(TopLeftBlueCargoToHubTrajectory)
      .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(0);}, m_IntakeSubsystem))
      .alongWith(new RaiseIntakeToTop(m_IntakeLiftSubsystem, IntakeLiftConstants.kFastIntakeRaise))
    )
    .andThen
      (
      new RunCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain)
      .alongWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(-1);}, m_IntakeSubsystem))
      );
  }

  public Command scoreAndMoveOutOfTarmac(){
    return new InstantCommand(()-> {m_drivetrain.resetOdometry(HubToOutOfTarmacTrajectory.getInitialPose());}, m_drivetrain)
    .andThen(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(-1);}, m_IntakeSubsystem).withTimeout(0.5))
    .andThen
    (
      generateRamseteCommand(HubToOutOfTarmacTrajectory)
      .raceWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(0);}, m_IntakeSubsystem))
    )
    .andThen
      (
      new RunCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain)
      .alongWith(new RunCommand(()-> {m_IntakeSubsystem.setIntakeSpeed(0);}, m_IntakeSubsystem))
      );
  }

  public Command goStraightTest(){
    return new InstantCommand(()-> {m_drivetrain.resetOdometry(GoStraightTestTrajectory.getInitialPose());}, m_drivetrain)
    .andThen(generateRamseteCommand(GoStraightTestTrajectory));
  }

  public Command oneTurnTest(){
    return new InstantCommand(()-> {m_drivetrain.resetOdometry(OneTurnTestTrajectory.getInitialPose());}, m_drivetrain)
    .andThen(generateRamseteCommand(OneTurnTestTrajectory));
  }

  public Command cosineTest(){
    return new InstantCommand(()-> {m_drivetrain.resetOdometry(CosineTestTrajectory.getInitialPose());}, m_drivetrain)
    .andThen(generateRamseteCommand(CosineTestTrajectory));
  }

  public Command simpleCurveTest(){
    return new InstantCommand(()-> {m_drivetrain.resetOdometry(SimpleCurveTestTrajectory.getInitialPose());}, m_drivetrain)
    .andThen(generateRamseteCommand(SimpleCurveTestTrajectory));
  }

  public Command goStraightandBackTest(){
    return new InstantCommand(()-> {m_drivetrain.resetOdometry(GoStraightTestTrajectory.getInitialPose());}, m_drivetrain)
    .andThen(generateRamseteCommand(GoStraightTestTrajectory))
    .andThen(generateRamseteCommand(GoStraightBackTestTrajectory));
  }

  public Command simpleCurve2andBackTest(){
    return new InstantCommand(()-> {m_drivetrain.resetOdometry(SimpleCurve2TestTrajectory.getInitialPose());}, m_drivetrain)
    .andThen(generateRamseteCommand(SimpleCurve2TestTrajectory))
    .andThen(generateRamseteCommand(SimpleCurve2BackTestTrajectory));
  }

  private Command generateRamseteCommand(Trajectory traj) {
    RamseteController standardRamseteController = new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta);
    PIDController leftWheelsController = new PIDController(DriveConstants.kPDriveVelMetersPerSec, 0, 0);
    PIDController rightWheelsController = new PIDController(DriveConstants.kPDriveVelMetersPerSec, 0, 0);
    RamseteCommand ramseteCommand = new RamseteCommand(
        traj,
        m_drivetrain::getPose,
        standardRamseteController,
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        leftWheelsController,
        rightWheelsController,
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

  public void print(){
    SmartDashboard.putNumber("Sensitivity", m_LeftStickSlider.getAsDouble());
  }

  public static Command curvatureDrive(DoubleSupplier forwardSupplier, DoubleSupplier rotationSupplier, DoubleSupplier rotationSensitivity, DriveSubsystem driveSubsystem)
  {
    return new RunCommand(
      () -> {
        double fwd = forwardSupplier.getAsDouble();
        double rot = rotationSupplier.getAsDouble();
        double sensitivity = rotationSensitivity.getAsDouble();
        double adjustedRot = rot;
        double deadband = 0.2;
        boolean isQuickTurn =  fwd < deadband && fwd > -deadband;
        if (isQuickTurn){
          adjustedRot = rot * Math.abs(rot);
        }
    
        double xSpeed = MathUtil.clamp(fwd, -1.0, 1.0);
        double zRotation = MathUtil.clamp(adjustedRot, -1.0, 1.0);
        zRotation = zRotation * sensitivity;
    
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
        driveSubsystem.tankDriveVolts(
          DriveConstants.kMaxDrivetrainVolts * leftSpeed,
          DriveConstants.kMaxDrivetrainVolts * rightSpeed);
      },
      driveSubsystem);
  }
}
