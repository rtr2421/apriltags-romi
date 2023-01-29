// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.FollowTrajectory;
import frc.robot.sensors.Camera;
import frc.robot.sensors.FieldPosition;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Camera camera = new Camera();
  private final FieldPosition fieldPosition = new FieldPosition(camera.getPhotonCamera());
  private final DriveTrain driveTrain = new DriveTrain(camera, fieldPosition);
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  // Assumes a gamepad plugged into channnel 0
  private final Joystick m_controller = new Joystick(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    driveTrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .onTrue(new PrintCommand("Button A Pressed"))
        .onFalse(new PrintCommand("Button A Released"));

    // Setup SmartDashboard options
    /* 
    m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain)); */
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return m_chooser.getSelected();
           // Create a voltage constraint to ensure we don't accelerate too fast
           var autoVoltageConstraint =
           new DifferentialDriveVoltageConstraint(
               new SimpleMotorFeedforward(
                   DriveConstants.ksVolts,
                   DriveConstants.kvVoltSecondsPerMeter,
                   DriveConstants.kaVoltSecondsSquaredPerMeter),
               DriveConstants.kDriveKinematics,
               10);
   
       // Create config for trajectory
       TrajectoryConfig config =
           new TrajectoryConfig(DriveConstants.maxVelocityMetersPerSecond, 0.3)
               // Add kinematics to ensure max speed is actually obeyed
               .setKinematics(DriveConstants.kDriveKinematics)
               // Apply the voltage constraint
               .addConstraint(autoVoltageConstraint);
   
       Trajectory straight =
           TrajectoryGenerator.generateTrajectory(
               new Pose2d(0, 0, new Rotation2d(0)),
               List.of(new Translation2d(0.5, 0)),
               new Pose2d(1.5, 0, new Rotation2d(0)),
               // Pass config
               config);
       Trajectory goThere =
           TrajectoryGenerator.generateTrajectory(
               // Start at the origin facing the +X direction
               new Pose2d(0, 0, new Rotation2d(0)),
               // Pass through these two interior waypoints, making an 's' curve path
               List.of(new Translation2d(0.5, -0.5), new Translation2d(1, 0.5)),
               //List.of(new Translation2d(0.2, 0.0)),
               // End 3 meters straight ahead of where we started, facing forward
               //new Pose2d(3, 0, new Rotation2d(0)),
               new Pose2d(1.5, 0, new Rotation2d(0)),
               // Pass config
               config);
       Trajectory comeBack =
           TrajectoryGenerator.generateTrajectory(
               // Start at the origin facing the +X direction
               new Pose2d(0, 0, new Rotation2d(0)),
               // Pass through these two interior waypoints, making an 's' curve path
               List.of(new Translation2d(1, -0.5), new Translation2d(2, 0.5)),
               //List.of(new Translation2d(0.2, 0.0)),
               // End 3 meters straight ahead of where we started, facing forward
               //new Pose2d(3, 0, new Rotation2d(0)),
               new Pose2d(3, 0, new Rotation2d(0)),
               // Pass config
               config.setReversed(true));
   
       // Reset odometry to the starting pose of the trajectory.
       return new InstantCommand(() -> fieldPosition.setRobotPose(straight.getInitialPose())).
         andThen(new FollowTrajectory(driveTrain, fieldPosition, straight)).
       // andThen(new FollowTrajectory(driveTrain, comeBack)).
         andThen(new PrintCommand("Done!!!")).
         andThen(() -> driveTrain.tankDriveVolts(0, 0));
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        driveTrain, () -> -m_controller.getRawAxis(1), () -> -m_controller.getRawAxis(2));
  }
}
