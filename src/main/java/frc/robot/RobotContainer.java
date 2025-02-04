// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeTestCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CommandAppliedController;

import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

/*
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AlignRobot;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.CommandAppliedController;
import frc.robot.vision.VisionSystem;

import java.io.File;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  private final CommandAppliedController m_driverController =
      new CommandAppliedController(OperatorConstants.kDriverPort);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       m_swerveDrive  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                SwerveConstants.kJsonDirectory));
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
   /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveDrive.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY(),
                                                                () -> m_driverController.getLeftX())
                                                            .withControllerRotationAxis(() -> -m_driverController.getRightX())
                                                            .deadband(OperatorConstants.kDeadband)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                           .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command m_driveFieldOrientedDirectAngle = m_swerveDrive.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command m_driveFieldOrientedAngularVelocity = m_swerveDrive.driveFieldOriented(driveAngularVelocity);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    /*
        m_driverController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
        m_driverController.b().whileTrue(
            Commands.deferredProxy(() -> drivebase.driveToPose(
                                        new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                                  ));
        m_driverController.y().whileTrue(drivebase.aimAtSpeaker(2));
        m_driverController.start().whileTrue(Commands.none());
        m_driverController.back().whileTrue(Commands.none());
        m_driverController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        m_driverController.rightBumper().onTrue(Commands.none());
    */
    /*
    //d-pad and X, B test controls:
    m_driverController.povUp().onTrue((drivebase.driveCommand(() -> 0.3, () -> 0, () -> 0, false)));
    m_driverController.povDown().onTrue((drivebase.driveCommand(() -> -0.3, () -> 0, () -> 0, false)));
    m_driverController.povLeft().onTrue((drivebase.driveCommand(() -> 0, () -> 0.3, () -> 0, false)));
    m_driverController.povRight().onTrue((drivebase.driveCommand(() -> 0, () -> -0.3, () -> 0, false)));
    m_driverController.x().onTrue(drivebase.driveCommand(() -> 0, () -> 0, () -> 0.5, false));
    m_driverController.b().onTrue(drivebase.driveCommand(() -> 0, () -> 0, () -> -0.5, false));
    */
    //this is field relative, right stick controls orientation relative to the field
    //drivebase.setDefaultCommand(m_driveFieldOrientedDirectAngle);


//this is field relative, right stick controls rotation around z axis
  drivebase.setDefaultCommand(m_driveFieldOrientedAngularVelocity);
  m_intakeSystem.setDefaultCommand(new IntakeTestCommand(m_intakeSystem));
drivebase.setDefaultCommand(m_driveFieldOrientedAngularVelocity);

    // this is field relative, right stick controls rotation around z axis
    m_swerveDrive.setDefaultCommand(m_driveFieldOrientedAngularVelocity);
  
    //D-pad drives straight (no gyro) for tests
    m_driverController.povUp().onTrue((m_swerveDrive.driveCommand(() -> 0.3, () -> 0, () -> 0, false)));
    m_driverController.povDown().onTrue((m_swerveDrive.driveCommand(() -> -0.3, () -> 0, () -> 0, false)));
    m_driverController.povLeft().onTrue((m_swerveDrive.driveCommand(() -> 0, () -> 0.3, () -> 0, false)));
    m_driverController.povRight().onTrue((m_swerveDrive.driveCommand(() -> 0, () -> -0.3, () -> 0, false)));
  
    // Start button resets the gyro
    m_driverController.start().onTrue((Commands.runOnce(m_swerveDrive::zeroGyro)));
    
    // A button aligns the robot using the AprilTag
    m_driverController.a().onTrue(new AlignRobot(m_swerveDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return new InstantCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    m_swerveDrive.setMotorBrake(brake);
  }

  public void updateVisionPose() {
    VisionSystem.updatePose();
  }
}