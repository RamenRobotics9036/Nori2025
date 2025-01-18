// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AbsoluteDriveAdv;
import frc.robot.commands.AlignRobot;
import frc.robot.subsystems.SwerveSubsystem;
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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController m_driveController = new CommandXboxController(OperatorConstants.kDriverPort);

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
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(m_swerveDrive,
                                                                 () -> -MathUtil.applyDeadband(m_driveController.getLeftY(),
                                                                                               OperatorConstants.kDeadband),

                                                                 () -> -MathUtil.applyDeadband(m_driveController.getLeftX(),
                                                                                               OperatorConstants.kDeadband),

                                                                 () -> -MathUtil.applyDeadband(m_driveController.getRightX(),
                                                                                               OperatorConstants.kDeadband));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveDrive.getSwerveDrive(),
                                                                () -> m_driveController.getLeftY() * -1,
                                                                () -> m_driveController.getLeftX() * -1)
                                                            .withControllerRotationAxis(m_driveController::getRightX)
                                                            .deadband(OperatorConstants.kDeadband)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driveController::getRightX,
                                                                                             m_driveController::getRightY)
                                                           .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = m_swerveDrive.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = m_swerveDrive.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = m_swerveDrive.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(m_swerveDrive.getSwerveDrive(),
                                                                   () -> -m_driveController.getLeftY(),
                                                                   () -> -m_driveController.getLeftX())
                                                               .withControllerRotationAxis(() -> m_driveController.getRawAxis(2))
                                                               .deadband(OperatorConstants.kDeadband)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    m_driveController.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    m_driveController.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = m_swerveDrive.driveFieldOriented(driveDirectAngleSim);

  Command driveFieldOrientedAnglularVelocitySim = m_swerveDrive.driveFieldOriented(driveAngularVelocitySim);

  Command driveSetpointGenSim = m_swerveDrive.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    VisionSystem.initShuffleboad();
    m_swerveDrive.initShuffleboad();
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
    m_swerveDrive.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  
    m_driveController.start().onTrue((Commands.runOnce(m_swerveDrive::zeroGyro)));
    
    m_driveController.a().onTrue(
      new AlignRobot(m_swerveDrive)
    );
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