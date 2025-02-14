// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimAtLimeLightV2;
import frc.robot.commands.AlignRobot;
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TestSwerveConstants;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeSpitCommand;
import frc.robot.commands.testcommands.TestTurnWheel;
import frc.robot.commands.testcommands.WheelTestContext;
import frc.robot.subsystems.IntakeArmSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AutoLogic;
import frc.robot.util.CommandAppliedController;
import frc.robot.vision.VisionSystem;
import swervelib.SwerveInputStream;

import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final CommandAppliedController m_driverController =
    new CommandAppliedController(OperatorConstants.kDriverPort);
  private final CommandAppliedController m_armController =
    new CommandAppliedController(OperatorConstants.kArmPort);

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

  private final IntakeSystem m_intakeSystem = new IntakeSystem();
  private IntakeArmSystem m_armSystem = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // $TODO - Pancake robot doesnt have an arm intake system.  If we instantiate this on the 
    // pancake robot, the code will crash on startup, preventing code deployment from
    // succeeding.  Note that this fix is TEMPORARY to unblock pancake bot code deployment!
    // A better fix would be not to throw in IntakeArmSystem, at least not on pancakebot.
    if (SwerveConstants.kJsonDirectory != "pancake") {
      m_armSystem = new IntakeArmSystem();
    }

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    m_swerveDrive.initShuffleboad();
    AutoLogic.initShuffleBoard();
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
    //this is field relative, right stick controls orientation relative to the field
    //drivebase.setDefaultCommand(m_driveFieldOrientedDirectAngle);



    // this is field relative, right stick controls rotation around z axis
    
    m_swerveDrive.setDefaultCommand(m_driveFieldOrientedAngularVelocity);
    m_intakeSystem.setDefaultCommand(new IntakeDefaultCommand(m_intakeSystem));
    if (m_armSystem != null) {
      m_armSystem.setDefaultCommand(new ArmDefaultCommand(m_armSystem, () -> m_armController.getLeftY()));
    }
  
    //D-pad drives straight (no gyro) for tests
    /*
    m_driverController.povUp().onTrue((m_swerveDrive.driveCommand(() -> 0.3, () -> 0, () -> 0, false)));
    m_driverController.povDown().onTrue((m_swerveDrive.driveCommand(() -> -0.3, () -> 0, () -> 0, false)));
    m_driverController.povLeft().onTrue((m_swerveDrive.driveCommand(() -> 0, () -> 0.3, () -> 0, false)));
    m_driverController.povRight().onTrue((m_swerveDrive.driveCommand(() -> 0, () -> -0.3, () -> 0, false)));
    */
    // Start button resets the gyro
    m_driverController.start().onTrue((Commands.runOnce(m_swerveDrive::zeroGyro)));

    // A button aligns the robot using the AprilTag
    //m_driverController.a().onTrue(new AimAtLimeLightV2(m_swerveDrive));
    m_driverController.a().onTrue(m_swerveDrive.alignWithAprilTagCommand());

    // Command to spit out game pieces
    m_armController.a().onTrue(new IntakeSpitCommand(m_intakeSystem));

    // Test mode has (b) button triggering a test sequence
    if (TestSwerveConstants.kIsTestMode) {
      m_driverController.b().onTrue(createTestWheelsCommand());
    }
  }

  private Command testSequence() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> m_swerveDrive.getWheelTestContext().reset()),
        new TestTurnWheel(m_swerveDrive, "frontleft"),
        new TestTurnWheel(m_swerveDrive, "frontright"),
        new TestTurnWheel(m_swerveDrive, "backleft"),
        new TestTurnWheel(m_swerveDrive, "backright")
    );
  }

  private Command createTestWheelsCommand() {
    WheelTestContext wheelTestContext = m_swerveDrive.getWheelTestContext();

    Command scheduledComand = new ConditionalCommand(
        testSequence(),
        new InstantCommand(),
        () -> DriverStation.isTest());

    // Save the sequence handle into subsystem, in case we later need to cancel it.
    wheelTestContext.cancellableCommand = scheduledComand;

    return scheduledComand;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return AutoLogic.getAutoCommand(AutoLogic.autoPicker.getSelected());
  }

  public void setMotorBrake(boolean brake)
  {
    m_swerveDrive.setMotorBrake(brake);
  }

  public void updateVisionPose() {
    m_swerveDrive.updateVisionPose();
  }
}