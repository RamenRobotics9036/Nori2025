// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoNameConstants;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.IntakeSpitCommandConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OuttakeSpitCommandConstants;
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.commands.ControllerRumbleCommand;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.commands.DriveForwardNow;
import frc.robot.commands.ElevatorDefaultCommand;
import frc.robot.commands.ElevatorManualCommand;
import frc.robot.commands.ElevatorToPositionCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.CommandConstants.AlignRobotConstants;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeSpitCommand;
import frc.robot.commands.OuttakeSpitCommand;
import frc.robot.commands.SetArmToAngleCommand;
import frc.robot.sim.SimConstants.SimCommandConstants;
import frc.robot.sim.simcommands.pretend.PretendCommandElevatorSystem;
import frc.robot.sim.simcommands.pretend.PretendCommandIntakeArmSystem;
import frc.robot.sim.simcommands.pretend.PretendCommandIntakeSystem;
import frc.robot.sim.simcommands.pretend.PretendCommandNoSystem;
import frc.robot.sim.simcommands.pretend.PretendCommandOuttakeSystem;
import frc.robot.sim.simcommands.pretend.UnexpectedCommand;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.IntakeArmSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.OuttakeSystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AutoLogic;
import frc.robot.util.CommandAppliedController;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private IntakeArmSystem m_armSystem = new IntakeArmSystem();
  private final OuttakeSystem m_outtakeSystem = new OuttakeSystem();
  private final ElevatorSystem m_elevatorSystem = new ElevatorSystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // $TODO - Pancake robot doesnt have an arm intake system.  If we instantiate this on the 
    // pancake robot, the code will crash on startup, preventing code deployment from
    // succeeding.  Note that this fix is TEMPORARY to unblock pancake bot code deployment!
    // A better fix would be not to throw in IntakeArmSystem, at least not on pancakebot.

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    m_swerveDrive.initShuffleboard();
    AutoLogic.initShuffleBoard();
    addTestButtonsToShuffleboard();

    NamedCommands.registerCommand("Set Arm Position To Bottom", CmdWrapperIntakeArmSystem(new SetArmToAngleCommand(m_armSystem, ArmConstants.kMaxArmRotation)));
    NamedCommands.registerCommand("Set Arm Position To Top", CmdWrapperIntakeArmSystem(new SetArmToAngleCommand(m_armSystem, ArmConstants.kMinArmRotation)));
    NamedCommands.registerCommand("Set Arm Position To L1", CmdWrapperIntakeArmSystem(new SetArmToAngleCommand(m_armSystem, ArmConstants.L1ArmAngle)));

    NamedCommands.registerCommand("Dispense Intake Into Bucket", CmdWrapperIntakeSystem(new IntakeSpitCommand(m_intakeSystem, -IntakeSpitCommandConstants.bucketSpeed, true)));
    NamedCommands.registerCommand("Shoot From Intake", CmdWrapperIntakeSystem(new IntakeSpitCommand(m_intakeSystem, IntakeSpitCommandConstants.speed, true)));
    NamedCommands.registerCommand("Idle Intake", CmdWrapperIntakeSystem(new IntakeDefaultCommand(m_intakeSystem).withTimeout(1)));

    NamedCommands.registerCommand("Outtake from Bucket", CmdWrapperOuttakeSystem(new OuttakeSpitCommand(m_outtakeSystem, OuttakeSpitCommandConstants.speed)));

    NamedCommands.registerCommand("Align to April Tag Left Side", CmdWrapperUnexpectedCommand(m_swerveDrive.alignWithAprilTagCommand(
      AlignRobotConstants.transformDrive,
      AlignRobotConstants.transformLeftStrafe
    ), "alignAprilLeft"));
    NamedCommands.registerCommand("Align to April Tag Right Side", CmdWrapperUnexpectedCommand(m_swerveDrive.alignWithAprilTagCommand(
      AlignRobotConstants.transformDrive,
      AlignRobotConstants.transformRightStrafe
    ), "alignAprilRight"));

    NamedCommands.registerCommand("Set Elevator Position To Bottom", CmdWrapperElevatorSystem(new ElevatorToPositionCommand(m_elevatorSystem, ElevatorConstants.kDownElevatorPosition)));
    NamedCommands.registerCommand("Set Elevator Position To L2", CmdWrapperElevatorSystem(new ElevatorToPositionCommand(m_elevatorSystem, ElevatorConstants.kLevel2ReefPosition)));
    NamedCommands.registerCommand("Set Elevator Position To L3", CmdWrapperElevatorSystem(new ElevatorToPositionCommand(m_elevatorSystem, ElevatorConstants.kLevel3ReefPosition)));
    NamedCommands.registerCommand("Set Elevator Position to L4", CmdWrapperElevatorSystem(new ElevatorToPositionCommand(m_elevatorSystem, ElevatorConstants.kLevel4ReefPosition)));

    //
    // Heres the Commands that we dont mock in simulation, since they work just fine in sim.
    //[]\


    NamedCommands.registerCommand("waitFiveSeconds", waitFiveSeconds);
  }

  private Command printStartStop(Command command, String name) {
    return command.beforeStarting(() -> System.out.println(">--> " + name + " start"))
                  .andThen(() -> System.out.println(">--> " + name + " end"));
  }

  private void addTestButtonsToShuffleboard() {
    Command spinCmd = printStartStop(
      m_swerveDrive.sysIdDriveMotorCommand(true), "Spin test");

    ShuffleboardTab tabTest = Shuffleboard.getTab("Test");
    tabTest.add("Spin Test", spinCmd).withWidget("Command");

    Command angleTest = printStartStop(
      m_swerveDrive.sysIdAngleMotorCommand(), "Angle test");

    tabTest.add("Angle Test", angleTest).withWidget("Command");
  }

  private Command CmdWrapperIntakeArmSystem(Command command) {
    // Now that we implemented sim for arm, re-enable these commands for sim.
    return command;
  }

  private Command CmdWrapperIntakeSystem(Command command) {
    if (disableCommandsInSim()) {
      return new PretendCommandIntakeSystem(m_intakeSystem);
    } else {
      return command;
    }
  }

  private Command CmdWrapperOuttakeSystem(Command command) {
    if (disableCommandsInSim()) {
      return new PretendCommandOuttakeSystem(m_outtakeSystem);
    } else {
      return command;
    }
  }

  private Command CmdWrapperElevatorSystem(Command command) {
    if (disableCommandsInSim()) {
      return new PretendCommandElevatorSystem(m_elevatorSystem);
    } else {
      return command;
    }
  }

  private Command CmdWrapperNoSystem(Command command) {
    if (disableCommandsInSim()) {
      return new PretendCommandNoSystem();
    } else {
      return command;
    }
  }

  private Command CmdWrapperUnexpectedCommand(Command command, String name) {
    if (disableCommandsInSim()) {
      return new UnexpectedCommand(name);
    } else {
      return command;
    }
  }

  // In simulation, not all Subsystems are simulated yet.  Therefore, in simulation,
  // we disable most registerCommand() that pathplanner calls, so that those commands
  // don't do anything. That lets us run the simulation for auto for the subsystems
  // that ARE currently simulated, such as Swerve.
  private boolean disableCommandsInSim() {
    if (!RobotBase.isSimulation()) {
      return false;
    }

    return SimCommandConstants.kDisableMostCommandsInSim;
  }

  public void configureDefaultCommands() {
    m_swerveDrive.setDefaultCommand(m_driveFieldOrientedAngularVelocity);
    m_intakeSystem.setDefaultCommand(new IntakeDefaultCommand(m_intakeSystem));
    
    // m_elevatorSystem.setDefaultCommand(new ElevatorManualCommand(m_elevatorSystem, () -> m_armController.getRightY()));

    m_armSystem.setDefaultCommand(new ArmDefaultCommand(m_armSystem, () -> m_armController.getLeftY()));
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
    configureDefaultCommands();
    if (m_armSystem != null) {
      // Arm up
      m_armController.rightTrigger().onTrue(new SetArmToAngleCommand(m_armSystem, ArmConstants.kMinArmRotation).
           alongWith(new ElevatorToPositionCommand(m_elevatorSystem, ElevatorConstants.kDownElevatorPosition)));
      // Algae preset
      m_armController.leftBumper().whileTrue(new SetArmToAngleCommand(m_armSystem, ArmConstants.algaePreset).
      andThen(new IntakeSpitCommand(m_intakeSystem, IntakeSpitCommandConstants.speed, true)));
      // Arm down
      m_armController.leftTrigger().whileTrue(new SetArmToAngleCommand(m_armSystem, ArmConstants.kMaxArmRotation));
    }
  
  
    //D-pad drives straight (no gyro) for tests
    /*
    m_driverController.povUp().onTrue((m_swerveDrive.driveCommand(() -> 0.3, () -> 0, () -> 0, false)));
    m_driverController.povDown().onTrue((m_swerveDrive.driveCommand(() -> -0.3, () -> 0, () -> 0, false)));
    m_driverController.povLeft().onTrue((m_swerveDrive.driveCommand(() -> 0, () -> 0.3, () -> 0, false)));
    m_driverController.povRight().onTrue((m_swerveDrive.driveCommand(() -> 0, () -> -0.3, () -> 0, false)));
    */

    // $TODO m_swerveDrive.sysIdDriveMotorCommand()

    // Start button resets the gyro
    m_driverController.start().onTrue((Commands.runOnce(m_swerveDrive::zeroGyroWithAlliance)));

    // A button aligns the robot using the AprilTag
    //m_driverController.a().onTrue(new AimAtLimeLightV2(m_swerveDrive));
    m_driverController.povLeft().onTrue(m_swerveDrive.alignWithAprilTagCommand(
    AlignRobotConstants.transformDrive,
    AlignRobotConstants.transformLeftStrafe
  ));
    m_driverController.povRight().onTrue(m_swerveDrive.alignWithAprilTagCommand(
      AlignRobotConstants.transformDrive,
      AlignRobotConstants.transformRightStrafe
  ));


    // Command to spit out game pieces
    m_armController.a().whileTrue(new IntakeSpitCommand(m_intakeSystem, IntakeSpitCommandConstants.speed));
    // Spit coral into outtake
    m_armController.b().whileTrue(new IntakeSpitCommand(m_intakeSystem, -IntakeSpitCommandConstants.bucketSpeed));
    // L2 preset
    m_armController.x().onTrue(new ElevatorToPositionCommand(m_elevatorSystem, ElevatorConstants.kLevel2ReefPosition));
    // L3 Preset
    m_armController.y().onTrue(new ElevatorToPositionCommand(m_elevatorSystem, ElevatorConstants.kLevel3ReefPosition));
    // L4 Preset
    m_armController.povUp().onTrue(new ElevatorToPositionCommand(m_elevatorSystem, ElevatorConstants.kLevel4ReefPosition));

    // Elevator down
    m_armController.povDown().onTrue(new ElevatorToPositionCommand(m_elevatorSystem, ElevatorConstants.kDownElevatorPosition));
    // Outtake reverse
    m_armController.povLeft().whileTrue(new OuttakeSpitCommand(m_outtakeSystem, -OuttakeSpitCommandConstants.speed));
    // Outtake coral
    m_armController.povRight().whileTrue(new OuttakeSpitCommand(m_outtakeSystem, OuttakeSpitCommandConstants.speed));
    
    // 
    m_armController.start().whileTrue(new OuttakeSpitCommand(m_outtakeSystem, -OuttakeSpitCommandConstants.speed));

    new Trigger(() -> (m_driverController.leftBumper().getAsBoolean() && m_swerveDrive.getVisionSystem().isDetecting())).onTrue(
      Commands.runOnce(() -> m_swerveDrive.trueResetPose())
    );

    new Trigger(() -> m_intakeSystem.isHoldingCoral()).onTrue(new ControllerRumbleCommand(m_armController, OperatorConstants.kRumbleTime)
    .alongWith(new ControllerRumbleCommand(m_driverController, OperatorConstants.kRumbleTime))
    );

  }

  private Command waitFiveSeconds = new WaitCommand(5)
    .beforeStarting(() -> System.out.println("Auto wait 5 seconds start"))
    .andThen(() -> System.out.println("Auto end wait"));

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    switch (AutoLogic.autoPicker.getSelected()) {
      case AutoNameConstants.kCenterL1AutoName:
        return new DriveForwardNow(m_swerveDrive, 1.4, true)
        .andThen(CmdWrapperIntakeArmSystem(new SetArmToAngleCommand(m_armSystem, ArmConstants.L1ArmAngle)))
        .andThen(CmdWrapperIntakeSystem(new IntakeSpitCommand(m_intakeSystem, IntakeSpitCommandConstants.speed, true)));

      case AutoNameConstants.kCenterL4AutoName:
          return new DriveForwardNow(m_swerveDrive, 1, false)
          .andThen(CmdWrapperElevatorSystem(new ElevatorToPositionCommand(m_elevatorSystem, ElevatorConstants.kMaxElevatorPosition).withTimeout(2)))
          .andThen(new DriveForwardNow(m_swerveDrive, 0.5, false))
          .andThen(CmdWrapperOuttakeSystem(new OuttakeSpitCommand(m_outtakeSystem, OuttakeSpitCommandConstants.speed, true)));
      
      default:
        return new DriveForwardNow(m_swerveDrive, 1.4, true).
          andThen(CmdWrapperIntakeArmSystem(new SetArmToAngleCommand(m_armSystem, ArmConstants.L1ArmAngle)))
          .andThen(CmdWrapperIntakeSystem(new IntakeSpitCommand(m_intakeSystem, IntakeSpitCommandConstants.speed, true)));
  }
    // return AutoLogic.getAutoCommand(AutoLogic.autoPicker.getSelected());
  }

  public void setMotorBrake(boolean brake)
  {
    m_swerveDrive.setMotorBrake(brake);
  }

  public void updateVisionPose() {
    m_swerveDrive.updateVisionPose();
  }
}