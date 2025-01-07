package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class VisionRotate extends Command {
    private final LimeLightSubsystem limelight;
    private final SwerveSubsystem swerve;
    private static final double ROTATION_TOLERANCE = 1.0; // Degrees
    private static final double MAX_ANGULAR_SPEED = 0.5;
    private Timer myTimer;

    public VisionRotate(LimeLightSubsystem limelight, SwerveSubsystem swerve) {
        this.limelight = limelight;
        this.swerve = swerve;
        addRequirements(limelight, swerve);
        myTimer = new Timer();
    }

    @Override
    public void initialize(){
        myTimer.reset();
        myTimer.start();
    }

    @Override
    public void execute() {
        double tx = limelight.getX();
        double rotationSpeed = calculateRotationSpeed(tx);
        swerve.drive(new Translation2d(0,0), rotationSpeed, true);
    }

    private double calculateRotationSpeed(double tx) {
        double speed = tx * 0.3; // Proportional control (NO ID, only da P)
        return -1 * (Math.min(Math.max(speed, -MAX_ANGULAR_SPEED), MAX_ANGULAR_SPEED));
    }

    @Override
    public boolean isFinished() {
        if (myTimer.get() > 2){
            return true;
        }
        return Math.abs(limelight.getX()) < ROTATION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0,0), 0, true);
    }
}


// package frc.robot.commands;
// import frc.robot.subsystems.LimeLightSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import edu.wpi.first.wpilibj2.command.Command;


// // DRIVE COMMAND SEEMS IMPORTANT

// // aimAtSpeaker seems like it is basically just the code I need (swervesubsystem) (seems like rotation)
// // driveToPose seems like the movement code

// // Mess with drive to distance command? (looks simple)
// // look at robot container tomorrow as it talks about the configureBindings where i put buttons on to a command

// public class VisionRotate extends Command {
//     @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//     private final LimeLightSubsystem m_subsystem;  

//     /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public VisionRotate(LimeLightSubsystem subsystem) {
//     m_subsystem = subsystem;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_subsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     while (Math.abs(m_subsystem.getXRadians()) > .05){
//         if (m_subsystem.getXRadians() > 0) {
//             // make robot turn one way
//         } else{
//             // make the robot turn the other way 
//         }
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }

