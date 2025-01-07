// Talk with the team to see what the goal is for vision. 
// With the current Vision Rotate command, we face the april tag perfectly. 
// With that, do we want to reset field relativity for a bit (so they can drive straight into the objective)
// until another button is pressed to set it back to the normal field relativity
// Do we also want a movement with vision? If so how would that work?

// Also I feel like instead of just doing april tags, we could "easily" do balls as well as it is a bright blue color
// so we would have to set up another pipeline in the limelight client.
// For this, we would probably also reset field relativity for a short bit, so the user can just hit forward for the ball?

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
    private static final double MAX_ANGULAR_SPEED = .75;
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
        swerve.drive(new Translation2d(0,0), rotationSpeed, true); // no translation whatsoever
    }

    private double calculateRotationSpeed(double tx) {
        double speed = tx * 0.2; // Proportional control (NO ID, only da P)
        return -1 * (Math.min(Math.max(speed, -MAX_ANGULAR_SPEED), MAX_ANGULAR_SPEED)); // this was chat gpt, what da poop is this
    }

    @Override
    public boolean isFinished() {
        if (myTimer.get() > 2){ // accounts for any weird timeout issues 
            return true;
        }
        return Math.abs(limelight.getX()) < ROTATION_TOLERANCE; // if it is within 1 degree
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0,0), 0, true);
    }
}



