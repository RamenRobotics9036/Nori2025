package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.CommandConstants.AlignRobotConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.VisionSystem;

public class AlignRobot extends Command {
    private SwerveSubsystem m_swerveDrive;
    private Timer m_timer = new Timer();
    private PIDController m_rotatePIDController = new PIDController(0.2, 0, 0.01);
    private boolean m_shouldRun = false;
    private double m_initialTX = 0;
    private double m_desiredAngle = 0;

    public AlignRobot(SwerveSubsystem swerveDrive) {
        m_swerveDrive = swerveDrive;

        addRequirements(m_swerveDrive);
    }

    @Override
    public void initialize() {
        m_timer.restart();

        // Dont do anything if target not detected during INITIAL button press
        m_shouldRun = m_swerveDrive.getVisionSystem().isDetecting();
        if (!m_shouldRun) {
            System.out.println("No target detected.");
            return;
        }

        // Find the angle we want to move the robot to
        m_initialTX = m_swerveDrive.getVisionSystem().getTX();

        // NOTE: If tolerance is set on PIDController to 1 degree, that means
        // we'll often end-up near the edge of the tolerance - i.e. about 1 degree
        // off.  That means that if we RESTART a new PIDLoop and we're already just
        // a hair outside of the tolerance, it will move VERY SLOWLY towards the 
        // tolerance this second time.  To avoid this, we don't even START a PID
        // loop unless we're currently more than 2x (or 3x) tolerance away from the target.
        if (Math.abs(m_initialTX) < VisionConstants.dontRotateIfSmallDegrees) {
            m_shouldRun = false;
            System.out.println("Already pointing at target.");
            return;
        }

        m_desiredAngle = MathUtil.inputModulus(
            m_swerveDrive.getPose().getRotation().getDegrees() - m_initialTX, 
            -180,
            180
        );

        System.out.println(String.format(
            "InitialTX: %f, Desired Angle: %f, Current Robot Angle: %f",
            m_initialTX,
            m_desiredAngle,
            m_swerveDrive.getPose().getRotation().getDegrees()));

        m_rotatePIDController.reset();

        // Prime the PID loop with the current TX, so that derivative doesnt spike
        double currentTX = getCurrentTX();
        m_rotatePIDController.calculate(currentTX);

        // Enable continuous input for proper angle wrapping
        m_rotatePIDController.enableContinuousInput(-180, 180);

        // We're always aiming to get to 0 TX
        m_rotatePIDController.setSetpoint(0.0);

        // Set derivative tolerance to infinity so atSetpoint() ignores the derivative.
        // Without this, we may be near the setpoint within tolerance, but derivative
        // is still too high, so we keep moving.
        m_rotatePIDController.setTolerance(VisionConstants.allowedAngleUncertaintyDegrees, Double.POSITIVE_INFINITY);
    }

    private double getCurrentTX() {
        Pose2d actualRobotPose = m_swerveDrive.getPose();
        double actualAngle = actualRobotPose.getRotation().getDegrees();
        return MathUtil.inputModulus(actualAngle - m_desiredAngle, -180, 180);
    }

    @Override
    public void execute() {
        // Dont do anything if target not detected during INITIAL button press
        if (!m_shouldRun) {
            return;
        }

        double currentTX = getCurrentTX();

        double rotatePowerDegrees = m_rotatePIDController.calculate(currentTX);

        double clampedRotatePower = MathUtil.clamp(
            rotatePowerDegrees,
            -AlignRobotConstants.kMaxRotateRadsPerSecond,
            AlignRobotConstants.kMaxRotateRadsPerSecond);

        // System.out.println(String.format(
        //     "InitialTX: %f, CurrentTX: %f, Rotate Power: %f, Clamped Power: %f",
        //     m_initialTX,
        //     currentTX,
        //     rotatePowerDegrees,
        //     clampedRotatePower));

        // Drive is negative
        m_swerveDrive.drive(new Translation2d(), clampedRotatePower, false);
    }

    @Override
    public boolean isFinished() {
        // Dont do anything if target not detected during INITIAL button press
        if (!m_shouldRun) {
            return true;
        }

        if (m_timer.get() > AlignRobotConstants.maxTimeSeconds) {
            System.out.println("UNEXPECTED TIMEOUT TURNING!");
            //throw new RuntimeException("TIMEOUT TURNING!");
            return true;
        }
        // $TODO - Add this back
        //if (!m_swerveDrive.getVisionSystem().isDetecting()) {
        //    System.out.println("TARGET LOST!");
        //    return true;
        //}

        boolean isDone = m_rotatePIDController.atSetpoint();
        if (isDone) {
            System.out.println("Done turning!");
        }

        return isDone;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
