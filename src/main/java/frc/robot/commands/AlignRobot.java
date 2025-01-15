package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandConstants.AlignRobotConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSystem;

public class AlignRobot extends Command {
    private Timer m_timer = new Timer();
    private SwerveSubsystem m_swerveDrive;

    public AlignRobot(SwerveSubsystem swerveDrive) {
        m_swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {
        m_timer.restart();
        if (VisionSystem.isDetecting()) {
            Pose2d targetPose = VisionSystem.getTargetPose();
            Transform2d transform = new Transform2d(
                AlignRobotConstants.transformX,
                AlignRobotConstants.transformY,
                Rotation2d.fromDegrees(AlignRobotConstants.transformRot)
            );
            targetPose = targetPose.plus(transform);
            m_swerveDrive.driveToPose(targetPose).schedule();
        }
    }
}
