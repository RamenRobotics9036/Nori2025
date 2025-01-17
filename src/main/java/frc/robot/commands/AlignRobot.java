package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandConstants.AlignRobotConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignRobot extends Command {
    private SwerveSubsystem m_swerveDrive;
    private Pose2d m_targetPose;


    public AlignRobot(SwerveSubsystem swerveDrive, Pose2d targetPose) {
        m_swerveDrive = swerveDrive;
        m_targetPose = targetPose;
    }

    @Override
    public void initialize() {
        m_swerveDrive.driveToPose(m_targetPose).withTimeout(AlignRobotConstants.maxTimeSeconds).schedule();
    }
}
