package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeSystem extends SubsystemBase {
    private SparkFlex m_outtakeFollower = new SparkFlex(OuttakeConstants.sparkflexID, MotorType.kBrushless);
    private SparkMax m_outtakeLeader = new SparkMax(OuttakeConstants.sparkmaxID, MotorType.kBrushless);
    private RelativeEncoder m_relativeEncoder = m_outtakeLeader.getEncoder();
    private Timer m_timer = new Timer();

    public OuttakeSystem() {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        sparkMaxConfig.smartCurrentLimit(IntakeConstants.kStallLimit);
        sparkMaxConfig.inverted(false);
        sparkMaxConfig.smartCurrentLimit(OuttakeConstants.currentLimit);
        m_outtakeLeader.configure(sparkMaxConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);

        SparkFlexConfig sparkFlexConfig = new SparkFlexConfig();
        sparkFlexConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        sparkFlexConfig.smartCurrentLimit(IntakeConstants.kStallLimit);
        sparkFlexConfig.inverted(false);
        sparkFlexConfig.follow(OuttakeConstants.sparkmaxID);
        sparkFlexConfig.smartCurrentLimit(OuttakeConstants.currentLimit);
        m_outtakeFollower.configure(sparkFlexConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);
    }

    public void setMotorSpeeds(double speed) {
        m_outtakeLeader.set(speed);
    }

    public double getLeaderPosition() {
        return m_relativeEncoder.getPosition();
    }
}
