package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSystem extends SubsystemBase{
    private SparkMax m_pullMotor = new SparkMax(IntakeConstants.pullMotorID, MotorType.kBrushless);
    private SparkMax m_loadMotor = new SparkMax(IntakeConstants.loadMotorID, MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();

    public IntakeSystem(){
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_pullMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        m_pullMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }
}
