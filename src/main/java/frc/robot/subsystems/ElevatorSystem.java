package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ElevatorContants;
import frc.robot.Constants.IntakeConstants;

public class ElevatorSystem {
    //Motors are on opposate sides of a shaft
    private final SparkMax m_leaderMotor = new SparkMax(ElevatorContants.kLeaderMotorID, MotorType.kBrushless);
    private final SparkMax m_followMotor = new SparkMax(ElevatorContants.kFollowMotorID, MotorType.kBrushless);
    private SparkMaxConfig m_leaderConfig = new SparkMaxConfig();
    private SparkMaxConfig m_followConfig = new SparkMaxConfig();

    private DigitalInput m_limitSwitch= new DigitalInput(ElevatorContants.DIOIndex);
    /* Sensor will reset a relative encoder when the elevator lowers fully
     * Encoder will be used to prevent arm from going too high*/

    private double maxOutput = ElevatorContants.kMaxOutputPercentage;
    
    public ElevatorSystem() {
        m_leaderConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_leaderConfig.smartCurrentLimit(IntakeConstants.kStallLimit);
        m_leaderConfig.inverted(false);
        m_leaderMotor.configure(m_leaderConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);

        m_followConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_followConfig.smartCurrentLimit(IntakeConstants.kStallLimit);
        m_followConfig.inverted(true);
        m_followConfig.follow(ElevatorContants.kLeaderMotorID);
        m_followMotor.configure(m_followConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);
    }

    public void setSpeed(double speed){
        m_leaderMotor.set(MathUtil.clamp(speed, -maxOutput, maxOutput));
    }

    public double getSpeed(){
        return m_leaderMotor.get();
    }
}
