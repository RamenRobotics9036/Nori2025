package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ElevatorContants;
import frc.robot.Constants.IntakeConstants;

public class ElevatorSystem {
    //Motors are on opposate sides of a shaft
    //IDS ARE FILLER!!!!! DON'T USE UNTIL THEY ARE CHANGED!!!!!!
    public final SparkMax m_leaderMotor = new SparkMax(ElevatorContants.kLeaderMotorID, MotorType.kBrushless);
    public final SparkMax m_followMotor = new SparkMax(ElevatorContants.kFollowMotorID, MotorType.kBrushless);
    private SparkMaxConfig m_leaderConfig = new SparkMaxConfig();
    private SparkMaxConfig m_followConfig = new SparkMaxConfig();

    /*add a sensor at some point. 
     * Sensor will reset a relative encoder when the elevator lowers fully
     * Encoder will be used to prevent arm from going too high
    */

    private double maxOutput = ElevatorContants.kMaxOutputPercentage;
    
    public ElevatorSystem() {
        m_leaderConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_leaderConfig.smartCurrentLimit(IntakeConstants.kStallLimit);
        m_leaderConfig.inverted(false);

        m_followConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_followConfig.smartCurrentLimit(IntakeConstants.kStallLimit);
        m_followConfig.inverted(true);
        m_followConfig.follow(ElevatorContants.kLeaderMotorID);
    }

    public void setSpeed(double speed){
        m_leaderMotor.set(MathUtil.clamp(speed, -maxOutput, maxOutput));
    }

    public double getSpeed(){
        return m_leaderMotor.get();
    }
}
