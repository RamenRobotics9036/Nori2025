package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeArmSystem extends SubsystemBase{
    private SparkMax m_armMotor = new SparkMax(IntakeConstants.kArmMotorID, MotorType.kBrushless);
    private SparkMaxConfig m_armConfig = new SparkMaxConfig();
    private DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(IntakeConstants.kArmEncoderID);
    private double maxOutput = IntakeConstants.kMaxOutputPercentage;

    //sets the idle mode of both motors to kBrake and adds a smartCurrentLimit
    public IntakeArmSystem(){
        m_armConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_armConfig.smartCurrentLimit(IntakeConstants.kStallLimit);
        // m_armConfig.inverted(true);
        m_armMotor.configure(m_armConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);
    }

    //sets the speed of m_armMotor. Cannot exceed maxOutputPercentage
    public void setArmMotorSpeed(double speed){
        speed = MathUtil.clamp(speed, -maxOutput, maxOutput);
        m_armMotor.set(speed);
    }

    //gets the speed of m_armMotor
    public double getArmMotorSpeed(){
        return m_armMotor.get();
    }



    // get encoder value
    public double getArmAngle() {
        return m_armEncoder.get();
    }

    // public double getAbsoluteArmAngle() {
    //     return m_armEncoder.getAbsolutePosition();
    // }

    // public void resetArmAngle() {
    //     m_armEncoder.reset();
    // }

    //stops everything
    public void stopSystem(){
        m_armMotor.stopMotor();
    }
}
