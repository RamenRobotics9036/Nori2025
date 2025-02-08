package frc.robot.subsystems;

import org.dyn4j.exception.ValueOutOfRangeException;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;


public class IntakeArmSystem extends SubsystemBase{
    private SparkMax m_armMotor = new SparkMax(ArmConstants.kArmMotorID, MotorType.kBrushless);
    private RelativeEncoder m_armRelativeEncoder = m_armMotor.getEncoder();
    private SparkMaxConfig m_armConfig = new SparkMaxConfig();
    private double maxOutput = ArmConstants.maxOutput;
    private SparkClosedLoopController m_armPIDController = m_armMotor.getClosedLoopController();
    private final DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(ArmConstants.kArmEncoderID);
    private double desiredAngle = 0;

    //sets the idle mode of both motors to kBrake and adds a smartCurrentLimit
    public IntakeArmSystem(){
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig
            .p(1)
            .i(0)
            .d(0);
        closedLoopConfig.positionWrappingEnabled(true);
        closedLoopConfig.positionWrappingMinInput(ArmConstants.kMinArmRotation);
        closedLoopConfig.positionWrappingMaxInput(ArmConstants.kMaxArmRotation);

        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor((Math.PI * 2) / ArmConstants.kArmGearBoxRatio);
        encoderConfig.velocityConversionFactor(((Math.PI * 2) / ArmConstants.kArmGearBoxRatio) / 60);
        // Did not set distance per rotation

        // m_armConfig.inverted(true);

        m_armConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_armConfig.smartCurrentLimit(IntakeConstants.kStallLimit);

        m_armConfig.apply(closedLoopConfig);
        m_armConfig.apply(encoderConfig);

        m_armMotor.configure(m_armConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);

        
        m_armRelativeEncoder.setPosition(
            (m_armEncoder.get() * 2 * Math.PI)
                    % (2 * Math.PI));

        if (m_armEncoder.get() == Math.PI * 2) {
            throw new ValueOutOfRangeException("ARM ABSOLUTE ENCODER NOT PLUGGED IN!", m_armEncoder.get());
        }
    }

    public void initShuffleboad() {
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
        tab.addDouble("Arm Relative Encoder", () -> m_armRelativeEncoder.getPosition());
        tab.addDouble("Arm Encoder", () -> m_armEncoder.get());
        tab.addDouble("Desired Angle", () -> desiredAngle);
  }

    public void setReference(double position) {
        desiredAngle = position;
        m_armPIDController.setReference(position, ControlType.kPosition);
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
