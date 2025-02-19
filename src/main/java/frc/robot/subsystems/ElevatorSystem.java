package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;

public class ElevatorSystem extends SubsystemBase{
    //Motors are on opposate sides of a shaft
    private final SparkMax m_leaderMotor = new SparkMax(ElevatorConstants.kLeaderMotorID, MotorType.kBrushless);
    private final SparkMax m_followMotor = new SparkMax(ElevatorConstants.kFollowMotorID, MotorType.kBrushless);
    private SparkMaxConfig m_leaderConfig = new SparkMaxConfig();
    private SparkMaxConfig m_followConfig = new SparkMaxConfig();
    private RelativeEncoder m_encoder = m_leaderMotor.getEncoder();
    private SparkClosedLoopController m_PIDController = m_leaderMotor.getClosedLoopController();
    private double m_desiredPosition;

    private DigitalInput m_limitSwitch= new DigitalInput(ElevatorConstants.kDIOIndex);
    /* Sensor will reset a relative encoder when the elevator lowers fully
     * Encoder will be used to prevent arm from going too high*/

    private double m_maxOutput = ElevatorConstants.kMaxOutputPercentage;

    enum states {
        INIT,
        READYLOW,
        READY,
    }
    private states m_state = states.INIT;
    
    public ElevatorSystem() {
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
        closedLoopConfig
            .p(1)
            .i(0)
            .d(0);
        closedLoopConfig.positionWrappingEnabled(false);
        //closedLoopConfig.minOutput();
        //closedLoopConfig.maxOutput();
        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(ElevatorConstants.kRotationToElevatorRatio);
        encoderConfig.velocityConversionFactor(ElevatorConstants.kRotationToElevatorRatio / 60);

        m_leaderConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        m_leaderConfig.smartCurrentLimit(IntakeConstants.kStallLimit);
        m_leaderConfig.inverted(false);
        m_leaderConfig.apply(closedLoopConfig);
        m_leaderConfig.apply(encoderConfig);
        m_leaderMotor.configure(m_leaderConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);

        m_followConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        m_followConfig.smartCurrentLimit(IntakeConstants.kStallLimit);
        m_followConfig.inverted(true);
        m_followConfig.follow(ElevatorConstants.kLeaderMotorID);
        m_followMotor.configure(m_followConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);

        initShuffleboad();
        // bringDownElevator().schedule();
    }

    private void initShuffleboad(){
        ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
        tab.addNumber("DesiredPosition", ()-> {return m_desiredPosition;});
        tab.addNumber("Position", this::getPosition);
        //tab.addBoolean("Initialized", ()-> {return m_positionInitialized;});
    }

    @Override
    public void periodic(){
        switch (m_state) {
            case INIT:
                if (m_limitSwitch.get()) {
                    m_state = states.READYLOW;
                    resetEncoder();
                    initializeMotorConfig(true);
                }
                break;
            case READYLOW:
                if (!m_limitSwitch.get()) {
                    m_state = states.READY;
                }
                break;
            case READY:
                if (m_limitSwitch.get()) {
                    m_state = states.INIT; // TODO: Stop for safety
                    initializeMotorConfig(false);
                }
        }
    }

    public void setReference(double position){
        // set desired position
        // measured in rotations of motor * a constant
        double currentEncoderPosition = m_encoder.getPosition();
        if (m_state == states.READY){
            m_desiredPosition = MathUtil.clamp(position, ElevatorConstants.kMinHeight, ElevatorConstants.kMaxHeight);
            m_PIDController.setReference(position, ControlType.kPosition);
        } else if (m_state == states.READYLOW) {
            m_desiredPosition = MathUtil.clamp(position, currentEncoderPosition, ElevatorConstants.kMaxHeight);
            m_PIDController.setReference(m_desiredPosition, ControlType.kPosition);
        }
    }

    private void initializeMotorConfig(boolean isBrakeMode) {
        m_leaderConfig.idleMode(isBrakeMode ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
        m_followConfig.idleMode(isBrakeMode ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);

        m_leaderMotor.configure(m_leaderConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);
        m_followMotor.configure(m_followConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);
    }

    private void resetEncoder(){
        m_encoder.setPosition(0);
    }

    public double getSpeed(){
        return m_leaderMotor.get();
    }

    public double getPosition(){
        // 0.0 is bottom and 1.0 is top
        if (m_state == states.READY || m_state == states.READYLOW){
            return m_encoder.getPosition(); // TODO: placeholder, change this to the correct ratio
        } else {
            return -1;
        }
    }

    public void stopSystem(){
        m_leaderMotor.stopMotor();
        m_followMotor.stopMotor(); // just in case
    }
}
