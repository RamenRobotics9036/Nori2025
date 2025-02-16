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
import edu.wpi.first.wpilibj2.command.SubsystemBase; 
 
import com.revrobotics.RelativeEncoder; 
import com.revrobotics.spark.SparkBase; 
import com.revrobotics.spark.SparkClosedLoopController; 
import com.revrobotics.spark.SparkLowLevel.MotorType; 
 
import frc.robot.Constants.ElevatorContants; 

import frc.robot.Constants.IntakeConstants; 
 
public class ElevatorSystem extends SubsystemBase{ 
    //Motors are on opposate sides of a shaft 
    private final SparkMax m_leaderMotor = new SparkMax(ElevatorContants.kLeaderMotorID, MotorType.kBrushless); 
    private final SparkMax m_followMotor = new SparkMax(ElevatorContants.kFollowMotorID, MotorType.kBrushless); 
    private SparkMaxConfig m_leaderConfig = new SparkMaxConfig(); 
    private SparkMaxConfig m_followConfig = new SparkMaxConfig(); 
    private RelativeEncoder m_encoder = m_leaderMotor.getEncoder(); 
    private SparkClosedLoopController m_PIDController = m_leaderMotor.getClosedLoopController(); 
    private double m_desiredPosition; 
 
    private DigitalInput m_limitSwitch= new DigitalInput(ElevatorContants.kDIOIndex); 
    /* Sensor will reset a relative encoder when the elevator lowers fully 
     * Encoder will be used to prevent arm from going too high*/ 
 
    private double m_maxOutput = ElevatorContants.kMaxOutputPercentage; 
    private boolean m_positionInitialized = false; 
     
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
        encoderConfig.positionConversionFactor(ElevatorContants.kRotationToElevatorRatio); 
        //encoderConfig.velocityConversionFactor(ElevatorContants.kRotationToElevatorRatio / 60); 
 
        m_leaderConfig.idleMode(SparkBaseConfig.IdleMode.kBrake); 
        m_leaderConfig.smartCurrentLimit(IntakeConstants.kStallLimit); 
        m_leaderConfig.inverted(false); 
        m_leaderConfig.apply(closedLoopConfig); 
        m_leaderConfig.apply(encoderConfig); 
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
 
        testResetPosition(); 
        initShuffleboad(); 
    } 
 
    private void initShuffleboad(){ 
        ShuffleboardTab tab = Shuffleboard.getTab("Elevator"); 
        tab.addNumber("DesiredPosition", ()-> {return m_desiredPosition;}); 
        tab.addNumber("Position", this::getPosition); 
        tab.addBoolean("Initialized", ()-> {return m_positionInitialized;}); 
    } 
 
    @Override 
    public void periodic(){ 
        testResetPosition(); 
 
    } 
 
    public void setReference(double position){ 
        // set desired position 
        // measured in rotations of motor * a constant 
        m_desiredPosition = position; 
        m_PIDController.setReference(position, ControlType.kPosition); 
    } 
 
    public void setSpeed(double speed){ 
         
        if (!m_positionInitialized) { 
            return; 
        } 
         
        speed = MathUtil.clamp(speed, -m_maxOutput, m_maxOutput); 
 
        if (getPosition() >= 1.0 && speed > 0){ // TODO: placeholder, we don't know wether positive is up or down 
            return; 
        } 
 
        if (getPosition() <= 0.0 && speed < 0){ // TODO: placeholder, we don't know wether positive is up or down 
            return; 
        } 
 
        m_leaderMotor.set(speed); 
    } 
 
 
    private void testResetPosition(){ 
        if (m_limitSwitch.get()){ 
            m_encoder.setPosition(0); 
            m_positionInitialized = true; 
        } 
    } 
 
    public double getSpeed(){ 
        return m_leaderMotor.get(); 
    } 
 
    public double getPosition(){ 
        // 0.0 is bottom and 1.0 is top 
        if (m_positionInitialized){ 
            return m_encoder.getPosition() * ElevatorContants.kRotationToElevatorRatio; // TODO: placeholder, change this to the correct ratio 
        } else { 
            return -1; 
        } 
    } 
 
    public void stopSystem(){ 
        m_leaderMotor.stopMotor(); 
        m_followMotor.stopMotor(); // just in case 
    } 
} 

