package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.OperatorConstants;
import frc.robot.ramenutils.logging.TriviaLogger;
import frc.robot.Constants.IntakeConstants;

public class IntakeSystem extends SubsystemBase{
    //the front motor, for pulling the coarl in and scoring on L1
    private SparkMax m_pullMotor = new SparkMax(IntakeConstants.kPullMotorID, MotorType.kBrushless);
    private SparkMaxConfig m_pullConfig = new SparkMaxConfig();
    //the back motor, for loading onto the robot
    private SparkMax m_loadMotor = new SparkMax(IntakeConstants.kLoadMotorID, MotorType.kBrushless);
    private SparkMaxConfig m_loadConfig = new SparkMaxConfig();

    private double maxOutput = IntakeConstants.kMaxOutputPercentage;
    private RelativeEncoder m_pullMotorRelativeEncoder = m_pullMotor.getEncoder();
    private RelativeEncoder m_loadMotorRelativeEncoder = m_loadMotor.getEncoder();

    //Proximity sensors for detecting coral
    private DigitalInput m_proximitySensor = new DigitalInput(IntakeConstants.kProximitySensorPort);

    //sets the idle mode of both motors to kBrake and adds a smartCurrentLimit
    public IntakeSystem(){
        //pull inverted because of motor positioning
        m_pullConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_pullConfig.smartCurrentLimit(IntakeConstants.kStallLimit);
        m_pullConfig.inverted(false);

        m_pullMotor.configure(m_pullConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);

        m_loadConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        m_loadConfig.smartCurrentLimit(IntakeConstants.kStallLimit);
        m_loadConfig.inverted(true);
        m_loadMotor.configure(m_loadConfig, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);

        initShuffleboard();
    }

    public void initShuffleboard() {
        if (!OperatorConstants.kCompetitionMode){
            ShuffleboardTab tab = Shuffleboard.getTab("Intake");
            tab.addBoolean("Is Holding Coral", this::isHoldingCoral);
        }

        initLogging();
    }

    public void initLogging() {
        // Log current command on this subsystem.
        TriviaLogger logger = TriviaLogger.getInstance();
        logger.registerSubsystemCmdCallback(
        getSubsystem(),
        () -> (this.getCurrentCommand() == null) ? "None"
                : this.getCurrentCommand().getName());
    }

    //sets the speed of m_pullMotor. Cannot exceed maxOutputPercentage
    public void setPullMotorSpeed(double speed){
        speed = MathUtil.clamp(speed, -maxOutput, maxOutput);
        m_pullMotor.set(speed);
    }
    //sets the speed of m_loadMotor. Cannot exceed maxOutputPercentage
    public void setLoadMotorSpeed(double speed){
        speed = MathUtil.clamp(speed, -maxOutput, maxOutput);
        m_loadMotor.set(speed);
    }


    public boolean isHoldingCoral() {
        // Removed encoder B because not working.
        return m_proximitySensor.get();
    }

    //gets the speed of m_pullMotor
    public double getPullMotorSpeed(){
        return m_pullMotor.get();
    }
    //gets the speed of m_loadMotor
    public double getLoadMotorSpeed(){
        return m_loadMotor.get();
    }

    //gets position from RELATIVE encoder
    public double getPullMotorPosition() {
        return m_pullMotorRelativeEncoder.getPosition();
    }

    //gets position from RELATIVE encoder
    public double getLoadMotorPosition() {
        return m_loadMotorRelativeEncoder.getPosition();
    }

    //stops everything
    public void stopSystem(){
        m_pullMotor.stopMotor();
        m_loadMotor.stopMotor();
    }
}
