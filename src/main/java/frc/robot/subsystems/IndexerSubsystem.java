package frc.robot.subsystems;

import java.time.Period;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Ports;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IndexerSubsystem extends SubsystemBase{   

    private final LazyTalonFX m_IndexerMotor;
    private static final I2C.Port kMxpI2C = I2C.Port.kMXP;
    private static TunedColorSensor m_indexerSensor;
    
    private static final I2C.Port onboardI2C = I2C.Port.kOnboard;
    private static TunedColorSensor m_intakeSensor;
    private final LazyTalonFX m_IntakeMotor;

    //get instance of subsystem    
    private static IndexerSubsystem instance = null;
    public static IndexerSubsystem getInstance() {
        if (instance == null) {
            instance = new IndexerSubsystem();
        }
        return instance;
    }

    //MERGE INDEXER SUBSYSTEM WITH INTAKE SUBSYSTEM???

    
    public IndexerSubsystem() {//TODO: ADD CURRENT LIMITATIONS
        m_IndexerMotor = TalonFXFactory.createDefaultFalcon("Indexer Motor", Ports.INDEXER_MOTOR);//creates motor
        //configure motor
        m_IndexerMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        m_IndexerMotor.enableVoltageCompensation(true);        
        m_IndexerMotor.setNeutralMode(NeutralMode.Brake);
        m_indexerSensor = new TunedColorSensor(Constants.kColorSensorIndexerDistance, kMxpI2C);

        m_IntakeMotor = TalonFXFactory.createDefaultFalcon("Intake Motor", Ports.INTAKE_MOTOR);//creates motor
        m_IntakeMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        m_IntakeMotor.enableVoltageCompensation(true);        
        m_IntakeMotor.setNeutralMode(NeutralMode.Coast);
        m_intakeSensor = new TunedColorSensor(Constants.kColorSensorLoadingDistance, onboardI2C);
    }
    
    private class TunedColorSensor{
        protected ColorSensorV3 colorSensor;
        protected double distanceThreshold;
        public TunedColorSensor(double threshold, I2C.Port port){
            colorSensor = new ColorSensorV3(port);
            distanceThreshold = threshold;
        }
    }
    
    boolean autoIndexer = false;
    public void setIndexerPercentPower(double power, boolean autoIndexer) {
        this.autoIndexer = autoIndexer;
        if(autoIndexer){
            if(!isIndexerBallLoaded())
                m_IndexerMotor.set(ControlMode.PercentOutput, power);
            
        }
        else{
            m_IndexerMotor.set(ControlMode.PercentOutput, power);
        }
        
    }
    
    boolean autoIntake = false;
    public void setIntakePercentPower(double power, boolean autoIntake) {
        this.autoIntake = autoIntake;

        if(autoIntake){
            if(!isIntakeBallLoaded())
                m_IntakeMotor.set(ControlMode.PercentOutput, power);
        }
        else{            
            m_IntakeMotor.set(ControlMode.PercentOutput, power);
        }
    }

    public void startAutoIntaking(){
        setIndexerPercentPower(Constants.indexerUp, true);
        setIntakePercentPower(Constants.intakeOn, true);
    }
    public void startAutoShooting(){
        setIndexerPercentPower(Constants.indexerUp, false);
        setIntakePercentPower(Constants.intakeOn, false);
    }
    public double getIntakeVelocity(){
        return m_IntakeMotor.getSelectedSensorVelocity();
    }
    public double getIndexerVelocity(){
        return m_IndexerMotor.getSelectedSensorVelocity();
    }
    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("intake proximity", m_intakeSensor.colorSensor.getProximity());
        SmartDashboard.putBoolean("intake loaded", isIntakeBallLoaded());
        //SmartDashboard.putBoolean("intake autoIntake", autoIntake);
        //SmartDashboard.putNumber("Intake speed", m_IntakeMotor.getSelectedSensorVelocity());
        //SmartDashboard.putNumber("Intake Voltage", m_IntakeMotor.getMotorOutputVoltage());
        //SmartDashboard.putNumber("Intake Output Current", m_IntakeMotor.getStatorCurrent());
        SmartDashboard.putNumber("Intake Input Current", m_IntakeMotor.getSupplyCurrent());

        SmartDashboard.putNumber("indexer proximity", m_indexerSensor.colorSensor.getProximity());
        SmartDashboard.putBoolean("indexer loaded", isIndexerBallLoaded());
        //SmartDashboard.putBoolean("indexer autoIntake", autoIndexer);
        //SmartDashboard.putNumber("indexer speed", m_IndexerMotor.getSelectedSensorVelocity());
        //SmartDashboard.putNumber("indexer Voltage", m_IndexerMotor.getMotorOutputVoltage());
        //SmartDashboard.putNumber("indexer Output Current", m_IndexerMotor.getStatorCurrent());
        SmartDashboard.putNumber("indexer Input Current", m_IndexerMotor.getSupplyCurrent());

        //SmartDashboard.putNumber("Indexer current from PDP", RobotContainer.getPDP().getCurrent(16));
        //SmartDashboard.putNumber("Intake current from PDP", RobotContainer.getPDP().getCurrent(3));

        if(autoIndexer && isIndexerBallLoaded()){                
            m_IndexerMotor.set(ControlMode.PercentOutput, 0);
            autoIndexer = false;
        }
        else if(!autoIndexer && autoIntake && isIntakeBallLoaded()){
            m_IntakeMotor.set(ControlMode.PercentOutput, 0);
        }
        
    }

    public boolean isIndexerBallLoaded(){
        return m_indexerSensor.colorSensor.getProximity() >= m_indexerSensor.distanceThreshold;
    }
    public boolean isIntakeBallLoaded(){
        return m_intakeSensor.colorSensor.getProximity() >= m_intakeSensor.distanceThreshold;
    }
    
}
