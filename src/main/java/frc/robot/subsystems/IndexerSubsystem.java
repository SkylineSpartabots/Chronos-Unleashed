package frc.robot.subsystems;

import java.time.Period;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    
    private static final I2C.Port onboardI2C = I2C.Port.kOnboard;
    private static ColorSensorV3 m_intakeSensor;
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

        m_IntakeMotor = TalonFXFactory.createDefaultFalcon("Intake Motor", Ports.INTAKE_MOTOR);//creates motor
        m_IntakeMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        m_IntakeMotor.enableVoltageCompensation(true);        
        m_IntakeMotor.setNeutralMode(NeutralMode.Coast);
        m_intakeSensor = new ColorSensorV3(onboardI2C);
        setMultipleStatuFramePeriod();
    }
    private void setMultipleStatuFramePeriod(){
        m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);//if rev, 4500
        m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 253);//4750
        m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 253);
        m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_6_Misc, 251);//5000
        m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_6_Misc, 251);
        m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 249);//5250
        m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 249);
        m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 247);//5500
        m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 247);
        m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 245);//5750
        m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 245);
        m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 243);//5850
        m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 243);
        m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 241);//5950
        m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 241);
        m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 239);//6100
        m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 239);
        m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 237);//6250
        m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 237);
    }
    
    
    boolean autoIndexer = false;
    public void setIndexerPercentPower(double power, boolean autoIndexer) {
        this.autoIndexer = autoIndexer;
        m_IndexerMotor.set(ControlMode.PercentOutput, power);      
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
        
        SmartDashboard.putNumber("intake proximity", m_intakeSensor.getProximity());
        SmartDashboard.putBoolean("intake loaded", isIntakeBallLoaded());
        //SmartDashboard.putBoolean("intake autoIntake", autoIntake);
        //SmartDashboard.putNumber("Intake speed", m_IntakeMotor.getSelectedSensorVelocity());
        //SmartDashboard.putNumber("Intake Voltage", m_IntakeMotor.getMotorOutputVoltage());
        //SmartDashboard.putNumber("Intake Output Current", m_IntakeMotor.getStatorCurrent());
        SmartDashboard.putNumber("Intake Input Current", m_IntakeMotor.getSupplyCurrent());

        //SmartDashboard.putBoolean("indexer autoIntake", autoIndexer);
        //SmartDashboard.putNumber("indexer speed", m_IndexerMotor.getSelectedSensorVelocity());
        //SmartDashboard.putNumber("indexer Voltage", m_IndexerMotor.getMotorOutputVoltage());
        //SmartDashboard.putNumber("indexer Output Current", m_IndexerMotor.getStatorCurrent());
        SmartDashboard.putNumber("indexer Input Current", m_IndexerMotor.getSupplyCurrent());

        //SmartDashboard.putNumber("Indexer current from PDP", RobotContainer.getPDP().getCurrent(16));
        //SmartDashboard.putNumber("Intake current from PDP", RobotContainer.getPDP().getCurrent(3));

        if(autoIndexer && isIntakeBallLoaded()){   
            autoIndexer = false;
            new SequentialCommandGroup(
                new InstantCommand(() -> m_IndexerMotor.set(ControlMode.PercentOutput, Constants.indexerUp)),
                new WaitCommand(0.2),
                new InstantCommand(() -> m_IndexerMotor.set(ControlMode.PercentOutput, 0)));
        }
        else if(!autoIndexer && autoIntake && isIntakeBallLoaded()){
            autoIntake = false;
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                new InstantCommand(() -> m_IndexerMotor.set(ControlMode.PercentOutput, Constants.indexerUp)),
                new WaitCommand(0.2),
                new InstantCommand(() -> m_IndexerMotor.set(ControlMode.PercentOutput, 0)),                
                new InstantCommand(() -> m_IntakeMotor.set(ControlMode.PercentOutput, 0))
                );
        }        
    }

    public boolean isIntakeBallLoaded(){
        return m_intakeSensor.getProximity() >= Constants.kColorSensorLoadingDistance;
    }
    
}
