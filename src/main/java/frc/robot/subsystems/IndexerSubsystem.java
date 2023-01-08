package frc.robot.subsystems;

import java.time.Period;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.revrobotics.ColorMatch;
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
import frc.robot.commands.SetSubsystemCommand.SetShooterCommand;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IndexerSubsystem extends SubsystemBase{
    private final LazyTalonFX m_IndexerMotor;
    public static int numberOfBalls;
    
    private static final I2C.Port onboardI2C = I2C.Port.kOnboard;
    private static ColorSensorV3 m_intakeSensor;
    private final LazyTalonFX m_IntakeMotor;
 
    private static IndexerSubsystem instance = null;
    public static IndexerSubsystem getInstance() {
        if (instance == null) {
            instance = new IndexerSubsystem();
        }
        return instance;
    }
    
    public IndexerSubsystem() {
        m_IndexerMotor = TalonFXFactory.createDefaultFalcon("Indexer Motor", Ports.INDEXER_MOTOR);//creates motor
        m_IndexerMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        m_IndexerMotor.enableVoltageCompensation(true);        
        m_IndexerMotor.setNeutralMode(NeutralMode.Brake);
        m_IndexerMotor.config_kP(0, 0.4);
        m_IndexerMotor.config_kI(0, 0);
        m_IndexerMotor.config_kD(0, 0);

        m_IntakeMotor = TalonFXFactory.createDefaultFalcon("Intake Motor", Ports.INTAKE_MOTOR);//creates motor
        m_IntakeMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        m_IntakeMotor.enableVoltageCompensation(true);        
        m_IntakeMotor.setNeutralMode(NeutralMode.Coast);

        m_intakeSensor = new ColorSensorV3(onboardI2C);
        
        m_colorMatcherIntake.addColorMatch(Constants.kColorSensorBlueIntake);
        m_colorMatcherIntake.addColorMatch(Constants.kColorSensorRedIntake);

        setMultipleStatuFramePeriod();
        numberOfBalls = 0;
    }

    public void automaticIntaking(){
        m_IntakeMotor.set(ControlMode.PercentOutput, Constants.intakeOn);  
        automaticIndexing = true;
        ready = true;
    }
    
    
    boolean autoIndexer = false;
    public void setIndexerPercentPower(double power, boolean autoIndexer) {
        this.autoIndexer = autoIndexer;
        if(!autoIndexer) m_IndexerMotor.set(ControlMode.PercentOutput, power);    
        else {
            setIntakePercentPower(Constants.intakeOn, true);
            m_IndexerMotor.set(ControlMode.PercentOutput, 0.0);
            waitForIndexer = true;
        }
    }
    
    boolean autoIntake = false;
    public void setIntakePercentPower(double power, boolean autoIntake) {
        this.autoIntake = autoIntake;
        m_IntakeMotor.set(ControlMode.PercentOutput, power);        
    }
    public void setIndexerPercentPower(double power){
        if(Math.abs(power)<0.001 && DriverStation.isTeleop()) automaticIndexing = true;
        else automaticIndexing = false;
        m_IndexerMotor.set(ControlMode.PercentOutput, power);
        ballCount=0;
    }

    public void fire(){
        ballCount=0;
        bottomBallIsWrong = false;
        setIndexerPercentPower(Constants.indexerUp);
    }

    private final int upAmount = 9000;
    private final int ejectAmount = upAmount * 5;
    public void moveUpIndexer(int amount){
        //m_IntakeMotor.set(ControlMode.PercentOutput, Constants.intakeOn); 
        m_IndexerMotor.set(ControlMode.Position, m_IndexerMotor.getSelectedSensorPosition() + amount);
    }

    boolean waitForIndexer = false;
    @Override
    public void periodic() {
        if(ready && automaticIndexing && ballCount <=2&& isIntakeBallLoaded()){
            ready = false;
            ballCount++;
            new SequentialCommandGroup(
                new WaitCommand(0.2),
                new InstantCommand(() -> ballIn())
                ).schedule(); 
        }

        SmartDashboard.putNumber("Ball Count", ballCount);
        SmartDashboard.putNumber("indexer position", m_IndexerMotor.getSelectedSensorPosition());

        SmartDashboard.putNumber("BALLS LOADED",numberOfBalls());
        SmartDashboard.putNumber("intake proximity", m_intakeSensor.getProximity());
        //SmartDashboard.putBoolean("intake loaded", isIntakeBallLoaded());
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
        SmartDashboard.putBoolean("Color correct", colorRight());
        SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().toString());
        //SmartDashboard.putNumber("Indexer current from PDP", RobotContainer.getPDP().getCurrent(16));
        //SmartDashboard.putNumber("Intake current from PDP", RobotContainer.getPDP().getCurrent(3));

        if(waitForIndexer && autoIndexer && isIntakeBallLoaded()){   
            waitForIndexer = false;
            new SequentialCommandGroup(
                new WaitCommand(0.25),
                new InstantCommand(() -> m_IndexerMotor.set(ControlMode.PercentOutput, Constants.indexerUp)),
                new InstantCommand(() -> autoIndexer = false),
                new InstantCommand(() -> moveIndexer())
                ).schedule();            
        }
        else if(!autoIndexer && autoIntake && isIntakeBallLoaded()){
            autoIntake = false;
            new SequentialCommandGroup(
                new WaitCommand(0.25),
                new InstantCommand(() -> m_IndexerMotor.set(ControlMode.PercentOutput, Constants.indexerUp)),
                new WaitCommand(0.25),
                new InstantCommand(() -> m_IndexerMotor.set(ControlMode.PercentOutput, 0)),                
                new InstantCommand(() -> m_IntakeMotor.set(ControlMode.PercentOutput, 0))
                ).schedule();
        }        
    }
    private int numberOfBalls(){
        int number = 0;
        if(!autoIntake) number++;
        if(!autoIndexer) number++;
        return number;
    }

    public boolean isIntakeBallLoaded(){
        return m_intakeSensor.getProximity() >= Constants.kColorSensorLoadingDistance;
    }
    
    public void moveIndexer(){
        if(autoIntake && isIntakeBallLoaded()){
            autoIntake = false;
            new SequentialCommandGroup(
                new WaitCommand(0.4),
                new InstantCommand(() -> m_IndexerMotor.set(ControlMode.PercentOutput, 0)),             
                new InstantCommand(() -> m_IntakeMotor.set(ControlMode.PercentOutput, 0))              
                ).schedule();
        }
        else{
            new SequentialCommandGroup(
                new WaitCommand(0.25),
                new InstantCommand(() -> m_IndexerMotor.set(ControlMode.PercentOutput, 0))                
                ).schedule();
        }
    }

}

