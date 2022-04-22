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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IndexerSubsystem extends SubsystemBase{
    private final LazyTalonFX m_IndexerMotor;
    
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
    }

    public void automaticIntaking(){
        m_IntakeMotor.set(ControlMode.PercentOutput, Constants.intakeOn);  
        automaticIndexing = true;
        ready = true;
    }

    public void setIntakePercentPower(double power) {
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
    private final int ejectAmount = upAmount * 3;
    public void moveUpIndexer(int amount){
        //m_IntakeMotor.set(ControlMode.PercentOutput, Constants.intakeOn); 
        m_IndexerMotor.set(ControlMode.Position, m_IndexerMotor.getSelectedSensorPosition() + amount);
    }

    private int ballCount = 0;
    private boolean bottomBallIsWrong = false;
    private boolean ready = true;
    private boolean automaticIndexing = true;
    @Override
    public void periodic() {
        if(ready && automaticIndexing && ballCount <=2&& isIntakeBallLoaded()){
            ready = false;
            ballCount++;
            new SequentialCommandGroup(
                new WaitCommand(0.25),
                new InstantCommand(() -> ballIn())
                ).schedule(); 
        }

        SmartDashboard.putNumber("Ball Count", ballCount);
        SmartDashboard.putNumber("indexer position", m_IndexerMotor.getSelectedSensorPosition());

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



    }
    public void ballIn(){
        //if(isCorrectColor()){
            moveUpIndexer(upAmount);
            new SequentialCommandGroup(new WaitCommand(0.25), new InstantCommand(() -> ready = true)).schedule();
            
            if(ballCount >= 2) setIntakePercentPower(0);
       /* }
        else{
            if(ballCount==0){
                moveUpIndexer(ejectAmount);
                new SequentialCommandGroup(new WaitCommand(0.5), 
                new InstantCommand(() -> moveUpIndexer(-upAmount)),
                new WaitCommand(0.25),
                new InstantCommand(() -> ready = true));
            }
            else{
                bottomBallIsWrong = true;
            }
        }*/
    }
    
    private static final ColorMatch m_colorMatcherIntake = new ColorMatch();
    private boolean isCorrectColor(){
        return true;
        //Color c =  m_colorMatcherIntake.matchClosestColor(m_intakeSensor.getColor()).color;
        //return c.equals(DriverStation.getAlliance().equals(Alliance.Red)?Constants.kColorSensorRedIntake:Constants.kColorSensorBlueIntake);
    }
    
    private boolean colorRight(){
        //return true;
        Color c =  m_colorMatcherIntake.matchClosestColor(m_intakeSensor.getColor()).color;
        return c.equals(DriverStation.getAlliance().equals(Alliance.Red)?Constants.kColorSensorRedIntake:Constants.kColorSensorBlueIntake);
    }
    
    public boolean isIntakeBallLoaded(){
        return m_intakeSensor.getProximity() >= Constants.kColorSensorLoadingDistance;
    }

    //first ball position
    //second ball
    //move second ball to first ball to shoot
    //eject first ball
    //eject second ball

    //eject both balls

    //start intaking

    //manual overrides for both



    private void setMultipleStatuFramePeriod(){
        m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        
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

}

