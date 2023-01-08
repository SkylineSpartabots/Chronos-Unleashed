package frc.robot.subsystems;


import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.TalonFXFactory;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.Ports;

public class PivotSubsystem extends SubsystemBase {
    //get instance
    private static PivotSubsystem instance = null;          
    public static PivotSubsystem getInstance() {
        if (instance == null) {
            instance = new PivotSubsystem();
        }
        return instance;
    }

    private final LazyTalonFX m_pivotMotor;  
    private boolean intakeToggle = false;
    public BooleanSupplier intakeState = () -> intakeToggle;


    PIDController shooterController;
    private PivotSubsystem() {
        m_pivotMotor = TalonFXFactory.createDefaultFalcon("Master Shooter Motor", Ports.PIVOT_MOTOR);
        configureMotor(m_pivotMotor, true);
        m_pivotMotor.setSelectedSensorPosition(0.0);
        setMultipleStatuFramePeriod();
    }
    private void setMultipleStatuFramePeriod(){
        m_pivotMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);//if rev, 4500
        m_pivotMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 253);//4750
        m_pivotMotor.setStatusFramePeriod(StatusFrame.Status_6_Misc, 251);//5000
        m_pivotMotor.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 249);//5250
        m_pivotMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 247);//5500
        m_pivotMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 245);//5750
        m_pivotMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 243);//5850
        m_pivotMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 241);//5950
        m_pivotMotor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 239);//6100
        m_pivotMotor.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 237);//6250
    }

    private void configureMotor(LazyTalonFX talon, boolean b){
        talon.setInverted(b);
        talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Brake);

        talon.config_kF(0, 1.0, Constants.kTimeOutMs);
        talon.config_kP(0, 1.0, Constants.kTimeOutMs);
        talon.config_kI(0, 0, Constants.kTimeOutMs);
        talon.config_kD(0, 0, Constants.kTimeOutMs);
    }

    public void retractIntake(){
        m_pivotMotor.configMotionAcceleration(10000);
        m_pivotMotor.configMotionCruiseVelocity(10000);
        moveToPosition(0);
        intakeToggle = false;
    }
    public void deployIntake(){
        m_pivotMotor.configMotionAcceleration(7000);
        m_pivotMotor.configMotionCruiseVelocity(4000);
        moveToPosition(34000);
        intakeToggle = true;
    }

    public void moveToPosition(double target){
        SmartDashboard.putNumber("Target Position", target);
        m_pivotMotor.set(ControlMode.MotionMagic, target);
    }
    public void resetIntakeDown(){
        m_pivotMotor.setSelectedSensorPosition(34000);
    }
    public void resetIntakeUp(){
        m_pivotMotor.setSelectedSensorPosition(0);
    }
<<<<<<< HEAD
    
=======
>>>>>>> turretV2

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Pivot Position", m_pivotMotor.getSelectedSensorPosition());
    }


}