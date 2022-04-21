package frc.robot.subsystems;


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

public class ShooterSubsystem extends SubsystemBase {
    //get instance
    private static ShooterSubsystem instance = null;          
    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    private final LazyTalonFX mMasterShooter, mSlaveShooter;  

    PIDController shooterController;
    private ShooterSubsystem() {
        mMasterShooter = TalonFXFactory.createDefaultFalcon("Master Shooter Motor", Ports.MASTER_SHOOTER_MOTOR);
        configureMotor(mMasterShooter, true);
        mSlaveShooter = TalonFXFactory.createSlaveFalcon("Follower Shooter Motor", Ports.FOLLOW_SHOOTER_MOTOR, Ports.MASTER_SHOOTER_MOTOR);
        mSlaveShooter.setMaster(mMasterShooter);
        configureMotor(mSlaveShooter, false);
        setMultipleStatuFramePeriod();
    }
    private void setMultipleStatuFramePeriod(){
        mSlaveShooter.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);//if rev, 4500
        mSlaveShooter.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 253);//4750
        mSlaveShooter.setStatusFramePeriod(StatusFrame.Status_6_Misc, 251);//5000
        mSlaveShooter.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 249);//5250
        mSlaveShooter.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 247);//5500
        mSlaveShooter.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 245);//5750
        mSlaveShooter.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 243);//5850
        mSlaveShooter.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 241);//5950
        mSlaveShooter.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 239);//6100
        mSlaveShooter.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 237);//6250
    }

    private void configureMotor(LazyTalonFX talon, boolean b){
        talon.setInverted(b);
        talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Coast);
        talon.config_kF(0, 0.05, Constants.kTimeOutMs);
        talon.config_kP(0, 0.12, Constants.kTimeOutMs);
        talon.config_kI(0, 0, Constants.kTimeOutMs);
        talon.config_kD(0, 0, Constants.kTimeOutMs);
    }

    public void stopShooter(){
        mMasterShooter.set(ControlMode.PercentOutput, 0);
    }
    //percent power: -1 through 1. Voltage compensated
    public void setShooterPercentPower(double power) {
        mMasterShooter.set(ControlMode.PercentOutput, power);
    }

    int velocity = 0;
    //unit: rotations per 100 ms. Shooter velocity for against the hub: 10,000 rp100ms
    public void setShooterVelocity(double velocity){        
        this.velocity = (int) velocity;
        mMasterShooter.set(ControlMode.Velocity, velocity);
    }

    public void increaseShooterVelocity(double speed){    
        int amount = (int)speed;
        if(mMasterShooter.getSelectedSensorVelocity()< 1000) amount = 1000;
        this.velocity = (int)mMasterShooter.getSelectedSensorVelocity() + (int)amount;
        setShooterVelocity(velocity);
    }

    //THROWS ERROR DO NOT USE
    //detects if shooter is at a RPS. Ex: shooterAtVelocityRPS(10000)
    public boolean isShooterAtVelocity(int velocity, int threshold){
        double actual = mMasterShooter.getSelectedSensorVelocity();
        return actual >= velocity - threshold && actual <= velocity + threshold;
    }
    public int getVelocity()
    {
        return (int) mMasterShooter.getSelectedSensorVelocity();
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("S%", mMasterShooter.getMotorOutputPercent());
        SmartDashboard.putNumber("STarg", velocity);
        SmartDashboard.putNumber("SVel", mMasterShooter.getSelectedSensorVelocity());
        //SmartDashboard.putNumber("Shooter Voltage", mMasterShooter.getMotorOutputVoltage());
        //SmartDashboard.putNumber("Shooter Output Current", mMasterShooter.getStatorCurrent());
        SmartDashboard.putNumber("SInput", mMasterShooter.getSupplyCurrent());
        SmartDashboard.putNumber("SVoltage", mSlaveShooter.getMotorOutputVoltage());
        //SmartDashboard.putNumber("Slave Shooter Output Current", mSlaveShooter.getStatorCurrent());
        SmartDashboard.putNumber("Slave Input", mSlaveShooter.getSupplyCurrent());
        //SmartDashboard.putBoolean("Is Shooter At Velocity?", isShooterAtVelocity(10000, 150));

        //SmartDashboard.putNumber("S2", RobotContainer.getPDP().getCurrent(10));
        //SmartDashboard.putNumber("S1", RobotContainer.getPDP().getCurrent(11));
    }

    public static int calculateShooterSpeed(double distance){

        double shooterSlope = 1099; //1661
        double shooterIntercept = 6500.0; //3300
  
        double minVelocity = 8500;
        double maxVelocity = 14000;
  
        
        double targetShooterVelocity = shooterSlope * distance + shooterIntercept;
        if(targetShooterVelocity > maxVelocity) {
          targetShooterVelocity = maxVelocity;
        }
        else if(targetShooterVelocity < minVelocity){
          targetShooterVelocity = minVelocity;
        }
          
        return (int)targetShooterVelocity;
      }
}