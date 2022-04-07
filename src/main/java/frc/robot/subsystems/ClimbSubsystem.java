package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.LazyTalonSRX;
import frc.lib.drivers.TalonFXFactory;
import frc.lib.drivers.TalonSRXFactory;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.Ports;

public class ClimbSubsystem extends SubsystemBase {
    private static ClimbSubsystem instance = null;

    public static ClimbSubsystem getInstance(){
        if(instance == null){
            instance = new ClimbSubsystem();
        }
        return instance;
    }

    //left motor handles the side closer to the shooter, right handles the side closer to intake
    private final LazyTalonFX mLeftClimb, mRightClimb;
    private final LazyTalonSRX mLeftPivot, mRightPivot;

    private ClimbSubsystem(){
        mLeftClimb = TalonFXFactory.createDefaultFalcon("Left Climb Motor", Ports.LEFT_CLIMB);
        configureMotor(mLeftClimb, false);
        mRightClimb = TalonFXFactory.createDefaultFalcon("Right Climb Motor", Ports.RIGHT_CLIMB);
        configureMotor(mRightClimb, false);
        mLeftPivot = TalonSRXFactory.createDefaultTalon("Left Pivot Motor", Ports.LEFT_PIVOT);
        configureMotor(mLeftPivot, true);
        mRightPivot = TalonSRXFactory.createDefaultTalon("Right Pivot Motor", Ports.RIGHT_PIVOT);
        configureMotor(mRightPivot, false);
        setMultipleStatuFramePeriod();
    }
    private void setMultipleStatuFramePeriod(){
        mLeftClimb.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);//if rev, 4500
        mRightClimb.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        mLeftClimb.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 253);//4750
        mRightClimb.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 253);
        mLeftClimb.setStatusFramePeriod(StatusFrame.Status_6_Misc, 251);//5000
        mRightClimb.setStatusFramePeriod(StatusFrame.Status_6_Misc, 251);
        mLeftClimb.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 249);//5250
        mRightClimb.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 249);
        mLeftClimb.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 247);//5500
        mRightClimb.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 247);
        mLeftClimb.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 245);//5750
        mRightClimb.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 245);
        mLeftClimb.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 243);//5850
        mRightClimb.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 243);
        mLeftClimb.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 241);//5950
        mRightClimb.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 241);
        mLeftClimb.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 239);//6100
        mRightClimb.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 239);
        mLeftClimb.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 237);//6250
        mRightClimb.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 237);
        
        mLeftPivot.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);//if rev, 4500
        mRightPivot.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        mLeftPivot.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 253);//4750
        mRightPivot.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 253);
        mLeftPivot.setStatusFramePeriod(StatusFrame.Status_6_Misc, 251);//5000
        mRightPivot.setStatusFramePeriod(StatusFrame.Status_6_Misc, 251);
        mLeftPivot.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 249);//5250
        mRightPivot.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 249);
        mLeftPivot.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 247);//5500
        mRightPivot.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 247);
        mLeftPivot.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 245);//5750
        mRightPivot.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 245);
        mLeftPivot.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 243);//5850
        mRightPivot.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 243);
        mLeftPivot.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 241);//5950
        mRightPivot.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 241);
        mLeftPivot.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 239);//6100
        mRightPivot.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 239);
        mLeftPivot.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 237);//6250
        mRightPivot.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 237);
    }

    private void configureMotor(LazyTalonFX talon, boolean b){
        talon.setInverted(b);
        talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Brake);
    }

    private void configureMotor(TalonSRX talon, boolean b){
        talon.setInverted(b);
        talon.setNeutralMode(NeutralMode.Brake);
    }

    public void setPercentPower(LazyTalonFX talon, double power){
        talon.set(ControlMode.PercentOutput, power);
    } 

    public void setPercentPower(LazyTalonSRX talon, double power){
        talon.set(ControlMode.PercentOutput, power);
    }    
    public void leftClimbPower(double power){
        setPercentPower(mLeftClimb, power);
    }
    public void rightClimbPower(double power){
        setPercentPower(mRightClimb, power);
    }

    public void pivotPower(double power){
        setPercentPower(mLeftPivot, power);
        setPercentPower(mRightPivot, power);
    }
    public void leftPivotPower(double power){
        setPercentPower(mLeftPivot, power);
    }
    public void rightPivotPower(double power){
        setPercentPower(mRightPivot, power);
    }
    @Override
    public void periodic(){        
        SmartDashboard.putNumber("R Climb Input Current", mRightClimb.getSupplyCurrent());
        SmartDashboard.putNumber("l Climb Input Current", mLeftClimb.getSupplyCurrent());
       /* SmartDashboard.putNumber("L Climb Voltage", mLeftClimb.getMotorOutputVoltage());
        SmartDashboard.putNumber("L Climb Output Current", mLeftClimb.getStatorCurrent());
        SmartDashboard.putNumber("l Climb Input Current", mLeftClimb.getSupplyCurrent());
        SmartDashboard.putNumber("R Climb Voltage", mRightClimb.getMotorOutputVoltage());
        SmartDashboard.putNumber("R Climb Output Current", mRightClimb.getStatorCurrent());
        SmartDashboard.putNumber("R Climb Input Current", mRightClimb.getSupplyCurrent());
        SmartDashboard.putNumber("L Pivot Voltage", mLeftPivot.getMotorOutputVoltage());
        SmartDashboard.putNumber("L Pivot Output Current", mLeftPivot.getStatorCurrent());
        SmartDashboard.putNumber("L Pivot Input Current", mLeftPivot.getSupplyCurrent());
        SmartDashboard.putNumber("R Pivot Voltage", mRightPivot.getMotorOutputVoltage());
        SmartDashboard.putNumber("R Pivot Output Current", mRightPivot.getStatorCurrent());
        SmartDashboard.putNumber("R Pivot Input Current", mRightPivot.getSupplyCurrent());*/

        //SmartDashboard.putNumber("LClimb", RobotContainer.getPDP().getCurrent(9));
        //SmartDashboard.putNumber("RClimb", RobotContainer.getPDP().getCurrent(8));
    }

}
