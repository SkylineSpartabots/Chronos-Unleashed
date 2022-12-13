package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    private static TurretSubsystem instance = null;

    public static TurretSubsystem getInstance() {
        if (instance == null) {
            instance = new TurretSubsystem();
        }
        return instance;
    }

    private CANSparkMax mTurretMotor;
    private SparkMaxPIDController mTurretMotorPID;
    private RelativeEncoder mEncoder;
    private double setpoint;
    
    // complete 360 is from -9.5 to 28
    // -8 to 26.5 bounds
    // -8 is 90 degrees counterclockwise
    // 26.5 270 degrees clockwise

    private TurretSubsystem() {
        mTurretMotor = new CANSparkMax(60, CANSparkMaxLowLevel.MotorType.kBrushless); // change this later
        mTurretMotor.restoreFactoryDefaults();
        mTurretMotorPID = mTurretMotor.getPIDController();
        mTurretMotorPID.setP(0.1);
        mTurretMotorPID.setI(1e-4);
        mTurretMotorPID.setD(0);
        mEncoder = mTurretMotor.getEncoder();
        setpoint = mEncoder.getPosition();
    }    

    public void setPosition(double pos) {
        mTurretMotorPID.setReference(pos, ControlType.kPosition);
        setpoint = pos;
    }

    public double getPosition() {
        return setpoint;
    }

    // counter clockwise +90
    // clockwise -270
    public double getAngle() {
        double angle = 0;
        if (Math.copySign(1, setpoint) < 0) {
            angle = (setpoint/-9.5) * 90;
		} else if (Math.copySign(1, setpoint) > 0) {
            angle = (setpoint/28) * -270;
		}
        return angle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("setpoint", setpoint);
        SmartDashboard.putNumber("position", mEncoder.getPosition());
    }

}
