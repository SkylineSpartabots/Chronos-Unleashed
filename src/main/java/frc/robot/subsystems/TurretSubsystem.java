package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;

public class TurretSubsystem {
    private static TurretSubsystem instance;

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
    

    private TurretSubsystem() {
        mTurretMotor = new CANSparkMax(62, CANSparkMaxLowLevel.MotorType.kBrushless); // change this later
        mTurretMotorPID = mTurretMotor.getPIDController();
        mTurretMotor.restoreFactoryDefaults();
        mTurretMotorPID.setP(0.1);
        mTurretMotorPID.setI(1e-4);
        mTurretMotorPID.setD(0);
        mEncoder = mTurretMotor.getEncoder();
        setpoint = 0;
    }    

    public void setPosition(double pos) {
        mTurretMotorPID.setReference(pos, ControlType.kPosition);
        setpoint = pos;
    }

    public double getPosition() {
        return setpoint;
    }


}
