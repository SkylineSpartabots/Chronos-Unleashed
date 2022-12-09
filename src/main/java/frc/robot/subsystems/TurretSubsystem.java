package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
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
        mTurretMotor = new CANSparkMax(60, CANSparkMaxLowLevel.MotorType.kBrushless); // change this later
        mTurretMotor.restoreFactoryDefaults();
        mTurretMotorPID = mTurretMotor.getPIDController();
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

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

}
