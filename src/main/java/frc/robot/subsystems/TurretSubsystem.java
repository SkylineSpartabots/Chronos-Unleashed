package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.TalonFXFactory;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    private static TurretSubsystem instance = null;

    public static TurretSubsystem getInstance() {
        if (instance == null) {
            instance = new TurretSubsystem();
        }
        return instance;
    }

    private final LazyTalonFX mTurretMotor;  
    private double setpoint;
    private double targetAngle;
    
    private TurretSubsystem() {
        mTurretMotor = TalonFXFactory.createDefaultFalcon("Turret Motor", 45);
        mTurretMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        mTurretMotor.enableVoltageCompensation(true);
        mTurretMotor.setNeutralMode(NeutralMode.Brake);
    }    

    public void setPosition(double pos) {
        mTurretMotor.configMotionAcceleration(10000); // accel limit for motion profile, test value
        mTurretMotor.configMotionCruiseVelocity(10000); // velo limit for motion profile, test value
        mTurretMotor.set(ControlMode.MotionMagic, pos);
        setpoint = pos;
    }
    
    public double getSetpoint() {
        return setpoint;
    }
    
    public double getPosition() {
        return mTurretMotor.getSelectedSensorPosition();
    }
    
    public void resetPosition() {
        mTurretMotor.setSelectedSensorPosition(0);
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    // counter clockwise +90
    // clockwise -270
    public double getAngle() {
        double angle = 0;
        if (Math.copySign(1, setpoint) < 0) {
            // depends on bounds for falcon
		} else if (Math.copySign(1, setpoint) > 0) {
            // depends on bounds for falcon
		}
        return angle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("turret setpoint", getSetpoint());
        SmartDashboard.putNumber("turret position", getPosition());
        SmartDashboard.putNumber("turret angle", getAngle());
        SmartDashboard.putNumber("turret target angle", getTargetAngle());
    }

}
