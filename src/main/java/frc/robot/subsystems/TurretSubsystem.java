package frc.robot.subsystems;

import java.lang.annotation.Target;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
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
    private static double targetAngle;
    
    // complete 360 is from -9.5 to 28
    // -8 to 26.5 bounds
    // -8 is 90 degrees counterclockwise
    // 26.5 270 degrees clockwise

    private TurretSubsystem() {
        mTurretMotor = TalonFXFactory.createDefaultFalcon("Turret Motor", 0);
        mTurretMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        mTurretMotor.enableVoltageCompensation(true);
        mTurretMotor.setNeutralMode(NeutralMode.Brake);
    }    

    public void setPosition(double pos) {
        mTurretMotor.set(ControlMode.MotionMagic, pos);
        setpoint = pos;
    }

    public void resetPosition() {
        mEncoder.setPosition(0);
    }

    public double getPosition() {
        return setpoint;
    }

    public void setAngle(double angle) {
        targetAngle = angle;
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
        SmartDashboard.putNumber("target angle", targetAngle);
        SmartDashboard.putNumber("actual angle", getAngle());
    }

}
