package frc.robot.commands.CAS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretAlign extends CommandBase {
	TurretSubsystem m_turret;
	DrivetrainSubsystem m_drivetrainSubsystem;
	double targetAngle;
	Translation2d m_targetPosition;
	double setpoint;

	public TurretAlign() {
		m_turret = TurretSubsystem.getInstance();
		m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();
		addRequirements(m_turret);
	}
	
	@Override
	public void initialize() {
		m_targetPosition = new Translation2d(Constants.targetHudPosition.getX(), Constants.targetHudPosition.getY());
		targetAngle = Math.toRadians(DrivetrainSubsystem.findAngle(m_drivetrainSubsystem.getPose(), m_targetPosition.getX(), m_targetPosition.getY(), 180));
		targetAngle = DrivetrainSubsystem.turretNormalize(targetAngle);
		if(Math.copySign(1, targetAngle) > 0) {
			setpoint = (targetAngle/90) * -9.5;
		} else if (Math.copySign(1, targetAngle) < 0) {
			setpoint = (targetAngle/-270) * 28;
		}
		if (setpoint > 26.5) {
			setpoint = 26.5;
		} else if (setpoint < -8) {
			setpoint = -8;
		}
	}

	@Override
	public void execute() {
		m_turret.setPosition(setpoint);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
