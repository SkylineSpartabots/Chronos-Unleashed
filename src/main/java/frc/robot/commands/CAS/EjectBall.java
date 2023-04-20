package frc.robot.commands.CAS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.Controller;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SetSubsystemCommand.SetIndexerCommand;
import frc.robot.commands.SetSubsystemCommand.SetIntakeCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class EjectBall extends CommandBase{ //REPLACABLE BY AIM SEQUENCE
    private PIDController m_thetaController;
    private Translation2d m_targetPosition = Constants.targetHudPosition;
    private final ShooterSubsystem m_shooter;
    private final IndexerSubsystem m_indexer;

    private final Timer m_timer = new Timer();

    public EjectBall() {
        m_shooter = ShooterSubsystem.getInstance();
        m_indexer = IndexerSubsystem.getInstance();
        addRequirements(m_shooter, m_indexer);
    }

    @Override
    public void initialize(){        
        m_timer.reset();
        m_timer.start();
        m_shooter.setShooterVelocity(5000);
        m_indexer.setIndexerPercentPower(Constants.indexerUp);
    }
/*
     @Override
     public boolean isFinished() {
         return m_timer.hasElapsed(0.3);//CALIBRATE THIS VALUE
     }
     */

    
    @Override
    public void end(boolean interruptable){  
        //stop intake, stop indexer
        m_indexer.setIndexerPercentPower(0.0);
        m_shooter.setShooterVelocity(Constants.shooterIdle);

    }

}
