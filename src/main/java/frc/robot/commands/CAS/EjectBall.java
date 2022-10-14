package frc.robot.commands.CAS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class EjectBall extends CommandBase{ //REPLACABLE BY AIM SEQUENCE
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
        m_indexer.setIndexerPercentPower(Constants.indexerUp, false);
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
        m_indexer.setIndexerPercentPower(0.0, false);
        m_shooter.setShooterVelocity(Constants.shooterIdle);

    }

}
