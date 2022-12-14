package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class SetTurretCommand extends CommandBase {
    TurretSubsystem m_turret;
    boolean increase;
    
    public SetTurretCommand(boolean increase) {
        m_turret = TurretSubsystem.getInstance();
        addRequirements(m_turret);
        this.increase = increase;
    }
    
    @Override
    public void initialize() {
        m_turret.setPosition(m_turret.getSetpoint() + (increase ? 1000 : -1000));
    }     

    @Override
    public boolean isFinished() {
        return true;
    }
}
