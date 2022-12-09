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

    public void initialize() {
        m_turret.setPosition(m_turret.getPosition() + (increase ? 0.1 : -0.1));
    }     

    public boolean isFinished() {
        return true;
    }
}
