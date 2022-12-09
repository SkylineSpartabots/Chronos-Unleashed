package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class SetTurretCommand extends CommandBase {
    TurretSubsystem m_turret;
    boolean increase;
    public void initialize() {
        m_turret = TurretSubsystem.getInstance();
    }     

    public SetTurretCommand(boolean increase) {
        this.increase = increase;
    }

    public void execute() {
        m_turret.setPosition(m_turret.getPosition() + (increase ? 0.1 : -0.1));
    }

    public boolean isFinished() {
        return true;
    }
}
