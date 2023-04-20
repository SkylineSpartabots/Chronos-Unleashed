package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterCommand extends CommandBase {
    private final ShooterSubsystem m_subsystem;
    private double percent;

    public SetShooterCommand(double percent) {
        m_subsystem = ShooterSubsystem.getInstance();
        addRequirements(m_subsystem);
        this.percent = percent;
    }

    @Override
    public void initialize() {
       m_subsystem.setShooterPercentPower(percent);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}
