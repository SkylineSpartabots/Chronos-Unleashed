package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class SetIndexerCommand extends CommandBase {
    private final IndexerSubsystem m_subsystem;
    private double percentPower;

    public SetIndexerCommand(double percentPower) {
        m_subsystem = IndexerSubsystem.getInstance();
        addRequirements(m_subsystem);
        this.percentPower = percentPower;
    }

    @Override
    public void initialize() {
        m_subsystem.setIndexerPercentPower(percentPower);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}
