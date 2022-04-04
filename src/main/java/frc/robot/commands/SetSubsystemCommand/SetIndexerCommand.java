package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class SetIndexerCommand extends CommandBase {
    private final IndexerSubsystem m_subsystem;
    private double percentPower;
    private boolean autoIndexer;

    public SetIndexerCommand(double percentPower, boolean autoIndexer) {
        m_subsystem = IndexerSubsystem.getInstance();
        addRequirements(m_subsystem);
        this.percentPower = percentPower;
        this.autoIndexer = autoIndexer;
    }

    @Override
    public void initialize() {
        m_subsystem.setIndexerPercentPower(percentPower, autoIndexer);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}
