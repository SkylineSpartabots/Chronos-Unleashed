package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class SetIntakeIndexerCommand extends CommandBase {
    private double intakePower, indexerPower;

    private IndexerSubsystem m_Indexer;

    public SetIntakeIndexerCommand(double intakePower, double indexerPower) {
        m_Indexer = IndexerSubsystem.getInstance();
        addRequirements(m_Indexer);
        this.intakePower = intakePower;
        this.indexerPower = indexerPower;
    }

    @Override
    public void initialize() {
        m_Indexer.setIndexerPercentPower(indexerPower);
        m_Indexer.setIntakePercentPower(intakePower);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
