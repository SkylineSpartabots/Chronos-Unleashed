package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class SetIntakeIndexerCommand extends CommandBase {
    private double intakePower, indexerPower;

    public SetIntakeIndexerCommand(double intakePower, double indexerPower) {
        addRequirements(IndexerSubsystem.getInstance());
        this.intakePower = intakePower;
        this.indexerPower = indexerPower;
    }

    @Override
    public void initialize() {
        IndexerSubsystem.getInstance().setIndexerPercentPower(indexerPower);
        IndexerSubsystem.getInstance().setIntakePercentPower(intakePower);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
