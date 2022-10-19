package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IndexerSubsystem.IndexerControlState;

public class SetIntakeIndexerCommand extends CommandBase {
    private IndexerControlState newState;

    public SetIntakeIndexerCommand(IndexerControlState newState) {
        addRequirements(IndexerSubsystem.getInstance());
        this.newState = newState;
    }

    @Override
    public void initialize() {
        IndexerSubsystem.getInstance().setState(newState);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}
