package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class SetIntakeDeploymentState extends CommandBase {
    private boolean intakeState;

    public SetIntakeDeploymentState(boolean intakeState) {
        addRequirements(IndexerSubsystem.getInstance());
        this.intakeState = intakeState;
    }

    @Override
    public void initialize() {
        IndexerSubsystem.getInstance().setIntakeDeploymentState(intakeState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
