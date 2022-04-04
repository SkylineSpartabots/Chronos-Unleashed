package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class SetIntakeCommand extends CommandBase {
    private final IndexerSubsystem m_subsystem;
    private double percentPower;
    private boolean autoIntake;

    public SetIntakeCommand(double percentPower, boolean autoIntake) {
        m_subsystem = IndexerSubsystem.getInstance();
        addRequirements(m_subsystem);
        this.percentPower = percentPower;
        this.autoIntake = autoIntake;
    }

    @Override
    public void initialize() {
        m_subsystem.setIntakePercentPower(percentPower, autoIntake);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}
