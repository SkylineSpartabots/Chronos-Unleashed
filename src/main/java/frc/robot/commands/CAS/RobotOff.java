package frc.robot.commands.CAS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotOff extends CommandBase {
    public RobotOff() {
        addRequirements(
            ShooterSubsystem.getInstance(),
            IndexerSubsystem.getInstance()
            );
    }

    @Override
    public void initialize() {
        ShooterSubsystem.getInstance().setShooterPercentPower(0);
        IndexerSubsystem.getInstance().setIndexerPercentPower(0, false);
        IndexerSubsystem.getInstance().setIntakePercentPower(0, false);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
