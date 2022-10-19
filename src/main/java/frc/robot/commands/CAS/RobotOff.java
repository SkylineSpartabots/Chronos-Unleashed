package frc.robot.commands.CAS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem.IndexerControlState;

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
        IndexerSubsystem.getInstance().setState(IndexerControlState.OFF);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
