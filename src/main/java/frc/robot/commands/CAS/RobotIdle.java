package frc.robot.commands.CAS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem.IndexerControlState;

public class RobotIdle extends CommandBase {


    public RobotIdle() {
      addRequirements(
          ShooterSubsystem.getInstance(),
          IndexerSubsystem.getInstance()
          );
    }

    @Override
    public void initialize() {        
        ShooterSubsystem.getInstance().setShooterVelocity(shooterIdle);
        IndexerSubsystem.getInstance().setState(IndexerControlState.AUTO);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
