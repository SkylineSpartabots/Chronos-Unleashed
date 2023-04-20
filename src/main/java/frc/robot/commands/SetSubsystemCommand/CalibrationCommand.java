package frc.robot.commands.SetSubsystemCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class CalibrationCommand extends CommandBase {
    Pose2d position;
    public CalibrationCommand(Pose2d pos) {
        position = pos;
        addRequirements(Swerve.getInstance());
    }

    @Override
    public void initialize() {
        Swerve.getInstance().resetOdometry(position);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}
