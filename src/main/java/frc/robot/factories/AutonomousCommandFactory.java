package frc.robot.factories;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.commands.CAS.AimShoot;
import frc.robot.commands.CAS.RobotOff;
import frc.robot.commands.SetSubsystemCommand.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.*;
import java.util.List;

public class AutonomousCommandFactory {

    public static Command getAutonomousCommand() {
        //return ballStealAuto();

        //return twoBallAuto();
        //return oneBallAuto();
        return fiveBallAuto();
        //return experimentationalFiveBallAuto();
    }

    public static Pose2d getPose(double x, double y, double rot){
            return new Pose2d(x, y, new Rotation2d(Math.toRadians(rot)));
    }

    public static Command fiveBallAuto(){                
        return new SequentialCommandGroup(
            new CalibrationCommand(getPose(7.57, 1.79, -89.18)),
            new InstantCommand(() -> PivotSubsystem.getInstance().deployIntake()),    
            new SetIntakeCommand(intakeOn),
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(10000)),
            new TrajectoryDriveCommand(getPose(7.59, 0.80, -90.42), List.of(), false,0.3, 1.5 ,0.8),
            new WaitCommand(0.2),            
            new SetIntakeCommand(0.0),
            new TrajectoryDriveCommand(getPose(5.80, 2.42, -145.45), List.of(new Translation2d(6.29, 2.23)), true, 1.0, 4,1.2),
            new SetIndexerCommand(indexerUp),
            new SetIntakeCommand(intakeOn),
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(10900)),
            new TrajectoryDriveCommand(getPose(5.32, 2.09, -145.45), List.of(), false, 1.0, 0.45, 0.25),
            new WaitCommand(1.0),
            new SetIndexerCommand(0.0),
            new InstantCommand(() -> IndexerSubsystem.getInstance().automaticIntaking()),
            new TrajectoryDriveCommand(getPose(1.31, 1.62, -137.29), List.of(), false, 0.5, 5, 2.2),
            new WaitCommand(0.2),
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(11111)),
            new TrajectoryDriveCommand(getPose(5.32, 2.09, -145.45), List.of(), true, 0.5,5,2.0),
            new InstantCommand(() -> PivotSubsystem.getInstance().retractIntake()), 
            new SetIntakeCommand(intakeOn),
            new SetIndexerCommand(indexerUp),
            new WaitCommand(3),
            new RobotOff()
            );
    }

    public static Command oneBallAuto(){//no navx auto
        return new SequentialCommandGroup(  
            new CalibrationCommand(getPose(0, 0, 0)),    
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(11000)),
            new TrajectoryDriveCommand(getPose(1.3, 0, 0), List.of(), false,0.5, 1 ,0.5),
            new WaitCommand(3),
            new SetIndexerCommand(indexerUp),
            new WaitCommand(3),
            new RobotOff()
            );
    }

    public static Command twoBallAuto(){    //no navx auto     
        return new SequentialCommandGroup(  
            new CalibrationCommand(getPose(0, 0, 0)),   
            new InstantCommand(() -> PivotSubsystem.getInstance().deployIntake()),
            new InstantCommand(() -> IndexerSubsystem.getInstance().automaticIntaking()),
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(11000)),
            new TrajectoryDriveCommand(getPose(1.3, 0, 0), List.of(), false,0.5, 1 ,0.5),
            new WaitCommand(2.5),
            new SetIndexerCommand(indexerUp),
            new WaitCommand(0.5),
            new SetIntakeCommand(intakeOn),
            new WaitCommand(3),
            new InstantCommand(() -> PivotSubsystem.getInstance().retractIntake()),
            new RobotOff()
            );
    }


    

    public static Command PIDTest(){
        DrivetrainSubsystem m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();

        Pose2d position1 = getPose(7.781, 2.937, -110.1519);
        Pose2d position2 = getPose(7.475, 1.756, -87.28149);

        Command resetOdo = new InstantCommand(()->m_drivetrainSubsystem.resetOdometryFromPosition(position1), m_drivetrainSubsystem);

        Command driveToPosition2 = new TrajectoryDriveCommand(position2, List.of(), false);


        return new SequentialCommandGroup(
            resetOdo,
            driveToPosition2
            );
    }

}