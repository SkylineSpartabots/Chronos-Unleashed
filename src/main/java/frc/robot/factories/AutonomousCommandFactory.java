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
<<<<<<< HEAD
import frc.robot.subsystems.IndexerSubsystem;
=======
>>>>>>> turretV2
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.*;
import java.util.List;

public class AutonomousCommandFactory {

    public static Command getAutonomousCommand() {
        //return ballStealAuto();
<<<<<<< HEAD

        //return twoBallAuto();
        return oneBallAuto();
        //return fiveBallAuto();
=======
        //return twoBallAuto();
        //return oneBallAuto();
        return fiveBallAuto();
>>>>>>> turretV2
        //return experimentationalFiveBallAuto();
    }

    public static Pose2d getPose(double x, double y, double rot){
            return new Pose2d(x, y, new Rotation2d(Math.toRadians(rot)));
    }

    public static Command fiveBallAuto(){                
        return new SequentialCommandGroup(
<<<<<<< HEAD
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
=======
            new CalibrationCommand(getPose(7.57, 1.79, -89.18)),  
            new InstantCommand(() -> PivotSubsystem.getInstance().deployIntake()),    
            new SetIntakeCommand(intakeOn,false),
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed-500)),
            new TrajectoryDriveCommand(getPose(7.59, 0.80, -90.42), List.of(), false,0.3, 1.2 ,1),
            new WaitCommand(0.2),            
            new SetIntakeCommand(0.0,false),
            new TrajectoryDriveCommand(getPose(5.79, 2.42, -147.52), List.of(new Translation2d(6.29, 2.23)), true, 1.4, 4,1.4),
            new SetIndexerCommand(indexerUp,false),
            new SetIntakeCommand(intakeOn,false),
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed+57)),
            new TrajectoryDriveCommand(getPose(5.40, 2.16, -145.46), List.of(), false, 1.0, 0.45, 0.25),
            new WaitCommand(1.1),
            new SetIntakeCommand(intakeOn+0.15,true),
            new SetIndexerCommand(indexerUp,true),
            new TrajectoryDriveCommand(getPose(1.32, 1.62, -137.29), List.of(), false, 0.45, 5, 2.2),
            new WaitCommand(1.0),
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed+240)),
            new TrajectoryDriveCommand(getPose(5.40, 2.16, -148.46), List.of(), true, 0.5,5,1.8),
            new SetIntakeCommand(intakeOn,false),
            new SetIndexerCommand(indexerUp,false),
            new WaitCommand(3),
            new RobotOff()
            );
    }
    //previous auto
    public static Command fiveBallAutoOld(){                
        return new SequentialCommandGroup(
            new CalibrationCommand(getPose(7.57, 1.79, -89.18)),            
            new SetIntakeCommand(intakeOn,false),
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed+200)),
            new TrajectoryDriveCommand(getPose(7.59, 0.80, -90.42), List.of(), false,0.3, 1.2 ,1),
            new WaitCommand(0.2),            
            new SetIntakeCommand(0.0,false),
            new TrajectoryDriveCommand(getPose(5.79, 2.42, -147.52), List.of(new Translation2d(6.29, 2.23)), true, 1.4, 4,1.4),
            new SetIndexerCommand(indexerUp,false),
            new SetIntakeCommand(intakeOn,false),
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed+300)),
            new TrajectoryDriveCommand(getPose(5.40, 2.16, -145.46), List.of(), false, 1.0, 0.45, 0.25),
            new WaitCommand(1.2),
            new SetIntakeCommand(intakeOn+0.15,true),
            new SetIndexerCommand(indexerUp,true),
            new TrajectoryDriveCommand(getPose(1.50, 1.60, -137.29), List.of(), false, 0.2, 5, 2.0),
            new WaitCommand(1),
            new TrajectoryDriveCommand(getPose(5.40, 2.16, -145.46), List.of(), true, 0.5,5,2.0),
            new SetIntakeCommand(intakeOn,false),
            new SetIndexerCommand(indexerUp,false),
            new WaitCommand(3),
            new RobotOff()
            );
    }

    public static Command fiveBallAutoDiagnostics(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed+200)),
            new SetIntakeIndexerCommand(0.5,0.5),
            new CalibrationCommand(getPose(7.57, 1.79, -89.18)),            
            new TrajectoryDriveCommand(getPose(7.59, 0.80, -90.42), List.of(), false,0.3, 1 ,0.8),
            new TrajectoryDriveCommand(getPose(5.59, 2.58, -145.65), List.of(new Translation2d(6.14, 2.05)), true, 0.7, 4,1.4),
            new TrajectoryDriveCommand(getPose(5.18, 2.28, -145.57), List.of(), false, 1.0, 0.5, 0.3),
            new TrajectoryDriveCommand(getPose(1.22, 1.48, -137.29), List.of(), false, 0.2, 5, 2.0),
            new TrajectoryDriveCommand(getPose(5.18, 1.95, -145.57), List.of(), true, 0.5,5,2.0),
            new RobotOff()
            );
    }
    
    public static Command twoBallAutoBottomBottom(){
        return new SequentialCommandGroup(
            new CalibrationCommand(getPose(7.57, 1.79, -88.42)), 
            new InstantCommand(() -> PivotSubsystem.getInstance().deployIntake()),    
            new SetIntakeCommand(intakeOn,true),
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed)),
            new TrajectoryDriveCommand(getPose(7.66, 0.78, -100.04), List.of(), false,0.5, 2 ,1),
            new SetIntakeCommand(intakeOn,false),
            new SetIndexerCommand(indexerUp,false),
            new WaitCommand(3),
            new RobotOff()
            );
    }
    
    
    public static Command twoBallAutoBottomTop(){
        return new SequentialCommandGroup(
            new CalibrationCommand(getPose(6.59, 2.52, -134.64)),  
            new InstantCommand(() -> PivotSubsystem.getInstance().deployIntake()),    
            new SetIntakeCommand(intakeOn,true),
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed)),
            new TrajectoryDriveCommand(getPose(5.43, 2.16, -141.57), List.of(), false,0.5, 2 ,1),
            new SetIntakeCommand(intakeOn,false),
            new SetIndexerCommand(indexerUp,false),
            new WaitCommand(3),
            new RobotOff()
            );
    }
    
    
    public static Command twoBallAutoTopMiddle(){
        return new SequentialCommandGroup(
            new CalibrationCommand(getPose(6.07, 5.19, 135.47)),      
            new InstantCommand(() -> PivotSubsystem.getInstance().deployIntake()),    
            new SetIntakeCommand(intakeOn,true),
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed+180)),
            new TrajectoryDriveCommand(getPose(5.27, 5.98, 145.05), List.of(), false,0.5, 1 ,0.5),
            new WaitCommand(2),
            new SetIntakeCommand(intakeOn,false),
            new SetIndexerCommand(indexerUp,false),
            new WaitCommand(3),
            new RobotOff()
            );
    }

    public static Command oneBallTopBottom(){
        return new SequentialCommandGroup(
            new CalibrationCommand(getPose(5.93, 3.96, -178.05)),
            new InstantCommand(() -> PivotSubsystem.getInstance().deployIntake()),    
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed)),
            new TrajectoryDriveCommand(getPose(4.83, 3.86, -175.17), List.of(), false,0.5, 1 ,0.8),
            new SetIndexerCommand(indexerUp,false),
            new WaitCommand(3),
            new RobotOff()
            );
    }

    public static Command oneBallTopTop(){
        return new SequentialCommandGroup(
            new CalibrationCommand(getPose(6.77, 5.77, 114.68)),
            new InstantCommand(() -> PivotSubsystem.getInstance().deployIntake()),    
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed)),
            new TrajectoryDriveCommand(getPose(6.28, 6.88, 126.06), List.of(), false,0.5, 1 ,0.8),
            new SetIndexerCommand(indexerUp,false),
>>>>>>> turretV2
            new WaitCommand(3),
            new RobotOff()
            );
    }

    public static Command oneBallAuto(){//no navx auto
        return new SequentialCommandGroup(  
<<<<<<< HEAD
            new CalibrationCommand(getPose(0, 0, 0)),    
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(11200)),
            new TrajectoryDriveCommand(getPose(1.3, 0, 0), List.of(), false,0.5, 1 ,0.5),
            new WaitCommand(3),
            new SetIndexerCommand(indexerUp),
=======
            new CalibrationCommand(getPose(0, 0, 180)),    
            new InstantCommand(() -> PivotSubsystem.getInstance().deployIntake()),    
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed+230)),
            new TrajectoryDriveCommand(getPose(1.2, 0, 0), List.of(), false,0.5, 1 ,0.5),
            new WaitCommand(2),
            new SetIndexerCommand(indexerUp,false),
>>>>>>> turretV2
            new WaitCommand(3),
            new RobotOff()
            );
    }

    public static Command twoBallAuto(){    //no navx auto     
        return new SequentialCommandGroup(  
<<<<<<< HEAD
            new CalibrationCommand(getPose(0, 0, 0)),   
            new InstantCommand(() -> PivotSubsystem.getInstance().deployIntake()),
            new InstantCommand(() -> IndexerSubsystem.getInstance().automaticIntaking()),
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(11200)),
=======
            new CalibrationCommand(getPose(0, 0, 135)),     
            new InstantCommand(() -> PivotSubsystem.getInstance().deployIntake()),    
            new SetIntakeCommand(intakeOn,true),
            new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterFixed+300)),
>>>>>>> turretV2
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