package frc.robot.commands.CAS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.Controller;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.SetSubsystemCommand.SetIndexerCommand;
import frc.robot.commands.SetSubsystemCommand.SetIntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimShoot extends TeleopDriveCommand{ //REPLACABLE BY AIM SEQUENCE
    private PIDController m_thetaController;
    private Translation2d m_targetPosition = Constants.targetHudPosition;
    private final ShooterSubsystem m_shooter;

    public AimShoot() {
        super(DrivetrainSubsystem.getInstance());
        m_shooter = ShooterSubsystem.getInstance();
        addRequirements(m_shooter);
       
        m_thetaController = new PIDController(0.5,0,0);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    boolean isIndexerOn = false;
    boolean hasRobertShotBall = false;
    
    @Override
    public void driveWithJoystick() {//called periodically
        var xSpeed = -modifyAxis(m_controller.getLeftY()) * DriveConstants.kMaxSpeedMetersPerSecond;
        var ySpeed = -modifyAxis(m_controller.getLeftX()) * DriveConstants.kMaxSpeedMetersPerSecond;
        double rot;

        
        double targetAngle = Math.toRadians(DrivetrainSubsystem.findAngle(m_drivetrainSubsystem.getPose(), m_targetPosition.getX(), m_targetPosition.getY(), 180));
        rot = m_thetaController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getRadians(),targetAngle);
        
        if(Math.abs(Math.toDegrees(m_drivetrainSubsystem.getGyroscopeRotation().getRadians() - targetAngle)) < 3.0){
            rot = 0;

            if(xSpeed == 0 && ySpeed == 0 && !isIndexerOn){
                //fire indexer
                
            new SequentialCommandGroup(
            //new WaitCommand(0.3),
            new SetIndexerCommand(Constants.indexerUp, false),
            new SetIntakeCommand(Constants.intakeOn, false)
            ).schedule();
                isIndexerOn = true;
                hasRobertShotBall = true;
            }
        }  
        else if(xSpeed != 0 && ySpeed != 0 && isIndexerOn){
            
            new SequentialCommandGroup(
            //new WaitCommand(0.3),
            new SetIndexerCommand(0.0, false),
            new SetIntakeCommand(0.0, false)
            ).schedule();
            //stop indexer
        }

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rotSpeed", rot);
    
        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveXFilter.calculate(xSpeed), driveYFilter.calculate(ySpeed), 
                rotFilter.calculate(rot), m_drivetrainSubsystem.getGyroscopeRotation()));
                //rot, m_drivetrainSubsystem.getGyroscopeRotation()));

    }
    
    @Override
    public void end(boolean interruptable){   
      if(hasRobertShotBall){
        new SequentialCommandGroup(
          //new WaitCommand(0.3),
          new SetIndexerCommand(Constants.indexerUp, true),
          new SetIntakeCommand(Constants.intakeOn, true)
        ).schedule();
      }   
    }



}
