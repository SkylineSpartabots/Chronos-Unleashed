package frc.robot.commands.CAS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.Controller;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.SetSubsystemCommand.SetIndexerCommand;
import frc.robot.commands.SetSubsystemCommand.SetIntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimShoot extends TeleopDriveCommand{ //REPLACABLE BY AIM SEQUENCE
    private PIDController m_thetaController;
    private Translation2d m_targetPosition = Constants.targetHudPosition;
    private final ShooterSubsystem m_shooter;

    public AimShoot() {
        super(DrivetrainSubsystem.getInstance());
        m_shooter = ShooterSubsystem.getInstance();
        isIndexerOn = false;
        addRequirements(m_shooter, IndexerSubsystem.getInstance());
       
        m_thetaController = new PIDController(6,0,0);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    boolean isIndexerOn = false;
    boolean hasRobertShotBall = false;
    boolean shooterWithinBounds = false;
    int shooterSpeed;

    @Override
    public void driveWithJoystick() {//called periodically
        var xSpeed = -modifyAxis(m_controller.getLeftY()) * DriveConstants.kMaxSpeedMetersPerSecond;
        var ySpeed = -modifyAxis(m_controller.getLeftX()) * DriveConstants.kMaxSpeedMetersPerSecond;
        double xSpeedFiltered = driveXFilter.calculate(xSpeed);
        double ySpeedFiltered = driveYFilter.calculate(ySpeed);
        
        /*double maxSpeed = 1.0;
        if(Math.abs(xSpeedFiltered) > maxSpeed) xSpeedFiltered = Math.copySign(maxSpeed, xSpeedFiltered);
        if(Math.abs(ySpeedFiltered) > maxSpeed) ySpeedFiltered = Math.copySign(maxSpeed, ySpeedFiltered);*/

        double turnThreshold = 6;
        double time = 0.02;
        m_targetPosition = new Translation2d(Constants.targetHudPosition.getX() - xSpeedFiltered * time, Constants.targetHudPosition.getY() - ySpeedFiltered * time);
        // this code is so amazing -atharv
        double targetAngle = Math.toRadians(DrivetrainSubsystem.findAngle(m_drivetrainSubsystem.getPose(), m_targetPosition.getX(), m_targetPosition.getY(), 180));
        double currentRotation = m_drivetrainSubsystem.getGyroscopeRotation().getRadians();
        double rot = m_thetaController.calculate(currentRotation,targetAngle + (Math.toRadians(turnThreshold) * Math.copySign(1, DrivetrainSubsystem.normalize(currentRotation-targetAngle))));
        
        shooterSpeed = calculateShooterSpeed(DrivetrainSubsystem.distanceFromHub(m_targetPosition.getX(), m_targetPosition.getY()));
        m_shooter.setShooterVelocity(shooterSpeed);
        
        int currentVel = m_shooter.getVelocity();
        int threshold = 200; 
        double angleDiff = DrivetrainSubsystem.normalize(Math.toDegrees(currentRotation - targetAngle));

        boolean isFacingTarget = Math.abs(angleDiff) < turnThreshold;
        boolean isRobotNotMoving = xSpeedFiltered == 0 && ySpeedFiltered == 0;
        boolean isShooterAtSpeed = (currentVel >= shooterSpeed - threshold && currentVel <= shooterSpeed + threshold);
        boolean isReadyToShoot = isShooterAtSpeed && shooterWithinBounds && isFacingTarget && isRobotNotMoving;

        DrivetrainSubsystem.getInstance().setHubPosition(m_targetPosition.getX(), m_targetPosition.getY());
        SmartDashboard.putNumber("hubX", m_targetPosition.getX());
        SmartDashboard.putNumber("hubY", m_targetPosition.getY());
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rotSpeed", rot);
        SmartDashboard.putNumber("CAS Shooter Target", shooterSpeed);
        SmartDashboard.putNumber("CAS Shooter Speed", currentVel);
        SmartDashboard.putNumber("CAS Distance Away", DrivetrainSubsystem.distanceFromHub(m_targetPosition.getX(), m_targetPosition.getY()));
        SmartDashboard.putNumber("CAS AngleDiff", angleDiff);
        SmartDashboard.putBoolean("?Facing Target", isFacingTarget);
        SmartDashboard.putBoolean("?Robot Not Moving", isRobotNotMoving);
        SmartDashboard.putBoolean("?Shooter At Speed", isShooterAtSpeed);
        SmartDashboard.putBoolean("?Indexer Off", !isIndexerOn);
        SmartDashboard.putBoolean("?shooter within bounds", shooterWithinBounds);
        SmartDashboard.putBoolean("?Ready To Shoot", isReadyToShoot);

        if(Math.abs(angleDiff) < turnThreshold){
            rot = 0;            
        }  
        if(isReadyToShoot && !isIndexerOn){
            //fire indexer if aimed, robot is not moving, shooter is at speed, and indexer is off              
            IndexerSubsystem.getInstance().setIndexerPercentPower(Constants.indexerUp, false);
            IndexerSubsystem.getInstance().setIntakePercentPower(Constants.intakeOn, false);
            isIndexerOn = true;
            hasRobertShotBall = true;
        }
        /*else if(!isReadyToShoot && isIndexerOn){
            IndexerSubsystem.getInstance().setIndexerPercentPower(0.0, false);
            IndexerSubsystem.getInstance().setIntakePercentPower(0.0, false);
            isIndexerOn = false;
            //stop indexer if robot is moving and indexer is on
        }*/
        
        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedFiltered, ySpeedFiltered, 
                //rotFilter.calculate(rot), m_drivetrainSubsystem.getGyroscopeRotation()));
                rot, m_drivetrainSubsystem.getGyroscopeRotation()));
    }
    
    @Override
    public void end(boolean interruptable){  
        isIndexerOn = false;
        if(hasRobertShotBall){
            IndexerSubsystem.getInstance().setIndexerPercentPower(Constants.indexerUp, true);
            IndexerSubsystem.getInstance().setIntakePercentPower(Constants.intakeOn, true);
            ShooterSubsystem.getInstance().setShooterVelocity(Constants.shooterIdle);
        }   
        hasRobertShotBall = false;
    }

    private int calculateShooterSpeed(double distance){

        double shooterSlope = 1099;
        double shooterIntercept = 6000.0;
  
        double minVelocity = 8000;
        double maxVelocity = 12500;
  
        
        double targetShooterVelocity = shooterSlope * distance + shooterIntercept;
        shooterWithinBounds = true;
        if(targetShooterVelocity > maxVelocity) {
          targetShooterVelocity = maxVelocity;
          shooterWithinBounds = false;
        }
        else if(targetShooterVelocity < minVelocity){
          targetShooterVelocity = minVelocity;
          shooterWithinBounds = false;
        }
          
        return (int)targetShooterVelocity;
      }

}
