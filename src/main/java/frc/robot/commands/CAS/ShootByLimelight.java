package frc.robot.commands.CAS;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.SetSubsystemCommand.SetIndexerCommand;
import frc.robot.commands.SetSubsystemCommand.SetIntakeIndexerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootByLimelight extends CommandBase {
    private final ShooterSubsystem m_shooter;
    private boolean moveIndexer;


    public ShootByLimelight(boolean moveIndexer) {
        m_shooter = ShooterSubsystem.getInstance();
        addRequirements(m_shooter);
        this.moveIndexer = moveIndexer;
    }

    @Override
    public void initialize() {
    }

    double targetVel = 0;
    int threshold = 200;
    @Override
    public void execute(){
      LimelightSubsystem m_limelightSubsystem = LimelightSubsystem.getInstance();
      int shooterSpeed = (int) Constants.shooterFixed;
      if(!m_limelightSubsystem.hasTarget()){
        //shooterSpeed = (int) Constants.shooterRamped;
      }
      else{
        shooterSpeed = calculateShooterSpeed(LimelightSubsystem.getInstance().getDistance());
      }
      
      targetVel = shooterSpeed;
      m_shooter.setShooterVelocity(shooterSpeed);
    }
    private int calculateShooterSpeed(double distance){

      double shooterSlope = 1099;
      double shooterIntercept = 6700.0;

      double minVelocity = 9500;
      double maxVelocity = 12500;

      
      double targetShooterVelocity = shooterSlope * distance + shooterIntercept;

      if(targetShooterVelocity > maxVelocity) {
        targetShooterVelocity = maxVelocity;
      }
      else if(targetShooterVelocity < minVelocity){
        targetShooterVelocity = minVelocity;
      }
      

      SmartDashboard.putNumber("Shooter Target", targetShooterVelocity);
      SmartDashboard.putNumber("Distance Away", distance);

      return (int)targetShooterVelocity;
    }

    @Override
    public boolean isFinished(){
      //return LimelightSubsystem.getInstance().hasTarget() && Math.abs(LimelightSubsystem.getInstance().getXOffset()) < 3;  
      //int currentVel = Integer.parseInt(SmartDashboard.getNumber("SVel", 10000) + ""); 
      int currentVel = m_shooter.getVelocity();
      int threshold = 200;  
      return LimelightSubsystem.getInstance().hasTarget() && Math.abs(LimelightSubsystem.getInstance().getXOffset()) < 3
       && (currentVel >= targetVel - threshold && currentVel <= targetVel + threshold);                  
    }
    @Override
    public void end(boolean interruptable){   
      if(moveIndexer){
        new SequentialCommandGroup(
          //new WaitCommand(0.3),
          new SetIntakeIndexerCommand(Constants.intakeOn, Constants.indexerUp)
        ).schedule();
      }   
    }

    class DistanceShooter{
      public double distance;
      public double shooter;
      public DistanceShooter(double distance, double shooter){
          this.distance = distance;
          this.shooter = shooter;
      }
    }
}

