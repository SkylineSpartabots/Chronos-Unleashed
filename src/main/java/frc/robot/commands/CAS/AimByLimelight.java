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
import frc.lib.util.Controller;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AimByLimelight extends TeleopDriveCommand{ //REPLACABLE BY AIM SEQUENCE
    private PIDController m_thetaController;
    private LimelightSubsystem m_limelightSubsystem;
    String direction = "";

    public AimByLimelight(String direction) {
        super(DrivetrainSubsystem.getInstance());
       
        m_thetaController = new PIDController(0.5,0,0);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_limelightSubsystem = LimelightSubsystem.getInstance();
        this.direction = direction;
    }
    
    public AimByLimelight() {
        super(DrivetrainSubsystem.getInstance());
       
        m_thetaController = new PIDController(0.5,0,0);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_limelightSubsystem = LimelightSubsystem.getInstance();
    }
    
    @Override
    public void driveWithJoystick() {
        var xSpeed = -modifyAxis(m_controller.getLeftY()) * DriveConstants.kMaxSpeedMetersPerSecond;
        var ySpeed = -modifyAxis(m_controller.getLeftX()) * DriveConstants.kMaxSpeedMetersPerSecond;
        double rot;

        
        if(LimelightSubsystem.getInstance().hasTarget()){
            rot = m_thetaController.calculate(Math.toRadians(LimelightSubsystem.getInstance().getXOffset()),0.0);
            if(Math.abs(m_limelightSubsystem.getXOffset()) < 3.0){
                rot = 0;
            }
        }
        else{
            if(direction.equals("right")){    //lol too lazy to use enums does anyone want to change this            
                rot = Math.toRadians(-150);
            }
            else if (direction.equals("left")){                
                rot = Math.toRadians(150);
            }
            else{
                rot = 0;
            }
        }      

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rotSpeed", rot);
    
        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveXFilter.calculate(xSpeed), driveYFilter.calculate(ySpeed), 
                rotFilter.calculate(rot), m_drivetrainSubsystem.getGyroscopeRotation()));
                //rot, m_drivetrainSubsystem.getGyroscopeRotation()));
    }
}
