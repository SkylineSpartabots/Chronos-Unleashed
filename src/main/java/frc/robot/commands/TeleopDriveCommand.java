package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Controller;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TeleopDriveCommand extends CommandBase {
    protected DrivetrainSubsystem m_drivetrainSubsystem;
    private PIDController m_thetaController;


    public TeleopDriveCommand(DrivetrainSubsystem m_drivetrainSubsystem) {
        this.m_drivetrainSubsystem = m_drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
        m_controller = RobotContainer.getController();
        m_thetaController = new PIDController(6,0,0.7);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        driveWithJoystick();
    }

    protected final Controller m_controller;

    //limit accel/deccel
    protected SlewRateLimiter driveXFilter = new SlewRateLimiter(10);
    protected SlewRateLimiter driveYFilter = new SlewRateLimiter(10);
    protected SlewRateLimiter rotFilter = new SlewRateLimiter(30);
    
    public void driveWithJoystick() {
        // get joystick input for drive
        var xSpeed = -modifyAxis(m_controller.getLeftY()) * DriveConstants.kMaxSpeedMetersPerSecond;
        var ySpeed = -modifyAxis(m_controller.getLeftX()) * DriveConstants.kMaxSpeedMetersPerSecond;
        // regular turning
        var rot = -modifyAxis(m_controller.getRightX()*0.7) * DriveConstants.kMaxAngularSpeedRadiansPerSecond;

        // field-oriented turning test
        // var turnAngleX = m_controller.getRightX();
        // var turnAngleY = m_controller.getRightY();
        // double currentRotation = m_drivetrainSubsystem.getGyroscopeRotation().getRadians();
        // double targetAngle = calculateTurnAngle(-turnAngleY, -turnAngleX, currentRotation);
        // double rot = m_thetaController.calculate(currentRotation,targetAngle);
        // variable turn speed control
        // rot *= Math.hypot(turnAngleX, turnAngleY);


        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rotSpeed", rot/3.14159*180);
        
        
        //rotFilter.calculate
        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveXFilter.calculate(xSpeed), driveYFilter.calculate(ySpeed), 
            rotFilter.calculate(rot), m_drivetrainSubsystem.getGyroscopeRotation()));
        /*m_drivetrainSubsystem.drive(new ChassisSpeeds(driveXFilter.calculate(xSpeed), driveYFilter.calculate(ySpeed), 
        rotFilter.calculate(rot)));*/
    } 

    public static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    protected static double modifyAxis(double value) {
        // Deadband
        value = applyDeadband(value, 0.1);

        // Square the axis
        value = Math.copySign(value, value);

        return value;
    }
}