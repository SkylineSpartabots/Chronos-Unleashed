// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.lib.util.Controller;
import frc.lib.util.DeviceFinder;
import frc.robot.commands.*;
import frc.robot.commands.CAS.AimShoot;
import frc.robot.commands.CAS.EjectBall;
import frc.robot.commands.CAS.RobotIdle;
import frc.robot.commands.CAS.RobotOff;
import frc.robot.commands.CAS.TurretAlign;
import frc.robot.commands.SetSubsystemCommand.*;
import frc.robot.factories.AutonomousCommandFactory;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import static frc.robot.Constants.*;
import frc.robot.commands.SetSubsystemCommand.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private LimelightSubsystem m_limelight;
  private IndexerSubsystem m_indexerSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private ClimbSubsystem m_climbSubsystem;
  private PivotSubsystem m_pivotSubsystem;
  private TurretSubsystem m_turretSubsystem;
  
  /*private static PowerDistribution powerModule = new PowerDistribution(1, ModuleType.kRev);

  public static PowerDistribution getPDP(){
    return powerModule;
  }*/
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    m_limelight = LimelightSubsystem.getInstance();
    m_indexerSubsystem = IndexerSubsystem.getInstance();
    m_shooterSubsystem = ShooterSubsystem.getInstance();
    m_climbSubsystem = ClimbSubsystem.getInstance();
    m_pivotSubsystem = PivotSubsystem.getInstance();
    // m_turretSubsystem = TurretSubsystem.getInstance();

    // Set the scheduler to log Shuffleboard events for command initialize,
    // interrupt, finish
    /*CommandScheduler.getInstance().onCommandInitialize(command -> Shuffleboard.addEventMarker(
        "Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance().onCommandInterrupt(command -> Shuffleboard.addEventMarker(
        "Command interrupted", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance().onCommandFinish(command -> Shuffleboard.addEventMarker(
        "Command finished", command.getName(), EventImportance.kNormal));
*/
    // Configure the button bindings
    configureButtonBindings();

  }

  private static final Controller m_controller = new Controller(new XboxController(0));
  private static final Controller m_controller2 = new Controller(new XboxController(1));

  public static Controller getController(){
    return m_controller;
  }

  public static void printDiagnostics(){
    /*SmartDashboard.putBoolean("NavX Connected?", DrivetrainSubsystem.getInstance().getNavxConnected());
    SmartDashboard.putBoolean("Limelight Connected?", LimelightSubsystem.getInstance().isConnected());
    SmartDashboard.putBoolean("Can Bus Connected?", isCanConnected());
    SmartDashboard.putBoolean("Battery Charged?", isBatteryCharged());*/
  }

  private static boolean isBatteryCharged(){
    return RobotController.getBatteryVoltage() >= Constants.kMinimumBatteryVoltage;
  }

  private static boolean isCanConnected(){
    return DeviceFinder.Find().size() == Constants.kCanDeviceCount;
  }
  // configures button bindings to controller
  private void configureButtonBindings() {
    final double triggerDeadzone = 0.2;

    //FIRST CONTROLLER

    //DPAD
    Trigger dpadUp = new Trigger(() -> {return m_controller.getDpadUp();});
    Trigger dpadUpRight = new Trigger(() -> {return m_controller.getDpadUpRight();});
    Trigger dpadDown = new Trigger(() -> {return m_controller.getDpadDown();});
    Trigger dpadLeft = new Trigger(() -> {return m_controller.getDpadLeft();});
    Trigger dpadRight = new Trigger(() -> {return m_controller.getDpadRight();});
  
    //dpad up and dpad right controls left and right climb. press both to move at the same time
    // dpadUp.whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(climbUp)))
    //   .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(0)));
    // dpadRight.whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(climbUp)))
    //   .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(0)));
    // dpadUpRight.whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(climbUp)))
    //   .whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(climbUp)))
    //   .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(0)))
    //   .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(0)));   

    // dpadDown.whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(climbDown)))
    //   .whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(climbDown)))
    //   .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(0)))
    //   .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(0)));   
    // dpadLeft.whenActive(new InstantCommand(() -> ClimbSubsystem.getInstance().pivotPower(pivotDown)))
    //   .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().pivotPower(0)));

    dpadUp.whileActiveOnce(new SetTurretCommand(true));
    dpadDown.whileActiveOnce(new SetTurretCommand(false));
    dpadRight.whileActiveOnce(new TurretAlign());

    m_controller.getStartButton().whenPressed(m_drivetrainSubsystem::resetOdometry);// resets to 0 -> for testing only
    m_controller.getBackButton().whenPressed(m_drivetrainSubsystem::resetOdometry);// resets to 0 -> for testing only
    m_controller.getXButton().whenActive(new SetIntakeIndexerCommand(intakeOn, indexerUp));//right bumper hold
    m_controller.getXButton().whenInactive(new SetIntakeIndexerCommand(0, 0));//right bumper release
    m_controller.getBButton().whenActive(new SetIntakeIndexerCommand(intakeReverse, indexerDown));//right bumper hold
    m_controller.getBButton().whenInactive(new SetIntakeIndexerCommand(0, 0));//right bumper release

    m_controller.getAButton().whenActive(new RobotIdle());
    m_controller.getYButton().whenActive(new RobotOff());

    m_controller.getLeftBumper().whenHeld(new SetIntakeCommand(intakeReverse, false));
    m_controller.getLeftBumper().whenReleased(new SetIntakeCommand(intakeOn, true));
    //m_controller.getRightBumper().whenHeld(new ShootByLimelight(false));
    m_controller.getRightBumper().whenHeld(new SetIntakeCommand(intakeOn, false));
    
    //m_controller.getRightStickButton().whenHeld(new ShootByLimelight(false));
    m_controller.getLeftStickButton().whenHeld(new AimShoot());
    
    Trigger leftTriggerAxis = new Trigger(() -> { return m_controller.getLeftTriggerAxis() > triggerDeadzone;});
    Trigger rightTriggerAxis = new Trigger(() -> { return m_controller.getRightTriggerAxis() > triggerDeadzone;});

    //leftTriggerAxis.whileActiveOnce(new ShootByLimelight(true));
    //leftTriggerAxis.whileActiveOnce(new AimShoot());
    //leftTriggerAxis.whenInactive(new SequentialCommandGroup(new WaitCommand(0.6), new RobotIdle()));
    leftTriggerAxis.whileActiveOnce(new EjectBall());
    rightTriggerAxis.whileActiveOnce(new AimShoot());


    //SECOND CONTROLLER

    //DPAD
    Trigger dpadUp2 = new Trigger(() -> {return m_controller2.getDpadUp();});
    Trigger dpadDown2 = new Trigger(() -> {return m_controller2.getDpadDown();});    
    Trigger dpadLeft2 = new Trigger(() -> {return m_controller2.getDpadLeft();});
    Trigger dpadRight2 = new Trigger(() -> {return m_controller2.getDpadRight();});

    /*dpadUp2.whileActiveContinuous(new InstantCommand(() -> m_shooterSubsystem.increaseShooterVelocity(250)));  //works  
    dpadDown2.whileActiveContinuous(new InstantCommand(() -> m_shooterSubsystem.increaseShooterVelocity(-250)));*/   //works
    dpadRight2.whenActive(new InstantCommand(() -> m_shooterSubsystem.setShooterVelocity(shooterFixed)));
    dpadLeft2.whenActive(new InstantCommand(() -> m_shooterSubsystem.setShooterVelocity(3000)));
    dpadDown2.whenActive(new SetIndexerCommand(indexerDown,false))
        .whenInactive(new SetIndexerCommand(0.0, false));
    dpadUp2.whenActive(new SetIndexerCommand(indexerUp,false))
        .whenInactive(new SetIndexerCommand(0.0, false));
    
    Trigger leftTriggerAxis2 = new Trigger(() -> { return m_controller2.getLeftTriggerAxis() > triggerDeadzone;});
    Trigger rightTriggerAxis2 = new Trigger(() -> { return m_controller2.getRightTriggerAxis() > triggerDeadzone;});

    /*leftTriggerAxis2.whenActive(new SetIndexerCommand(indexerDown,false))
                    .whenInactive(new SetIndexerCommand(0.0, false));
    rightTriggerAxis2.whenActive(new SetIndexerCommand(indexerUp,false))
                     .whenInactive(new SetIndexerCommand(0.0, false));*/
    m_controller2.getLeftBumper().whenPressed(new SetIntakeCommand(intakeOn,true));
    m_controller2.getRightBumper().whenPressed(new SetIndexerCommand(indexerUp,true));
                     
    leftTriggerAxis2.whileActiveContinuous(new InstantCommand(() -> ClimbSubsystem.getInstance().leftPivotPower(pivotDown*m_controller2.getLeftTriggerAxis())))
      .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().leftPivotPower(0)));
    rightTriggerAxis2.whileActiveContinuous(new InstantCommand(() -> ClimbSubsystem.getInstance().rightPivotPower(pivotDown*m_controller2.getRightTriggerAxis())))
      .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().rightPivotPower(0)));                    
                    
    Trigger leftJoyStick = new Trigger(() -> { return Math.abs(m_controller2.getLeftY()) > triggerDeadzone;});
    Trigger rightJoystick = new Trigger(() -> { return Math.abs(m_controller2.getRightY()) > triggerDeadzone;});
    leftJoyStick.whileActiveContinuous(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(-climbUp*m_controller2.getLeftY())))
      .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().leftClimbPower(0)));
    rightJoystick.whileActiveContinuous(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(-climbUp*m_controller2.getRightY())))
      .whenInactive(new InstantCommand(() -> ClimbSubsystem.getInstance().rightClimbPower(0)));

    
    m_controller2.getXButton().whenHeld(new SetIntakeCommand(intakeOn,false)).whenReleased(new SetIntakeCommand(0.0, false));
    m_controller2.getBButton().whenHeld(new SetIntakeCommand(intakeReverse,false)).whenReleased(new SetIntakeCommand(0.0, false));
   // m_controller2.getAButton().whenActive(new RobotIdle());
    // m_controller2.getAButton().whenActive(new InstantCommand(() -> PivotSubsystem.getInstance().deployIntake()));
    m_controller2.getAButton().whenActive(new ConditionalCommand(
      new InstantCommand(() -> PivotSubsystem.getInstance().retractIntake()), 
      new InstantCommand(() -> PivotSubsystem.getInstance().deployIntake()), 
      PivotSubsystem.getInstance().intakeState));
    // .whenActive(new SetIntakeCommand(intakeOn, true))
    // .whenActive(new SetIndexerCommand(0, true))
    // .whenInactive(new SetIntakeCommand(0, false))  
    // .whenInactive(new SetIndexerCommand(0, false))
    // .whenInactive(new InstantCommand(() -> PivotSubsystem.getInstance().moveToPosition(0)));
    m_controller2.getYButton().whenActive(new RobotOff());    
    m_controller2.getStartButton().whenPressed(m_drivetrainSubsystem::resetOdometry);
    m_controller2.getBackButton().whenPressed(m_drivetrainSubsystem::resetOdometry);
  }

  public void onRobotDisabled() {
    //called when robot is disabled. Set all subsytems to 0
    IndexerSubsystem.getInstance().setIntakePercentPower(0.0, false);
    IndexerSubsystem.getInstance().setIndexerPercentPower(0.0, false);
    ShooterSubsystem.getInstance().setShooterVelocity(0);
  }
}
