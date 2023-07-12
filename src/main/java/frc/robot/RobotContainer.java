// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.lib.util.Controller;
import frc.lib.util.DeviceFinder;
import frc.robot.commands.*;
import frc.robot.commands.CAS.EjectBall;
// import frc.robot.commands.CAS.MemeShoot;
import frc.robot.commands.CAS.RobotOff;
import frc.robot.commands.SetSubsystemCommand.*;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  private Swerve s_Swerve;
  private IndexerSubsystem m_indexerSubsystem;
  private ShooterSubsystem m_shooterSubsystem;

  /*
   * private static PowerDistribution powerModule = new PowerDistribution(1,
   * ModuleType.kRev);
   * 
   * public static PowerDistribution getPDP(){
   * return powerModule;
   * }
   */
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    s_Swerve = Swerve.getInstance();
    m_indexerSubsystem = IndexerSubsystem.getInstance();
    m_shooterSubsystem = ShooterSubsystem.getInstance();
    s_Swerve.zeroGyro();
    // sets the teleop swerve command as default command with the input from driver
    // joysticks
    // to control the swerve
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis) / 2.75,
            () -> -driver.getRawAxis(strafeAxis) / 2.75,
            () -> -driver.getRawAxis(rotationAxis) / 2.75));

    // Set the scheduler to log Shuffleboard events for command inditialize,
    // interrupt, finish
    /*
     * CommandScheduler.getInstance().onCommandInitialize(command ->
     * Shuffleboard.addEventMarker(
     * "Command initialized", command.getName(), EventImportance.kNormal));
     * CommandScheduler.getInstance().onCommandInterrupt(command ->
     * Shuffleboard.addEventMarker(
     * "Command interrupted", command.getName(), EventImportance.kNormal));
     * CommandScheduler.getInstance().onCommandFinish(command ->
     * Shuffleboard.addEventMarker(
     * "Command finished", command.getName(), EventImportance.kNormal));
     */
    // Configure the button bindings
    configureButtonBindings();

  }

  private static final XboxController driver = new XboxController(0);
  private static final XboxController driver2 = new XboxController(1);

  /* Driver Joysticks (drive control) */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton driverBack = new JoystickButton(driver, XboxController.Button.kBack.value);
  private final JoystickButton driverStart = new JoystickButton(driver, XboxController.Button.kStart.value);
  private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton driverRightBumper = new JoystickButton(driver,
      XboxController.Button.kRightBumper.value);
  private final JoystickButton driverLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  public static XboxController getController() {
    return driver;
  }

  public static void printDiagnostics() {
    /*
     * SmartDashboard.putBoolean("NavX Connected?",
     * Swerve.getInstance().getNavxConnected());
     * SmartDashboard.putBoolean("Limelight Connected?",
     * LimelightSubsystem.getInstance().isConnected());
     * SmartDashboard.putBoolean("Can Bus Connected?", isCanConnected());
     * SmartDashboard.putBoolean("Battery Charged?", isBatteryCharged());
     */
  }

  private static boolean isBatteryCharged() {
    return RobotController.getBatteryVoltage() >= Constants.kMinimumBatteryVoltage;
  }

  private static boolean isCanConnected() {
    return DeviceFinder.Find().size() == Constants.kCanDeviceCount;
  }

  // configures button bindings to controller
  private void configureButtonBindings() {
    final double triggerDeadzone = 0.2;

    // FIRST CONTROLLER

    // DPAD
    // Trigger dpadUp = new Trigger(() -> {return driver.getDpadUp();});
    // Trigger dpadUpRight = new Trigger(() -> {return driver.getDpadUpRight();});
    // Trigger dpadDown = new Trigger(() -> {return driver.getDpadDown();});
    // Trigger dpadLeft = new Trigger(() -> {return driver.getDpadLeft();});
    // Trigger dpadRight = new Trigger(() -> {return driver.getDpadRight();});

    // dpad up and dpad right controls left and right climb. press both to move at
    // the same time
    // dpadUp.onTrue(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().leftClimbPower(climbUp)))
    // .onFalse(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().leftClimbPower(0)));
    // dpadRight.onTrue(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().rightClimbPower(climbUp)))
    // .onFalse(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().rightClimbPower(0)));
    // dpadUpRight.onTrue(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().leftClimbPower(climbUp)))
    // .onTrue(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().rightClimbPower(climbUp)))
    // .onFalse(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().leftClimbPower(0)))
    // .onFalse(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().rightClimbPower(0)));

    // dpadDown.onTrue(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().leftClimbPower(climbDown)))
    // .onTrue(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().rightClimbPower(climbDown)))
    // .onFalse(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().leftClimbPower(0)))
    // .onFalse(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().rightClimbPower(0)));
    // dpadLeft.onTrue(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().pivotPower(pivotDown)))
    // .onFalse(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().pivotPower(0)));

    driverStart.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())));// resets to 0 -> for testing
                                                                                       // only
    driverBack.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())));// resets to 0 -> for testing only
    driverX.onTrue(new SetIntakeIndexerCommand(intakeOn, indexerUp));// right bumper hold
    driverX.onFalse(new SetIntakeIndexerCommand(0, 0));// right bumper release
    driverB.onTrue(new SetIntakeIndexerCommand(intakeReverse, indexerDown));// right bumper hold
    driverB.onFalse(new SetIntakeIndexerCommand(0, 0));// right bumper release
    driverRightBumper.onTrue(new SetIntakeDeploymentState(true));
    driverRightBumper.onFalse(new SetIntakeDeploymentState(false));

    // driver.getAButton().onTrue(new InstantCommand(() ->
    // IndexerSubsystem.getInstance().automaticIntaking()));
    driverA.onTrue(new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(shooterIdle)));
    driverY.onTrue(new RobotOff());

    driverLeftBumper.onTrue(new EjectBall());

    // driver.getLeftBumper().whenHeld(new SetIntakeCommand(intakeReverse, false));
    // driver.getLeftBumper().whenReleased(new SetIntakeCommand(intakeOn, true));
    // driver.getRightBumper().whenHeld(new ShootByLimelight(false));
    // driver.getRightBumper().whileActiveOnce(new MemeShoot());

    // driver.getRightStickButton().whenHeld(new ShootByLimelight(false));
    // driverLeftBumper.onTrue(new AimShoot());

    Trigger leftTriggerAxis = new Trigger(() -> {
      return driver.getLeftTriggerAxis() > triggerDeadzone;
    });
    Trigger rightTriggerAxis = new Trigger(() -> {
      return driver.getRightTriggerAxis() > triggerDeadzone;
    });

    // leftTriggerAxis.whileActiveOnce(new ShootByLimelight(true));
    // leftTriggerAxis.whileActiveOnce(new AimShoot());
    // leftTriggerAxis.onFalse(new SequentialCommandGroup(new WaitCommand(0.6), new
    // RobotIdle()));
    // leftTriggerAxis.whileActiveOnce(new EjectBall());

    // leftTriggerAxis.onTrue(new InstantCommand(() ->
    // PivotSubsystem.getInstance().deployIntake()))
    // .onTrue(new InstantCommand(() ->
    // IndexerSubsystem.getInstance().automaticIntaking()))
    // .onFalse(new SetIntakeCommand(0))
    // .onFalse(new InstantCommand(() ->
    // PivotSubsystem.getInstance().retractIntake()));
    // rightTriggerAxis.toggleOnTrue(new AimShoot());

    // SECOND CONTROLLER

    // //DPAD
    // Trigger dpadUp2 = new Trigger(() -> {return driver2.getDpadUp();});
    // Trigger dpadDown2 = new Trigger(() -> {return driver2.getDpadDown();});
    // Trigger dpadLeft2 = new Trigger(() -> {return driver2.getDpadLeft();});
    // Trigger dpadRight2 = new Trigger(() -> {return driver2.getDpadRight();});

    // /*dpadUp2.whileActiveContinuous(new InstantCommand(() ->
    // m_shooterSubsystem.increaseShooterVelocity(250))); //works
    // dpadDown2.whileActiveContinuous(new InstantCommand(() ->
    // m_shooterSubsystem.increaseShooterVelocity(-250))); //works
    // */
    // //dpadRight2.onTrue(new InstantCommand(() ->
    // m_shooterSubsystem.setShooterVelocity(shooterFixed)));
    // //dpadLeft2.onTrue(new InstantCommand(() ->
    // m_shooterSubsystem.setShooterVelocity(3000)));

    // dpadRight2.onTrue(new InstantCommand(() -> m_indexerSubsystem.useColorSort =
    // true));
    // dpadLeft2.onTrue(new InstantCommand(() -> m_indexerSubsystem.useColorSort =
    // false));

    // dpadDown2.onTrue(new InstantCommand(()
    // ->m_pivotSubsystem.getInstance().resetIntakeDown()));
    // dpadUp2.onTrue(new InstantCommand(()
    // ->m_pivotSubsystem.getInstance().resetIntakeUp()));
    // /*dpadDown2.onTrue(new SetIndexerCommand(indexerDown))
    // .onFalse(new SetIndexerCommand(0.0));
    // dpadUp2.onTrue(new SetIndexerCommand(indexerUp))
    // .onFalse(new SetIndexerCommand(0.0));*/

    // Trigger leftTriggerAxis2 = new Trigger(() -> { return
    // driver2.getLeftTriggerAxis() > triggerDeadzone;});
    // Trigger rightTriggerAxis2 = new Trigger(() -> { return
    // driver2.getRightTriggerAxis() > triggerDeadzone;});

    // /*leftTriggerAxis2.onTrue(new SetIndexerCommand(indexerDown,false))
    // .onFalse(new SetIndexerCommand(0.0, false));
    // rightTriggerAxis2.onTrue(new SetIndexerCommand(indexerUp,false))
    // .onFalse(new SetIndexerCommand(0.0, false));*/
    // //driver2.getLeftBumper().whenPressed(new SetIntakeCommand(intakeOn,true));
    // //driver2.getRightBumper().whenPressed(new
    // SetIndexerCommand(indexerUp,true));

    // leftTriggerAxis2.whileActiveContinuous(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().leftPivotPower(pivotDown*driver2.getLeftTriggerAxis())))
    // .onFalse(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().leftPivotPower(0)));
    // rightTriggerAxis2.whileActiveContinuous(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().rightPivotPower(pivotDown*driver2.getRightTriggerAxis())))
    // .onFalse(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().rightPivotPower(0)));

    // Trigger leftJoyStick = new Trigger(() -> { return
    // Math.abs(driver2.getLeftY()) > triggerDeadzone;});
    // Trigger rightJoystick = new Trigger(() -> { return
    // Math.abs(driver2.getRightY()) > triggerDeadzone;});
    // leftJoyStick.whileActiveContinuous(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().leftClimbPower(-climbUp*driver2.getLeftY())))
    // .onFalse(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().leftClimbPower(0)));
    // rightJoystick.whileActiveContinuous(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().rightClimbPower(-climbUp*driver2.getRightY())))
    // .onFalse(new InstantCommand(() ->
    // ClimbSubsystem.getInstance().rightClimbPower(0)));

    // //driver2.getXButton().whenHeld(new
    // SetIntakeCommand(intakeOn)).whenReleased(new SetIntakeCommand(0.0));
    // driver2.getBButton().whenHeld(new SetIntakeCommand(0)).whenReleased(new
    // InstantCommand(() -> IndexerSubsystem.getInstance().automaticIntaking()));
    // driver2.getAButton().onTrue(new InstantCommand(() ->
    // PivotSubsystem.getInstance().deployIntake()))
    // .onTrue(new InstantCommand(() ->
    // IndexerSubsystem.getInstance().automaticIntaking()))
    // //.onFalse(new SetIntakeCommand(0))
    // .onFalse(new InstantCommand(() ->
    // PivotSubsystem.getInstance().retractIntake()));
    // driver2.getYButton().onTrue(new RobotOff());
    // driver2.getStartButton().whenPressed(m_Swerve::resetOdometry);
    // driver2.getBackButton().whenPressed(m_Swerve::resetOdometry);
  }

  public void onRobotDisabled() {
    // called when robot is disabled. Set all subsytems to 0
    IndexerSubsystem.getInstance().setIntakePercentPower(0.0);
    IndexerSubsystem.getInstance().setIndexerPercentPower(0.0);
    ShooterSubsystem.getInstance().setShooterVelocity(0);
  }

  // right trigger shoot
  // left trigger intake deploy and intake, release, fold back
  // left bumper eject
  // right bumper shooter while moving

}
