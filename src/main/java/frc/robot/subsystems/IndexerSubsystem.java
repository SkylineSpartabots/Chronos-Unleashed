package frc.robot.subsystems;

import java.time.Period;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.TalonFXFactory;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Ports;
import frc.robot.commands.SetSubsystemCommand.SetShooterCommand;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IndexerSubsystem extends SubsystemBase {
    private final CANSparkMax m_IndexerMotor;

    private static final I2C.Port onboardI2C = I2C.Port.kOnboard;
    private static ColorSensorV3 m_intakeSensor;
    private final CANSparkMax m_IntakeMotor;
    private final Solenoid m_Solenoid;
    private final Compressor m_Compressor;

    private static IndexerSubsystem instance = null;

    public static IndexerSubsystem getInstance() {
        if (instance == null) {
            instance = new IndexerSubsystem();
        }
        return instance;
    }

    public IndexerSubsystem() {
        m_IndexerMotor = new CANSparkMax(Constants.Ports.NEO_INDEXER, MotorType.kBrushless);

        // m_IndexerMotor = TalonFXFactory.createDefaultFalcon("Indexer Motor",
        // Ports.INDEXER_MOTOR);//creates motor
        // m_IndexerMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        // m_IndexerMotor.enableVoltageCompensation(true);
        // m_IndexerMotor.setNeutralMode(NeutralMode.Brake);
        // m_IndexerMotor.config_kP(0, 0.4);
        // m_IndexerMotor.config_kI(0, 0);
        // m_IndexerMotor.config_kD(0, 0);

        // !Old intake with Falcon 500 Motor
        // m_IntakeMotor = TalonFXFactory.createDefaultFalcon("Intake Motor",
        // Ports.INTAKE_MOTOR);//creates motor
        // m_IntakeMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs);
        // m_IntakeMotor.enableVoltageCompensation(true);
        // m_IntakeMotor.setNeutralMode(NeutralMode.Coast);

        // !Chronos Upgraded 4 Bar Neo550 Intake Motor
        m_IntakeMotor = new CANSparkMax(Constants.Ports.INTAKE_MOTOR, MotorType.kBrushless);
        m_IntakeMotor.restoreFactoryDefaults();

        // !Chronos Upgraded 4 Bar Pneuamtic
        m_Solenoid = new Solenoid(
                Constants.Ports.PNEUMATIC_HUB,
                PneumaticsModuleType.REVPH,
                Constants.Ports.INTAKE_SOLENOID_CHANNEL);

        m_Compressor = new Compressor(Constants.Ports.PNEUMATIC_HUB, PneumaticsModuleType.REVPH);
        m_Compressor.enableDigital();

        m_intakeSensor = new ColorSensorV3(onboardI2C);

        m_colorMatcherIntake.addColorMatch(Constants.kColorSensorBlueIntake);
        m_colorMatcherIntake.addColorMatch(Constants.kColorSensorRedIntake);

        setMultipleStatuFramePeriod();
    }

    private double IntakeMotorSpeed = 1000;

    public void automaticIntaking() {
        m_IntakeMotor.set(Math.abs(Constants.intakeOn * IntakeMotorSpeed));
        automaticIndexing = true;
        ready = true;
    }

    public void setIntakePercentPower(double power) {
        m_IntakeMotor.set(power * IntakeMotorSpeed);
    }

    public void setIntakeDeploymentState(boolean state) {
        m_Solenoid.set(state);
    }

    private double neoSpeed = 1000;

    public void setIndexerPercentPower(double power) {
        if (Math.abs(power) < 0.001 && DriverStation.isTeleop())
            automaticIndexing = true;
        else
            automaticIndexing = false;
        if (power == 0) {
            m_IndexerMotor.set(0);
        } else {
            m_IndexerMotor.set(neoSpeed);
        }
        ballCount = 0;
    }

    public void fire() {
        ballCount = 0;
        bottomBallIsWrong = false;
        setIndexerPercentPower(Constants.indexerUp);
    }

    private final int upAmount = 9000;
    private final int ejectAmount = upAmount * 5;

    public void moveUpIndexer(int amount) {
        // m_IntakeMotor.set(ControlMode.PercentOutput, Constants.intakeOn);
        // m_IndexerMotor.setPositi(ControlMode.Position, m_IndexerMotor + amount);
    }

    private int ballCount = 0;
    private boolean bottomBallIsWrong = false;
    private boolean ready = true;
    private boolean automaticIndexing = true;

    @Override
    public void periodic() {
        // if(ready && automaticIndexing && ballCount <=2&& isIntakeBallLoaded()){ not
        // using for outreach + neos
        // ready = false;
        // ballCount++;
        // new SequentialCommandGroup(
        // new WaitCommand(0.2),
        // new InstantCommand(() -> ballIn())
        // ).schedule();
        // }

        SmartDashboard.putNumber("Ball Count", ballCount);
        SmartDashboard.putNumber("indexer position", m_IndexerMotor.getEncoder().getPosition());

        SmartDashboard.putNumber("intake proximity", m_intakeSensor.getProximity());
        // SmartDashboard.putBoolean("intake loaded", isIntakeBallLoaded());
        // SmartDashboard.putBoolean("intake autoIntake", autoIntake);
        // SmartDashboard.putNumber("Intake speed",
        // m_IntakeMotor.getSelectedSensorVelocity());
        // SmartDashboard.putNumber("Intake Voltage",
        // m_IntakeMotor.getMotorOutputVoltage());
        // SmartDashboard.putNumber("Intake Output Current",
        // m_IntakeMotor.getStatorCurrent());
        // SmartDashboard.putNumber("Intake Input Current",
        // m_IntakeMotor.getSupplyCurrent());
        SmartDashboard.putNumber("Intake Input Current", m_IntakeMotor.getAppliedOutput());

        // SmartDashboard.putBoolean("indexer autoIntake", autoIndexer);
        // SmartDashboard.putNumber("indexer speed",
        // m_IndexerMotor.getSelectedSensorVelocity());
        // SmartDashboard.putNumber("indexer Voltage",
        // m_IndexerMotor.getMotorOutputVoltage());
        // SmartDashboard.putNumber("indexer Output Current",
        // m_IndexerMotor.getStatorCurrent());
        SmartDashboard.putNumber("indexer Input Current", m_IndexerMotor.getAppliedOutput());
        SmartDashboard.putBoolean("Color correct", colorRight());
        SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().toString());
        // SmartDashboard.putNumber("Indexer current from PDP",
        // RobotContainer.getPDP().getCurrent(16));
        // SmartDashboard.putNumber("Intake current from PDP",
        // RobotContainer.getPDP().getCurrent(3));

    }

    public void ballIn() {
        if (isCorrectColor()) {
            moveUpIndexer(upAmount);
            new SequentialCommandGroup(new WaitCommand(0.5), new InstantCommand(() -> ready = true)).schedule();

            if (ballCount >= 2)
                setIntakePercentPower(0);
        } else {
            ballCount = 0;
            // if(ballCount==0){
            new SequentialCommandGroup(
                    new InstantCommand(() -> setIndexerPercentPower(Constants.indexerUp)),
                    new SetShooterCommand(0.3),
                    new WaitCommand(1.0),
                    new InstantCommand(() -> setIndexerPercentPower(0)),
                    new InstantCommand(() -> ready = true),
                    new InstantCommand(() -> ShooterSubsystem.getInstance().setShooterVelocity(Constants.shooterIdle)))
                    .schedule();
            // }
            // else{
            // bottomBallIsWrong = true;
            // }
        }
    }

    public boolean useColorSort = false;
    private static final ColorMatch m_colorMatcherIntake = new ColorMatch();

    private boolean isCorrectColor() {
        return true;
        // if(!DriverStation.isTeleop() || !useColorSort) return true;
        // Color c =
        // m_colorMatcherIntake.matchClosestColor(m_intakeSensor.getColor()).color;
        // return
        // c.equals(DriverStation.getAlliance().equals(Alliance.Red)?Constants.kColorSensorRedIntake:Constants.kColorSensorBlueIntake);
    }

    private boolean colorRight() {
        // return true;
        Color c = m_colorMatcherIntake.matchClosestColor(m_intakeSensor.getColor()).color;
        return c.equals(DriverStation.getAlliance().equals(Alliance.Red) ? Constants.kColorSensorRedIntake
                : Constants.kColorSensorBlueIntake);
    }

    public boolean isIntakeBallLoaded() {
        return m_intakeSensor.getProximity() >= Constants.kColorSensorLoadingDistance;
    }

    // first ball position
    // second ball
    // move second ball to first ball to shoot
    // eject first ball
    // eject second ball

    // eject both balls

    // start intaking

    // manual overrides for both

    private void setMultipleStatuFramePeriod() {
        m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        // m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);

        // m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);//if
        // rev, 4500
        // m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        // m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat,
        // 253);//4750
        // m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 253);
        // m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_6_Misc, 251);//5000
        // m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_6_Misc, 251);
        // m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_7_CommStatus,
        // 249);//5250
        // m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 249);
        // m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic,
        // 247);//5500
        // m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 247);
        // m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,
        // 245);//5750
        // m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 245);
        // m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,
        // 243);//5850
        // m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 243);
        // m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1,
        // 241);//5950
        // m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 241);
        // m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus,
        // 239);//6100
        // m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus,
        // 239);
        // m_IndexerMotor.setStatusFramePeriod(StatusFrame.Status_17_Targets1,
        // 237);//6250
        m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 237);
    }

}
