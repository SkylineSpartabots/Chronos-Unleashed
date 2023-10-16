// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

//holds robot-wide constants
public final class Constants {
        public static final double stickDeadband = 0.15;
        // timeout for CAN commands and error checking
        public static final int kTimeOutMs = 10;
        public static final double kMinimumBatteryVoltage = 12;
        public static final int kCanDeviceCount = 0;

        // public static final double shooterFixed = 10500;
        // public static final double shooterRamped = 11000;
        // public static final double shooterIdle = 8000;
        // public static final double shooterEjection = 8000;
        public static final double shooterFixed = 12500;
        public static final double shooterRamped = 13000;
        public static final double shooterIdle = 9000;
        public static final double shooterEjection = 9000;

        public static final double indexerUp = 0.80;
        public static final double indexerDown = -0.5;

        public static final double intakeOn = 0.9;
        public static final double intakeReverse = -0.5;

        public static final double waitBetweenShots = 0.25;

        public static final double climbUp = 0.99;
        public static final double climbDown = -0.99; // usually 0.8, but potential issues having joystick moved to a
                                                      // full position
        public static final double pivotUp = -0.4;
        public static final double pivotDown = 0.4;

        public static final Translation2d targetHudPosition = new Translation2d(8.23, 4.165);

        public static final Color kColorSensorBlueIntake = new Color(0.19, 0.43, 0.37);
        public static final Color kColorSensorRedIntake = new Color(0.44, 0.38, 0.16);

        public static final Color allianceColorIntake = kColorSensorBlueIntake;

        public static final double kColorSensorLoadingDistance = 70;

        // Dummy values, need to find/calculate
        public static final List<Translation2d> kReferenceTranslations = List.of(
                        new Translation2d(1, 0),
                        new Translation2d(3, 3));

        public static final class DriveConstants {
                public static final double kTrackWidth = 0.4953;
                // Distance between centers of right and left wheels on robot
                public static final double kWheelBase = 0.4953;
                // Distance between front and back wheels on robot
                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

                public static final boolean kGyroReversed = false;
                // Calculated via SysId
                public static final double ksVolts = 0.74397; // before 3/1: 70541, 33259, 016433
                public static final double kvVoltSecondsPerMeter = 0.33778;
                public static final double kaVoltSecondsSquaredPerMeter = 0.016934;
                // Tuned to taste for desired max velocity
                public static final double kVelocityGain = 6;
                // The maximum voltage that will be delivered to the drive motors.
                // This can be reduced to cap the robot's maximum speed. Typically, this is
                // useful during initial testing of the robot.
                public static final double kMaxVoltage = 12.0;
                // The maximum velocity of the robot in meters per second.
                // This is a measure of how fast the robot should be able to drive in a straight
                // line.
                public static final double kMaxSpeedMetersPerSecond = 6380.0 / 60.0 *
                                SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                                SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
                // need measure on robot
                public static final double kMaxAccelerationMetersPerSecondSquared = 10;
                // The maximum angular velocity of the robot in radians per second.
                // This is a measure of how fast the robot can rotate in place.
                // Here we calculate the theoretical maximum angular velocity. You can also
                // replace this with a measured amount.
                public static final double kMaxAngularSpeedRadiansPerSecond = kMaxSpeedMetersPerSecond /
                                Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);

                public static final double kMaxAngularSpeedRadiansPerSecondSquared = kMaxAngularSpeedRadiansPerSecond;

                public static final double kpRotation = 0.1;
                public static final double kiRotation = 0.0;
                public static final double kdRotation = 0;
                public static final TrapezoidProfile.Constraints kRotationConstraints = new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        }

        public static final class Ports {
                public static final int NEO_INDEXER = 50;
                public static final int NEO_PIVOT = 23;

                // public static final int FRONT_LEFT_DRIVE = 2;
                // public static final int FRONT_LEFT_STEER = 1;
                // public static final int FRONT_LEFT_STEER_ENCODER = 9;
                // public static final double FRONT_LEFT_OFFSET =
                // -Math.toRadians(81.3812255859375);

                // public static final int FRONT_RIGHT_DRIVE = 8;
                // public static final int FRONT_RIGHT_STEER = 7;
                // public static final int FRONT_RIGHT_STEER_ENCODER = 12;
                // public static final double FRONT_RIGHT_OFFSET =
                // -Math.toRadians(303.48358154296875);

                // public static final int BACK_LEFT_DRIVE = 4;
                // public static final int BACK_LEFT_STEER = 3;
                // public static final int BACK_LEFT_STEER_ENCODER = 10;
                // public static final double BACK_LEFT_OFFSET =
                // -Math.toRadians(349.26910400390625);

                // public static final int BACK_RIGHT_DRIVE = 6;
                // public static final int BACK_RIGHT_STEER = 5;
                // public static final int BACK_RIGHT_STEER_ENCODER = 11;
                // public static final double BACK_RIGHT_OFFSET =
                // -Math.toRadians(156.5277099609375);

                public static final int MASTER_SHOOTER_MOTOR = 21;
                public static final int FOLLOW_SHOOTER_MOTOR = 22;
                public static final int INTAKE_MOTOR = 31;
                public static final int INDEXER_MOTOR = 32;
                public static final int PIVOT_MOTOR = 23;

                public static final int LEFT_CLIMB = 41;
                public static final int RIGHT_CLIMB = 42;
                public static final int LEFT_PIVOT = 44;
                public static final int RIGHT_PIVOT = 43;

                // !Subject to Change
                public static final int INTAKE_SOLENOID_CHANNEL = 0;

                // !Subject to Change
                public static final int PNEUMATIC_HUB = 16;
        }

        public static final class SwerveConstants {

                public static final double swervePower = .80;

                // public static final int pigeonID = 15;

                public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants
                                .SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3);

                /* Drivetrain Constants */
                public static final double trackWidth = 0.5715;
                public static final double wheelBase = 0.5715;
                public static final double wheelCircumference = chosenModule.wheelCircumference;
                /*
                 * Swerve Kinematics
                 * No need to ever change this unless you are not doing a traditional
                 * rectangular/square 4 module swerve
                 */
                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

                /* Module Gear Ratios */
                public static final double driveGearRatio = chosenModule.driveGearRatio;
                public static final double angleGearRatio = chosenModule.angleGearRatio;

                /* Motor Inverts */
                public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
                public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

                /* Angle Encoder Invert */
                public static final boolean canCoderInvert = chosenModule.canCoderInvert;

                /* Swerve Current Limiting */ // DID NOT CHANGE
                public static final int angleContinuousCurrentLimit = 25;
                public static final int anglePeakCurrentLimit = 40;
                public static final double anglePeakCurrentDuration = 0.1;
                public static final boolean angleEnableCurrentLimit = true;

                public static final int driveContinuousCurrentLimit = 35;
                public static final int drivePeakCurrentLimit = 60;
                public static final double drivePeakCurrentDuration = 0.1;
                public static final boolean driveEnableCurrentLimit = true;

                /*
                 * These values are used by the drive falcon to ramp in open loop and closed
                 * loop driving.
                 * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
                 */ // DID NOT CHANGE
                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;

                /* Angle Motor PID Values */
                public static final double angleKP = chosenModule.angleKP;
                public static final double angleKI = chosenModule.angleKI;
                public static final double angleKD = chosenModule.angleKD;
                public static final double angleKF = chosenModule.angleKF;

                /* Drive Motor PID Values */
                public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
                public static final double driveKI = 0.0;
                public static final double driveKD = 0.0;
                public static final double driveKF = 0.0;

                /*
                 * Drive Motor Characterization Values
                 * Divide SYSID values by 12 to convert from volts to percent output for CTRE
                 */
                public static final double driveKS = (0.15932 / 12); // 0.23217 // 0.17387
                public static final double driveKV = (2.3349 / 12); // 2.2688 // 2.2365
                public static final double driveKA = (0.47337 / 12); // 0.42472 // 1.2552

                /* Swerve Profiling Values */
                /** Meters per Second */
                public static final double maxSpeed = 4.5;
                /** Radians per Second */
                public static final double maxAngularVelocity = 7.0;

                /* Neutral Modes */
                public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
                public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

                /* Module Specific Constants */
                /* Front Left Module - Module 0 */
                public static final class Mod0 {
                        public static final int driveMotorID = 2;
                        public static final int angleMotorID = 1;
                        public static final int canCoderID = 9;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(356.66);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }

                /* Front Right Module - Module 1 */
                public static final class Mod1 {
                        public static final int driveMotorID = 8;
                        public static final int angleMotorID = 7;
                        public static final int canCoderID = 12;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(326.16);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }

                /* Back Left Module - Module 2 */
                public static final class Mod2 {
                        public static final int driveMotorID = 4;
                        public static final int angleMotorID = 3;
                        public static final int canCoderID = 10;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(70.31);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }

                /* Back Right Module - Module 3 */
                public static final class Mod3 {
                        public static final int driveMotorID = 6;
                        public static final int angleMotorID = 5;
                        public static final int canCoderID = 11;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(295.83);
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        canCoderID, angleOffset);
                }
        }
}
