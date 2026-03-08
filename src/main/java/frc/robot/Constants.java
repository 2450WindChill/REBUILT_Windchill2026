// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.libs.ModuleConfiguration;
import frc.robot.swerveModules.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public enum SwerveMode {
    NEO,
    KRAKEN
  }

  public static final double SHOOTER_SPEED = 0.75;
  public static final double SHOOTER_VOLTAGE = 3;
  public static final double INDEX_VOLTAGE = 3;
  // Motor ids for the shooters. Still need the actual IDs for the consta nts
  public static final int SHOOTER_ONE_MOTOR_ID = 31;
  public static final int SHOOTER_TWO_MOTOR_ID = 32;
  public static final int SHOOTER_THREE_MOTOR_ID = 33;
  public static final int SHOOTER_FOUR_MOTOR_ID = 34;
  public static final int SHOOTER_INDEXER_ID = 35;
  public static final int INTAKE_MOTOR_PIVOT_ID = 41;
  public static final int INTAKE_MOTOR_SPIN_ID = 42;
  public static final int CLIMB_MOTOR_ID = 51;
  public static final int PIVOT_MAX_DEGREES = 70; // WILL NEED TO CHANGE AND TEST!!!!!!!!!
  public static final int PIVOT_MIN_DEGREES = 10;

  public static final double INTAKE_DOWN_ROTATIONS = 3.0;
  public static final double INTAKE_UP_ROTATIONS = 5.0;
  public static final double CLIMBER_EXTEND_ROTATIONS = 3.0; // don't know numbers
  public static final double CLIMBER_RETRACT_ROTATIONS = 5.0; // don't know numbers

  public static final double TARGET_SHOOTER_RPM = 450; //change later!!!!!!!!!!!!!!!!!!!!!!!!! <----
  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.55;
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.54;

  public static final double stickDeadband = 0.125;

  public static final int pigeonID = 5;
  public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

  /* Drivetrain Constants */
  public static final double trackWidth = Units.inchesToMeters(21.73);
  public static final double wheelBase = Units.inchesToMeters(21.73);
  public static final double wheelDiameter = Units.inchesToMeters(3.0);
  public static final double wheelCircumference = wheelDiameter * Math.PI;

  public static final int elevatorMotorId = 5;
  public static final int endeffectorMotorId = 18;
  public static final int elevatorLowSwitchChannel = 4;
  public static final int elevatorHighSwitchChannel = 5; // TODO: check for conflicts!!
  public static final int beamBreakRecieverChannel = 1;
  public static final int climbMotorOneId = 6;
  public static final int climbMotorTwoId = 7;

  public static final double openLoopRamp = 0.25;
  public static final double closedLoopRamp = 0.0;

  public static final double rotationsPerOneFoot = 0.25;

  public static final double feetToMeters = 0.3048;

  public static final double driveGearRatio = (8.14 / 1.0);
  public static final double angleGearRatio = (150 / 7);

  public static final double driveBaseRadius = (Math.sqrt((trackWidth * trackWidth) + (trackWidth * trackWidth))) / 2;

  public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      // Front Left
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      // Front right
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      // Back left
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      // Back right
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

  /* Swerve Voltage Compensation */
  public static final double voltageComp = 12.0;

  /* Swerve Current Limiting */
  public static final int angleContinuousCurrentLimit = 20;
  public static final int driveContinuousCurrentLimit = 30;

  /* Angle Motor PID Values */
  public static final double angleKP = 0.01;
  public static final double angleKI = 0.0;
  public static final double angleKD = 0.0;
  public static final double angleKFF = 0.0;

  /* Drive Motor PID Values */
  public static final double driveKP = 0.1;
  public static final double driveKI = 0.0;
  public static final double driveKD = 0.0;
  public static final double driveKFF = 0.0;

  /* Drive Motor Conversion Factors */
  public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
  public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
  public static final double angleConversionFactor = 360.0 /* (2 * Math.PI) */ / angleGearRatio;

  /* Swerve Profiling Values */
  // public static final double maxSpeed = 3.6576; // meters per second

  // Max speed is 4.630
  public static final double maxSpeed = 6380 / 60 * ModuleConfiguration.MK4I_L1.getDriveReduction()
      * ModuleConfiguration.MK4I_L1.getWheelDiameter() * Math.PI;

  public static final double maxAngularVelocity = maxSpeed / Math.hypot(wheelBase / 2.0, trackWidth / 2.0);;

  public static final double TICKS_PER_ROTATION = 42;

  /* Neutral Modes */
  // public static final IdleMode angleNeutralMode = IdleMode.kBrake;
  public static final IdleMode angleIdleMode = IdleMode.kBrake;
  public static final IdleMode wristIdleMode = IdleMode.kBrake;
  public static final IdleMode driveIdleMode = IdleMode.kBrake;
  public static final IdleMode elevatorIdleMode = IdleMode.kBrake;

  /* Motor Inverts */

  public static final boolean driveInvert = false;
  public static final boolean angleInvert = true;

  /* Angle Encoder Invert */
  public static final boolean canCoderInvert = false;

  /* Robot centric invert */
  public static final boolean isRobotCentric = false;

  /* Front Left Module - Module 0 */
  public static final class FrontLeftKrakenModule {
    public static final int driveMotorID = 11;
    public static final int angleMotorID = 1;
    public static final int canCoderID = 21;
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID);
  }

  /* Front Right Module - Module 1 */
  public static final class FrontRightKrakenModule {
    public static final int driveMotorID = 13;
    public static final int angleMotorID = 3;
    public static final int canCoderID = 23;
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID);
  }

  /* Back Right Module - Module 3 */
  public static final class BackRightKrakenModule {
    public static final int driveMotorID = 12;
    public static final int angleMotorID = 2;
    public static final int canCoderID = 22;
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID);
  }

  /* Back Left Module - Module 2 */
  public static final class BackLeftKrakenModule {
    public static final int driveMotorID = 14;
    public static final int angleMotorID = 4;
    public static final int canCoderID = 24;
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID);
  }

  /* Front Left Module - Module 0 */
  public static final class FrontLeftNeoModule {
    public static final int driveMotorID = 6;
    public static final int angleMotorID = 5;
    public static final int canCoderID = 11;
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID);
  }

  /* Front Right Module - Module 1 */
  public static final class FrontRightNeoModule {
    public static final int driveMotorID = 8;
    public static final int angleMotorID = 7;
    public static final int canCoderID = 12;
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID);
  }

  /* Back Right Module - Module 3 */
  public static final class BackRightNeoModule {
    public static final int driveMotorID = 4;
    public static final int angleMotorID = 3;
    public static final int canCoderID = 10;
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID);
  }

  /* Back Left Module - Module 2 */
  public static final class BackLeftNeoModule {
    public static final int driveMotorID = 2;
    public static final int angleMotorID = 1;
    public static final int canCoderID = 9;
    public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
        canCoderID);
  }

  public static final double moveToPoseSpeed = 0.05;
  public static final double moveToPoseRotationSpeed = 0.5;
  public static final double RPM_TOLERANCE = 100; //maybe change later

  public static final class AutoConstants {
    public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI,
        2 / Math.PI);
    public static final double THETA_kP = 6.0;
    public static final double THETA_kI = 0.02;
    public static final double THETA_kD = 0.0;

    public static final double X_kP = 5.0;
    public static final double X_kI = 0.0;
    public static final double X_kD = 0.0;

    public static final double Y_kP = 5.0;
    public static final double Y_kI = 0.0;
    public static final double Y_kD = 0.0;
  }
}
