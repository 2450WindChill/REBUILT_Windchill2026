// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

//import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  public final SparkFlex leaderMotor = new SparkFlex(Constants.SHOOTER_ONE_MOTOR_ID, MotorType.kBrushless);
  public final SparkFlex followerMotor = new SparkFlex(Constants.SHOOTER_TWO_MOTOR_ID, MotorType.kBrushless);
  public final SparkFlex indexMotor = new SparkFlex(Constants.SHOOTER_INDEXER_ID, MotorType.kBrushless);

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    SparkFlexConfig leaderConfig = new SparkFlexConfig();
    leaderConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(70);
    leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig followerConfig = new SparkFlexConfig();
    followerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(70).follow(leaderMotor, true);
    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkFlexConfig indexConfig = new SparkFlexConfig();
    indexConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(70);
    indexMotor.configure(indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void spinShoot() {
    leaderMotor.setVoltage(Constants.SHOOTER_VOLTAGE);
  }

  public void loadShoot() {
    leaderMotor.setVoltage(Constants.LOAD_SHOOTER_VOLTAGE);
  }

  public void spinIndex() {
    indexMotor.setVoltage(Constants.INDEX_VOLTAGE);
    System.out.println("TEST IS IT GETTING HERE!@@#$@#!#@@!#@!#@!#@//#endregion");
  }

  public void loadIndex() {
    leaderMotor.setVoltage(Constants.LOAD_INDEX_VOLTAGE);
  }

  public void stopShoot() {
    leaderMotor.setVoltage(0);

  }

  public void stopIndex() {
    indexMotor.setVoltage(0);
    System.out.println("STOPPPPPPPPPPPPPPPPPPPPPppps");

  }

  public Command shootWhileHeld() {
    return Commands.sequence(
        this.runOnce(() -> spinShoot()),
        Commands.waitUntil(this::atSpeed),
        Commands.startEnd(this::spinIndex, this::stopIndex, this))
        .finallyDo(interupted -> {
          stopIndex();
          stopShoot();
        });

  }

  public Command loadWhileHeld() {
    return Commands.sequence(
        this.runOnce(() -> loadShoot()),
        Commands.startEnd(this::loadIndex, this::stopIndex, this))
        .finallyDo(interupted -> {
          stopIndex();
          stopShoot();
        });

  }

  public boolean atSpeed() {
    double shooter_rpm = leaderMotor.getEncoder().getVelocity();
    boolean atSpeed = Math.abs(shooter_rpm - Constants.TARGET_SHOOTER_RPM) <= Constants.RPM_TOLERANCE;
    return atSpeed;
  }

  public boolean atLoadSpeed() {
    double shooter_rpm = leaderMotor.getEncoder().getVelocity();
    boolean atLoadSpeed = Math.abs(shooter_rpm - Constants.TARGET_SHOOTER_RPM) <= Constants.RPM_TOLERANCE;
    return atLoadSpeed;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // public void setSpeedShooterOne(double speed) {
  // leaderMotor.set(speed);
  // }

  // public void setSpeedShooterTwo(double speed) {
  // followerMotor.set(speed);
  // }
}
