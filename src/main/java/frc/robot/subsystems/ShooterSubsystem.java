// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.ModuleLayer.Controller;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  public final SparkFlex leaderMotor = new SparkFlex(Constants.SHOOTER_ONE_MOTOR_ID, MotorType.kBrushless);
  public final SparkFlex followerMotor = new SparkFlex(Constants.SHOOTER_TWO_MOTOR_ID, MotorType.kBrushless);
  public final SparkClosedLoopController leaderController = leaderMotor.getClosedLoopController();

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    SparkFlexConfig leaderConfig = new SparkFlexConfig();
    leaderConfig
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12)
        .smartCurrentLimit(70);
    leaderConfig.closedLoop
        .p(0.0002)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
    leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig followerConfig = new SparkFlexConfig();
    followerConfig
        .idleMode(IdleMode.kCoast)
        .voltageCompensation(12)
        .smartCurrentLimit(70).follow(leaderMotor, false);
    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void spinShoot() {
    // leaderMotor.setVoltage(Constants.SHOOTER_VOLTAGE);
    leaderController.setSetpoint(Constants.TARGET_SHOOTER_RPM, ControlType.kVelocity);
    // leaderMotor.set(-.8);

  }

  public void loadShoot() {
    leaderMotor.setVoltage(Constants.LOAD_SHOOTER_VOLTAGE);
  }

  public void stopShoot() {
    leaderMotor.setVoltage(0);

  }

  // public Command shootWhileHeld() {
  //   // return Commands.sequence(
  //   // this.runOnce(() -> spinShoot()),
  //   // Commands.waitUntil(this::atSpeed),
  //   // Commands.startEnd(this::spinIndex, this::stopIndex, this))
  //   // .finallyDo(interupted -> {
  //   // stopIndex();
  //   // stopShoot();
  //   // });
  //   Trigger readyToFeed = new Trigger(this::atSpeed).debounce(0.1);
  //     return Commands.startEnd(this::spinShoot, this::stopShoot, this)
  //       .alongWith(
  //           Commands.startEnd(this::spinIndex, this::stopIndex, this)
  //               .onlyWhile(readyToFeed));
  // }

  // public Command loadWhileHeld() {
  //   return Commands.sequence(
  //       this.runOnce(() -> loadShoot()),
  //       Commands.startEnd(this::spinIndex, this::stopIndex, this))
  //       .finallyDo(interupted -> {
  //         stopIndex();
  //         stopShoot();
  //       });
  // }

  public boolean atSpeed() {
    double shooter_rpm = leaderMotor.getEncoder().getVelocity();
    boolean atSpeed = Math.abs(3400 - shooter_rpm) <= Constants.RPM_TOLERANCE;

    return atSpeed;
  }

  public boolean atLoadSpeed() {
    double shooter_rpm = leaderMotor.getEncoder().getVelocity();
    boolean atLoadSpeed = Math.abs(shooter_rpm - 3400) <= Constants.RPM_TOLERANCE;
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
    var instance = NetworkTableInstance.getDefault();
    instance.getTable("Indexer");
    double shooter_rpm = leaderMotor.getEncoder().getVelocity();

    SmartDashboard.putNumber("shooter_rpm", shooter_rpm);
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
