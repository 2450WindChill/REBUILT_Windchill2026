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

//import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class IndexSubsystem extends SubsystemBase {

  public final SparkFlex indexMotor = new SparkFlex(Constants.SHOOTER_INDEXER_ID, MotorType.kBrushless);

  /** Creates a new ExampleSubsystem. */
  public IndexSubsystem() {
    SparkFlexConfig indexConfig = new SparkFlexConfig();
    indexConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(70);
    indexMotor.configure(indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  

  public void spinIndex() {
    indexMotor.setVoltage(Constants.LOAD_INDEX_VOLTAGE);
  }

  public void reverseIndex() {
    indexMotor.setVoltage(Constants.REVERSE_INDEX_VOLTAGE);
  }

  public void reverseIndexFullSpeed() {
    indexMotor.set(1);
  }


  public void stopIndex() {
    indexMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var instance = NetworkTableInstance.getDefault();
    instance.getTable("Indexer");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


}
