//TODO: write functions? idrk how the climb is going to work at all

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public final TalonFX climbMotor;
  public final PositionVoltage p_req = new PositionVoltage(Constants.INTAKE_DOWN_ROTATIONS);

  public ClimberSubsystem() {

    climbMotor = new TalonFX(Constants.CLIMB_MOTOR_ID);
    var climbConfig = new TalonFXConfiguration();
    climbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climbConfig.CurrentLimits.SupplyCurrentLimit = 35;
    climbConfig.CurrentLimits.SupplyCurrentLowerLimit = 60;
    climbConfig.CurrentLimits.SupplyCurrentLowerLimit = .2;
    climbConfig.Slot0.kP = 1; // 1
    climbConfig.Slot0.kI = 1.2; // 1.2
    climbConfig.Slot0.kD = 0; // 0
    climbConfig.Slot0.kS = .15; // .15
    climbConfig.Slot0.kV = 0; // 0
    climbConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    climbConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    // pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    // degToMotorRot(Constants.PIVOT_MAX_DEGREES);
    // pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    // degToMotorRot(Constants.PIVOT_MIN_DEGREES);
    climbConfig.MotionMagic.MotionMagicCruiseVelocity = 5;
    climbConfig.MotionMagic.MotionMagicAcceleration = 10;
    climbConfig.MotionMagic.MotionMagicJerk = 100;
    climbMotor.getConfigurator().apply(climbConfig);
    climbMotor.setNeutralMode(NeutralModeValue.Brake);

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

  public void retract() {
    setPosition(Constants.CLIMBER_RETRACT_ROTATIONS);
  }

  public void extend() {
    setPosition(Constants.CLIMBER_EXTEND_ROTATIONS);
  }

  public void setPosition(double rotations) {
    var request = new MotionMagicVoltage(0).withSlot(0);
    climbMotor.setControl(request.withPosition(rotations));
    
  }
public void manualMove(double voltage){
  climbMotor.setVoltage(voltage);
}
  @Override
  public void periodic() {
    SmartDashboard.putNumber("climberPosition", climbMotor.getPosition().getValueAsDouble());
  }

  // This method will be called once per scheduler run

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}