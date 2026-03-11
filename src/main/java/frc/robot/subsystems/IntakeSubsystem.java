//TODO: Command with a button that spins the wheels when pressed or held down 

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private static final double GEAR_RATIO = 20;
  private static final double MOTOR_ROT_PER_DEG = GEAR_RATIO / 360;

  private static final double STOW_DEG = 20;
  private static final double DEPLOY_DEG = 60;
  private static final MotionMagicVoltage mm = new MotionMagicVoltage(0.0);
  public final SparkFlex intakeSpinMotor;
  public final TalonFX intakePivotMotor;
  public final PositionVoltage p_req = new PositionVoltage(Constants.INTAKE_DOWN_ROTATIONS);

  public IntakeSubsystem() {
    intakeSpinMotor = new SparkFlex(Constants.INTAKE_MOTOR_SPIN_ID, MotorType.kBrushless);
    SparkFlexConfig spinConfig = new SparkFlexConfig();
    spinConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(70);
    intakeSpinMotor.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakePivotMotor = new TalonFX(Constants.INTAKE_MOTOR_PIVOT_ID);
    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 35;
    pivotConfig.CurrentLimits.SupplyCurrentLowerLimit = 60;
    pivotConfig.CurrentLimits.SupplyCurrentLowerLimit = .2;
    pivotConfig.Slot0.kP = 1; // 1
    pivotConfig.Slot0.kI = 1.2; // 1.2
    pivotConfig.Slot0.kD = 0; // 0
    pivotConfig.Slot0.kS = .15; // .15
    pivotConfig.Slot0.kV = 0; // 0
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    // pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    // degToMotorRot(Constants.PIVOT_MAX_DEGREES);
    // pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    // degToMotorRot(Constants.PIVOT_MIN_DEGREES);
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 5;
    pivotConfig.MotionMagic.MotionMagicAcceleration = 10;
    pivotConfig.MotionMagic.MotionMagicJerk = 100;

    MotorOutputConfigs pivOutConfigs = new MotorOutputConfigs();
    pivOutConfigs.PeakForwardDutyCycle = .01;
    pivOutConfigs.PeakForwardDutyCycle = -.01;
    intakePivotMotor.getConfigurator().apply(pivotConfig);
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

  public void intake() {
    intakeSpinMotor.set(Constants.INTAKE_SPEED);
  }

  public void outTake() {
    intakeSpinMotor.set(Constants.OUTAKE_SPEED);
  }

  public void stopIntake() {
    intakeSpinMotor.stopMotor();
  }

  public void retract() {
    setPosition(Constants.INTAKE_UP_ROTATIONS);
  }

  public void deploy() {
    setPosition(Constants.INTAKE_DOWN_ROTATIONS);
  }

  public void setPosition(double rotations) {
    var request = new MotionMagicVoltage(0).withSlot(0);
    intakePivotMotor.setControl(request.withPosition(rotations));
  }

  public void setPivotDeg(double deg) {
    // double clamped = MathUtil.clamp(deg, Constants.PIVOT_MIN_DEGREES,
    // Constants.PIVOT_MAX_DEGREES);
    // intakePivotMotor.setControl(mm.withPosition(clamped));
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
    SmartDashboard.putNumber("intakeAnglePosition", intakePivotMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private static double degToMotorRot(double degrees) {
    return degrees * MOTOR_ROT_PER_DEG;
  }

}
