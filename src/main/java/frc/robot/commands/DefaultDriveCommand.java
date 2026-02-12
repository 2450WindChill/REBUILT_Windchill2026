// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;

/** An example command that uses an example subsystem. */
public class DefaultDriveCommand extends Command {
  private final DrivetrainSubsystem m_driveTrainSubSystem;
  private DoubleSupplier translationSupplier;
  private DoubleSupplier strafeSupplier;
  private DoubleSupplier rotationSupplier;
  private BooleanSupplier isRobotCentricSupplier;
  private BooleanSupplier isSlowMode;
  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);
  private IntSupplier m_POVSupplier;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultDriveCommand(
      DrivetrainSubsystem subsystem,
      DoubleSupplier translationSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier isRobotCentricSupplier,
      BooleanSupplier isSlowModeSupplier,
      IntSupplier POVSupplier) {

    m_driveTrainSubSystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    this.translationSupplier = translationSupplier;
    this.strafeSupplier = strafeSupplier;
    this.rotationSupplier = rotationSupplier;
    this.isRobotCentricSupplier = isRobotCentricSupplier;
    this.isSlowMode = isSlowModeSupplier;
    m_POVSupplier = POVSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationVal = translationLimiter.calculate(
        MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.stickDeadband));
    double strafeVal = strafeLimiter.calculate(
        MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.stickDeadband));
    double rotationVal = rotationLimiter.calculate(
        MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.stickDeadband));

    int pov = m_POVSupplier.getAsInt();
    double forwardSpeed = 0.0;
    double strafeSpeed = 0.0;
    double m_adjustSpeed = 0.2;

    // If a d-pad button is being pressed, do micro adjustments robot-centric
    if (pov >= 0) {
      // Check the POV value and set speeds accordingly.
      if (pov == 0) {
        // D-pad up: move forward (robot's front)
        forwardSpeed = m_adjustSpeed;
      } else if (pov == 180) {
        // D-pad down: move backward
        forwardSpeed = -m_adjustSpeed;
      } else if (pov == 90) {
        // D-pad right: strafe right
        strafeSpeed = m_adjustSpeed;
      } else if (pov == 270) {
        // D-pad left: strafe left
        strafeSpeed = -m_adjustSpeed;
      }

      // Drive with the given speeds, no rotation.
      // The last parameter 'true' indicates robot-oriented control.
      m_driveTrainSubSystem.drive(new Translation2d(forwardSpeed, strafeSpeed), 0, true, isSlowMode.getAsBoolean());

    } else {
      // Drive normally with joystick in field-centric
      m_driveTrainSubSystem.drive(
          new Translation2d(translationVal, strafeVal).times(Constants.maxSpeed),
          rotationVal * Constants.maxAngularVelocity,
          isRobotCentricSupplier.getAsBoolean(),
          isSlowMode.getAsBoolean());
    }
  }
}