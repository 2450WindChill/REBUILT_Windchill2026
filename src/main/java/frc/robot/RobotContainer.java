// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.Constants.Camera;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.SwerveMode;
// import frc.robot.Constants.autoConstants.ReefDirection;
// import frc.robot.Constants.autoConstants.ReefLevel;
import frc.robot.commands.DefaultDriveCommand;
// import frc.robot.subsystems.DrivetrainSubsystem;

// import java.util.Vector;

// import com.pathplanner.lib.commands.PathPlannerAuto;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
// import edu.wpi.first.wpilibj.event.BooleanEvent;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(SwerveMode.NEO);
  private final XboxController m_driverController = new XboxController(ControllerConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(ControllerConstants.kOperatorControllerPort);

  public final JoystickButton dr_aButton = new JoystickButton(m_driverController, Button.kA.value);
  public final JoystickButton dr_bButton = new JoystickButton(m_driverController, Button.kB.value);
  public final JoystickButton dr_xButton = new JoystickButton(m_driverController, Button.kX.value);
  public final JoystickButton dr_yButton = new JoystickButton(m_driverController, Button.kY.value);

  public final JoystickButton dr_leftBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
  public final JoystickButton dr_rightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);
  public final JoystickButton dr_startButton = new JoystickButton(m_driverController, Button.kStart.value);

  public final JoystickButton op_aButton = new JoystickButton(m_operatorController, Button.kA.value);
  public final JoystickButton op_bButton = new JoystickButton(m_operatorController, Button.kB.value);
  public final JoystickButton op_xButton = new JoystickButton(m_operatorController, Button.kX.value);
  public final JoystickButton op_yButton = new JoystickButton(m_operatorController, Button.kY.value);

  public final POVButton op_UpDpad = new POVButton(m_operatorController, 0);
  public final POVButton op_DownDpad = new POVButton(m_operatorController, 180);
  public final POVButton op_LeftDpad = new POVButton(m_operatorController, 270);
  public final POVButton op_RightDpad = new POVButton(m_operatorController, 90);

  public final JoystickButton op_leftBumper = new JoystickButton(m_operatorController, Button.kLeftBumper.value);
  public final JoystickButton op_rightBumper = new JoystickButton(m_operatorController, Button.kRightBumper.value);

  public SendableChooser<Command> m_chooser;
  Timer timer = new Timer();
  double time = 0.0;

  public RobotContainer() {
    m_drivetrainSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> (m_driverController.getLeftY()),
            () -> (m_driverController.getLeftX()),
            () -> (m_driverController.getRightX()),
            () -> Constants.isRobotCentric,
            () -> dr_leftBumper.getAsBoolean(),
            () -> m_driverController.getPOV()));
    configureControllerBindings();
    configureAutoChooser();
  }

  public Command getAutonomousCommand() {
    return null;
  }

  private void configureControllerBindings() {

  }

  private void configureAutoChooser() {
    m_chooser = new SendableChooser<>();
    SmartDashboard.putData("Auto Mode", m_chooser);
    // m_chooser.addOption("Autonomous", scoreCoral(ReefDirection.LEFT,
    // ReefLevel.L2));
  }

}