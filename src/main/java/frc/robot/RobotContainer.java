// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ModuleLayer.Controller;

// import frc.robot.Constants.Camera;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveMode;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RevShootCommand;
import frc.robot.commands.UnstuckCommand;
import frc.robot.commands.StowIntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
        public static final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
        private static final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
        public static final ShooterSubsystem m_shooterSubsytem = new ShooterSubsystem();
        public static final IndexSubsystem m_indexSubsytem = new IndexSubsystem();
        public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(SwerveMode.KRAKEN);
        private final XboxController m_driverController = new XboxController(ControllerConstants.kDriverControllerPort);
        private final XboxController m_operatorController = new XboxController(
                        ControllerConstants.kOperatorControllerPort);

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
        public final JoystickButton op_rightBumper = new JoystickButton(m_operatorController,
                        Button.kRightBumper.value);

        private Boolean slowModeState = false;
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
                                                () -> getSlowMode(),
                                                () -> m_driverController.getPOV()));
                configureCompControllerBindings();
                configureAutoChooser();
        }

        public Boolean getSlowMode() {

                return slowModeState;
        }

        public void setSlowMode(Boolean slowMode) {
                slowModeState = slowMode;
        }

        public Command getAutonomousCommand() {

                //return new SequentialCommandGroup(
                        // new InstantCommand(() -> m_IntakeSubsystem.deploy(), m_IntakeSubsystem),
                        // new InstantCommand(() -> m_shooterSubsytem.autoSpinShoot(), m_shooterSubsytem),

                        // new WaitCommand(1),
                        // new InstantCommand(() -> m_indexSubsytem.reverseIndexFullSpeed(), m_indexSubsytem),

                        // new WaitCommand(1),
                        // new InstantCommand(() -> m_indexSubsytem.spinIndex(), m_indexSubsytem),

                        // new WaitCommand(10),
                        // new InstantCommand(() -> m_indexSubsytem.reverseIndexFullSpeed(), m_indexSubsytem),

                        // new WaitCommand(1),
                        // new InstantCommand(() -> m_indexSubsytem.spinIndex(), m_indexSubsytem),

                        // new WaitCommand(10),

                        // new InstantCommand(() -> m_indexSubsytem.stopIndex(), m_indexSubsytem),
                        // new InstantCommand(() -> m_shooterSubsytem.stopShoot(), m_shooterSubsytem));
                        return null;
        }

        private void configureTestControllerBindings() {
        }

        private void configureCompControllerBindings() {
                // new Trigger(() ->
                // m_operatorController.getRightBumper()).whileTrue(m_shooterSubsytem.shootWhileHeld());
                // new Trigger(() ->
                // m_operatorController.getLeftBumper()).whileTrue(m_shooterSubsytem.loadWhileHeld());
                // new Trigger(() -> m_operatorController.getAButton()).whileTrue(new
                // IntakeCommand(m_IntakeSubsystem));
                // new Trigger(() -> m_operatorCont
                // roller.getYButton())
                // .onTrue(Commands.runOnce(() -> m_shooterSubsytem.reverseIndex(),
                // m_shooterSubsytem));
                // new Trigger(() -> m_operatorController.getXButton())
                // .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.retract(),
                // m_IntakeSubsystem));

                // DRIVER CONTROLS
                // new Trigger(() -> dr_aButton.getAsBoolean())
                // .onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.zeroGyro(),
                // m_drivetrainSubsystem));

                // OPERATOR CONTROL
                new Trigger(() -> m_operatorController.getBButton())
                                .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.intake(), m_IntakeSubsystem));
                new Trigger(() -> m_operatorController.getBButton())
                                .onFalse(Commands.runOnce(() -> m_IntakeSubsystem.stopIntake(), m_IntakeSubsystem));
                new Trigger(() -> m_operatorController.getYButton())
                                .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.retract(), m_IntakeSubsystem));
                new Trigger(() -> m_operatorController.getAButton())
                                .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.deploy(), m_IntakeSubsystem));
                new Trigger(() -> m_operatorController.getXButton())
                                .onTrue(Commands.runOnce(() -> m_IntakeSubsystem.outTake(), m_IntakeSubsystem));
                new Trigger(() -> m_operatorController.getXButton())
                                .onFalse(Commands.runOnce(() -> m_IntakeSubsystem.stopOutTake(), m_IntakeSubsystem));

                // DRIVER CONTROL
                new Trigger(() -> m_driverController.getAButton())
                                .onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.zeroGyro(),
                                                m_drivetrainSubsystem));
                new Trigger(() -> m_driverController.getBButton())
                                .onTrue(Commands.runOnce(() -> setSlowMode(!slowModeState))); // TEST
                // new Trigger(() -> m_driverController.getRightBumperButton())
                // .whileTrue(shootWhileHeld());
                new Trigger(() -> m_driverController.getLeftBumperButton())
                                .onTrue(Commands.runOnce(() -> m_indexSubsytem.reverseIndex(), m_indexSubsytem));
                new Trigger(() -> m_driverController.getLeftBumperButton())
                                .onFalse(Commands.runOnce(() -> m_indexSubsytem.stopIndex(), m_indexSubsytem));

                new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.3)
                                .onTrue(Commands.runOnce(() -> m_shooterSubsytem.spinShoot(), m_shooterSubsytem));
                new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.3)
                                .onFalse(Commands.runOnce(() -> m_shooterSubsytem.stopShoot(), m_shooterSubsytem));

                new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.3)
                                .whileTrue(new UnstuckCommand(m_shooterSubsytem, m_indexSubsytem));

                new Trigger(() -> m_driverController.getRightBumperButton())
                                .onTrue(Commands.runOnce(() -> m_indexSubsytem.spinIndex(), m_indexSubsytem));
                new Trigger(() -> m_driverController.getRightBumperButton())
                                .onFalse(Commands.runOnce(() -> m_indexSubsytem.stopIndex(), m_indexSubsytem));

                // new Trigger(() -> m_driverController.getLeftBumperButton())
                // .onTrue(Commands.runOnce(() -> m_indexSubsytem.reverseIndex(),
                // m_indexSubsytem));
                // new Trigger(() -> m_driverController.getLeftBumperButton())
                // .onFalse(Commands.runOnce(() -> m_indexSubsytem.stopIndex(),
                // m_indexSubsytem));
        }

        private void configureAutoChooser() {
                m_chooser = new SendableChooser<>();
                SmartDashboard.putData("Auto Mode", m_chooser);
                // m_chooser.addOption("Autonomous", scoreCoral(ReefDirection.LEFT,
                // ReefLevel.L2));
        }

        public Command shootWhileHeld() {
                // return Commands.sequence(
                // this.runOnce(() -> spinShoot()),
                // Commands.waitUntil(this::atSpeed),
                // Commands.startEnd(this::spinIndex, this::stopIndex, this))
                // .finallyDo(interupted -> {
                // stopIndex();
                // stopShoot();
                // });
                Trigger readyToFeed = new Trigger(m_shooterSubsytem::atSpeed).debounce(0.1);
                return Commands.startEnd(m_shooterSubsytem::spinShoot, m_shooterSubsytem::stopShoot, m_shooterSubsytem)
                                .alongWith(
                                                Commands.startEnd(m_indexSubsytem::spinIndex,
                                                                m_indexSubsytem::stopIndex, m_indexSubsytem)
                                                                .onlyWhile(readyToFeed));
        }

        // public Command loadWhileHeld() {
        // return Commands.sequence(
        // this.runOnce(() -> m_shooterSubsytem.loadShoot()),
        // Commands.startEnd(m_indexSubsytem::spinIndex, m_indexSubsytem::stopIndex,
        // m_indexSubsytem))
        // .finallyDo(interupted -> {
        // stopIndex();
        // stopShoot();
        // });
        // }
        // notes: slow down inake reverse
        // done
}