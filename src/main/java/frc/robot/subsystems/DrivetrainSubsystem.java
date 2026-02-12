
package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveMode;
import frc.robot.swerveModules.BaseWindChillSwerveModule;
import frc.robot.swerveModules.WindChillKrakenSwerveModule;
import frc.robot.swerveModules.WindChillNeoSwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
  public final Pigeon2 gyro;
  public BaseWindChillSwerveModule[] swerveModules;
  public SwerveDriveOdometry swerveOdometry;
  public SparkMax testMotor;
  public CANcoder canCoder;
  public HolonomicDriveController holonomicDriveController;
  public static SwerveDrivePoseEstimator poseEstimate;
  public static Enum<SwerveMode> mymode = Constants.SwerveMode.KRAKEN;
  private final Field2d m_field = new Field2d();

  public DrivetrainSubsystem(SwerveMode myMode) {

    CANBus canbus = new CANBus("canivore");

    switch (myMode) {
      case NEO:
        swerveModules = new WindChillNeoSwerveModule[] {
            new WindChillNeoSwerveModule(0, Constants.FrontLeftNeoModule.constants, canbus),
            new WindChillNeoSwerveModule(1, Constants.FrontRightNeoModule.constants, canbus),
            new WindChillNeoSwerveModule(2, Constants.BackLeftNeoModule.constants, canbus),
            new WindChillNeoSwerveModule(3, Constants.BackRightNeoModule.constants, canbus) };
        break;

      case KRAKEN:
        swerveModules = new WindChillKrakenSwerveModule[] {
            new WindChillKrakenSwerveModule(0, Constants.FrontLeftKrakenModule.constants, canbus),
            new WindChillKrakenSwerveModule(1, Constants.FrontRightKrakenModule.constants, canbus),
            new WindChillKrakenSwerveModule(2, Constants.BackLeftKrakenModule.constants, canbus),
            new WindChillKrakenSwerveModule(3, Constants.BackRightKrakenModule.constants, canbus) };
        break;

      default:
        swerveModules = new WindChillKrakenSwerveModule[] {
            new WindChillKrakenSwerveModule(0, Constants.FrontLeftKrakenModule.constants, canbus),
            new WindChillKrakenSwerveModule(1, Constants.FrontRightKrakenModule.constants, canbus),
            new WindChillKrakenSwerveModule(2, Constants.BackLeftKrakenModule.constants, canbus),
            new WindChillKrakenSwerveModule(3, Constants.BackRightKrakenModule.constants, canbus) };
        break;
    }

    gyro = new Pigeon2(Constants.pigeonID, canbus);

    poseEstimate = new SwerveDrivePoseEstimator(
        Constants.swerveKinematics,
        getGyroYaw(),
        getModulePositions(),
        new Pose2d(0, 0, new Rotation2d(0)));

    holonomicDriveController = new HolonomicDriveController(
        new PIDController(1, 0, 0), new PIDController(1, 0, 0),
        new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(6.28, 3.14)));
    zeroGyro();

    // Do this in either robot or subsystem init
    SmartDashboard.putData("Field", m_field);
    // Do this in either robot periodic or subsystem periodic
    m_field.setRobotPose(getBotPose());

    // -----------------------------------------------------------------------------------------------------------------------------------------
    // Pathplanner:

    // Load the RobotConfig from the GUI settings
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      return;
    }

    // AutoBuilder is used to create full autonomous routines based on auto files
    // created in the GUI app
    // This code configures it to control our robot.
    AutoBuilder.configure(
        getBotPoseSupplier(),
        resetPoseConsumer(),
        getSpeedsSupplier(),
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(0.05, 0.0, 0.0),
            new PIDConstants(0.05, 0.0, 0.0)),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  @Override
  public void periodic() {
    // poseEstimate.update(
    // getGyroYaw(),
    // getPositions());
  }

  // --------------------------------------------------------------------------------
  // Swerve instantiation and methods
  // --------------------------------------------------------------------------------
  public void drive(Translation2d translation, double rotation, boolean isRobotCentric, boolean isSlowMode) {
    SwerveModuleState[] swerveModuleStates;
    if (isRobotCentric) {
      swerveModuleStates = Constants.swerveKinematics.toSwerveModuleStates(
          new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    } else {
      swerveModuleStates = Constants.swerveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(),
              translation.getY(),
              rotation,
              getGyroYaw()));
    }

    for (BaseWindChillSwerveModule mod : swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isSlowMode);
    }
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getPosition();
    }
    return positions;
  }

  public Rotation2d getGyroYaw() {
    Rotation2d rotation2d = Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    return rotation2d;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (BaseWindChillSwerveModule mod : swerveModules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (BaseWindChillSwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModuleState[] getAutoModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (BaseWindChillSwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getAutoState();
    }
    return states;
  }

  public BaseWindChillSwerveModule[] getModules() {
    return swerveModules;
  }

  public void resetMods() {
    for (BaseWindChillSwerveModule mod : swerveModules) {
      mod.resetToAbsolute();
    }
  }

  public double getFrontLeftEncoderVal() {
    double frontLeftEncoderVal = swerveModules[0].getDriveEncoder();

    return frontLeftEncoderVal;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public void setGyro(int angle) {
    gyro.setYaw(angle);
  }

  // -------------------------------------------------------------------------------
  // Pose Estimation:
  // --------------------------------------------------------------------------------

  // Gets current estimated bot pose
  public static Pose2d getBotPose() {
    return poseEstimate.getEstimatedPosition();
  }

  // Gets current estimated bot pose
  public static Supplier<Pose2d> getBotPoseSupplier() {
    return () -> poseEstimate.getEstimatedPosition();
  }

  public Pose2d getThisPose() {
    return poseEstimate.getEstimatedPosition();
  }

  public void zeroPose() {
    poseEstimate = new SwerveDrivePoseEstimator(
        Constants.swerveKinematics,
        getGyroYaw(),
        getModulePositions(),
        new Pose2d(0, 0, new Rotation2d(0)));
  }

  public void resetPose(Pose2d newPose) {
    poseEstimate.resetPosition(gyro.getRotation2d(), getPositions(),
        newPose);
  }

  public Consumer<Pose2d> resetPoseConsumer() {
    return (lambdaNewPose) -> poseEstimate.resetPosition(gyro.getRotation2d(), getPositions(), lambdaNewPose);
  }

  // Gets estimated bot x
  public double getBotX() {
    return getBotPose().getX();
  }

  // Gets estimated bot y
  public double getBotY() {
    return getBotPose().getY();
  }

  // Gets estimated bot rotation
  public double getBotRotation() {
    return getBotPose().getRotation().getDegrees();
  }

  // -------------------------------------------------------------------------------
  // Pathplanning methods:
  // --------------------------------------------------------------------------------

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public StatusCode resetCANcoder() {
    return canCoder.setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetCANcoder();
    swerveOdometry.resetPosition(
        gyro.getRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public Supplier<ChassisSpeeds> getSpeedsSupplier() {
    return () -> Constants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public ChassisSpeeds getAutoSpeeds() {
    ChassisSpeeds temChassisSpeeds = Constants.swerveKinematics.toChassisSpeeds(getAutoModuleStates());
    ChassisSpeeds reversedcChassisSpeeds = new ChassisSpeeds(-temChassisSpeeds.vxMetersPerSecond,
        -temChassisSpeeds.vyMetersPerSecond, -temChassisSpeeds.omegaRadiansPerSecond);
    return reversedcChassisSpeeds;
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.maxSpeed);
    var modules = getModules();
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(targetStates[i], false);
    }
  }

  public void setPosition(double position) {
    swerveModules[0].setPosition(position);
    System.err.println("Setting position to 0");
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds reversedChassisSpeeds = new ChassisSpeeds(-robotRelativeSpeeds.vxMetersPerSecond,
        -robotRelativeSpeeds.vyMetersPerSecond, -robotRelativeSpeeds.omegaRadiansPerSecond);
    SwerveModuleState[] modStates = Constants.swerveKinematics.toSwerveModuleStates(reversedChassisSpeeds);

    for (BaseWindChillSwerveModule mod : swerveModules) {
      mod.setDesiredState(modStates[mod.moduleNumber], false);
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}