package frc.robot.swerveModules;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class BaseWindChillSwerveModule {
    public BaseWindChillSwerveModule(int ModuleNumber) {
        this.moduleNumber = ModuleNumber;
    }

    public int moduleNumber;

    public BaseWindChillSwerveModule[] swerveModules;

    abstract public SwerveModulePosition getPosition();

    abstract public void setDesiredState(SwerveModuleState desiredState, boolean isSlowMode);

    abstract public SwerveModuleState getState();

    abstract public SwerveModuleState getAutoState();

    abstract public void resetToAbsolute();

    abstract public double getDriveEncoder();

    abstract public void setPosition(double position);
}
