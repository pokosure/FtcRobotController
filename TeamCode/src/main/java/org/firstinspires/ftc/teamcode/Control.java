package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;

import org.firstinspires.ftc.teamcode.SwerveSystem.Constants;
import org.firstinspires.ftc.teamcode.SwerveSystem.Swerve;

import java.util.function.Supplier;



public class Control extends CommandBase {
    private final Swerve swerveSubSystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, steeringSpdFunction;
    private final Supplier<Boolean> fieldOrientatedControl;

    public Control(Swerve swerveSubsystem, Supplier<Double> xSpdFunction,
                   Supplier<Double> ySpdFunction, Supplier<Double> steeringSpdFunction,
                   Supplier<Boolean> fieldOrientatedControl) {
        this.swerveSubSystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.steeringSpdFunction = steeringSpdFunction;
        this.fieldOrientatedControl = fieldOrientatedControl;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double steerSpeed = steeringSpdFunction.get();

        xSpeed = Math.abs(xSpeed) > Constants.OIConstants.kDeadband?xSpeed:0.0;
        ySpeed = Math.abs(ySpeed) > Constants.OIConstants.kDeadband?ySpeed:0.0;
        steerSpeed = Math.abs(steerSpeed) > Constants.OIConstants.kDeadband?steerSpeed:0.0;

        // PUT IN SLEW RATE LIMITER AFTER
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientatedControl.get()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed,ySpeed,steerSpeed,swerveSubSystem.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,steerSpeed);
        }

        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubSystem.setModuleStates(moduleStates);
    }
    @Override
    public void initialize() {

    }
    @Override
    public void end(boolean interrupted) {
        swerveSubSystem.stopModules();
    }
    @Override
    public boolean isFinished() {
        return false;
    }

}
