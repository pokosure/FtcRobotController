package org.firstinspires.ftc.teamcode.SwerveSystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Swerve extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            Constants.DriveConstants.kFrontLeftDriveMotorName,
            Constants.DriveConstants.kFrontLeftSteeringMotorName,
            Constants.DriveConstants.kFrontLeftDriveEncoderReversed,
            Constants.DriveConstants.kFrontLeftSteeringEncoderReversed,
            Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderName,
            Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule frontRight = new SwerveModule(
            Constants.DriveConstants.kFrontRightDriveMotorName,
            Constants.DriveConstants.kFrontRightSteeringMotorName,
            Constants.DriveConstants.kFrontRightDriveEncoderReversed,
            Constants.DriveConstants.kFrontRightSteeringEncoderReversed,
            Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderName,
            Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
    );

    private final SwerveModule backLeft = new SwerveModule(
            Constants.DriveConstants.kBackLeftDriveMotorName,
            Constants.DriveConstants.kBackLeftSteeringMotorName,
            Constants.DriveConstants.kBackLeftDriveEncoderReversed,
            Constants.DriveConstants.kBackLeftSteeringEncoderReversed,
            Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderName,
            Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule backRight = new SwerveModule(
            Constants.DriveConstants.kBackRightDriveMotorName,
            Constants.DriveConstants.kBackRightSteeringMotorName,
            Constants.DriveConstants.kBackRightDriveEncoderReversed,
            Constants.DriveConstants.kBackRightSteeringEncoderReversed,
            Constants.DriveConstants.kBackRightDriveAbsoluteEncoderName,
            Constants.DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            Constants.DriveConstants.kBackRightDriveAbsoluteEncoderReversed
    );

    private IMU imu;
    private double currentAngle;
    private double offsetAngle;
    private double actualAngle;
    public void initIMU() {
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.XZY, AngleUnit.DEGREES)));
        imu.initialize(parameters);
        currentAngle = 0.0;
        offsetAngle = 0.0;
    }
    public Swerve() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                initIMU();
            } catch (Exception e){

            }
        }).start();
    }

    public double getHeading() {
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
        currentAngle = orientation.firstAngle;
        if (currentAngle>offsetAngle) {actualAngle = currentAngle-offsetAngle;} else {actualAngle = (360-(offsetAngle-currentAngle))%360;}
        return actualAngle;
    }

    public void zeroHeading() {
        offsetAngle = offsetAngle+getHeading()%360;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {
        telemetry.addData("Heading", getHeading());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

}
