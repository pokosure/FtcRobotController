package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;

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
}
