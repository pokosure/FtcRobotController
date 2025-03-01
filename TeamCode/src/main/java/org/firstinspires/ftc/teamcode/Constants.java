package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.frozenmilk.util.units.distance.Distance;
import dev.frozenmilk.util.units.distance.DistanceUnits;
import dev.frozenmilk.util.units.distance.Distances;

import java.lang.Math;


public class Constants {
    // Physical Constants
    //length
    Distance kWheelBase = new Distance(DistanceUnits.INCH, 10.0);
    Distance kTrackWidth = new Distance(DistanceUnits.INCH, 10.0);
    public static final Distance kWheelDiameterMeters = new Distance(DistanceUnits.METER, 0.005);

    // Mechanical/Power Settings
    public static double powerPercentage = 1.0;
    public static final double motorrpm = 5900.0;
    public static final double kdrivegearratio = 1 / (1.0 * (24.0 / 10.0) * (52.0 / 18.0));
    public static final double ksteergearratio = 1.0;
    public static final double kDriveEncoderRot2Meter = kdrivegearratio * Math.PI * kWheelDiameterMeters.getValue();
    public static final double kSteerEncoderRot2Rad = ksteergearratio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kSteeringEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60;
    public static final double kPSteering = 0.5;

    public static final int TICKS_PER_REV_6000 = 28;
    //


    //Dependent
    //Physical Constants
    Distance kWheelCircumferenceMeters = kWheelDiameterMeters.times(Math.PI);
    //Mechanical/Power Setting
    double kMaxAngularSpeedRadiansPerSecond = motorrpm / kdrivegearratio / 60.0 * 2 * Math.PI;


    public static final class DriveConstants {

        public static final double kTrackWidth = Distances.inch(12.0).intoMeters().getValue();
        // Distance between right and left wheels
        public static final double kWheelBase = Distances.inch(12.0).intoMeters().getValue();
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final String kFrontLeftDriveMotorName = "";
        public static final String kBackLeftDriveMotorName = "";
        public static final String kFrontRightDriveMotorName = "";
        public static final String kBackRightDriveMotorName = "";

        public static final String kFrontLeftSteeringMotorName = "";
        public static final String kBackLeftSteeringMotorName = "";
        public static final String kFrontRightSteeringMotorName = "";
        public static final String kBackRightSteeringMotorName = "";

        public static final boolean kFrontLeftSteeringEncoderReversed = true;
        public static final boolean kBackLeftSteeringEncoderReversed = true;
        public static final boolean kFrontRightSteeringEncoderReversed = true;
        public static final boolean kBackRightSteeringEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final String kFrontLeftDriveAbsoluteEncoderName = "";
        public static final String kBackLeftDriveAbsoluteEncoderName = "";
        public static final String kFrontRightDriveAbsoluteEncoderName = "";
        public static final String kBackRightDriveAbsoluteEncoderName = "";

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = Distances.ft(7.5).intoMeters().getValue();
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;


    }
}
