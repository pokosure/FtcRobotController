package org.firstinspires.ftc.teamcode.SwerveSystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class SwerveModule extends Constants {
    private final DcMotorEx driveMotor;
    private final CRServo steerMotor;
    private PIDFController steerpidf;
    private AnalogInput absoluteEncoder;
    private boolean absoluteEncoderReversed;
    private double absoluteEncoderOffsetRad;
    private int driveEncoderInt;
    private double driveEncoder;
    private double driveEncoderVeloRev;
    private double steerEncoder;
    private double steerEncoderVeloRad;
    double prevPosition = 0;
    double prevTime = 0;
    int filterSize = 5;


    double positionInRad;

    public SwerveModule(String drivemotorId, String steerMotorId, boolean driveMotorReversed,
                        boolean steerMotorReversed, String absoluteEncoderId,
                        double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = hardwareMap.get(AnalogInput.class, absoluteEncoderId);


        driveMotor = hardwareMap.get(DcMotorEx.class, drivemotorId);
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (driveMotorReversed) {
            driveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        } else {
            driveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        }

        steerMotor = hardwareMap.get(CRServo.class, steerMotorId);
        if (steerMotorReversed) {
            steerMotor.setDirection(CRServo.Direction.REVERSE);
        } else {
            steerMotor.setDirection(CRServo.Direction.FORWARD);
        }

        driveEncoderInt = driveMotor.getCurrentPosition();
        driveEncoder = (double) driveEncoderInt/TICKS_PER_REV_6000; //Revolutions
        driveEncoderVeloRev = driveMotor.getVelocity(AngleUnit.DEGREES)/360.0;
        //POSITION AND VELOCITY CONVERSION FACTORS

        steerEncoder = (absoluteEncoder.getVoltage() / 3.3 * 360); //Revolutions

        steerpidf = new PIDFController(0,0,0,0); //Fill parameters later!

        steerEncoder = getSteerPosition(); //in rad

    }

    public double getDrivePosition() {
        return driveEncoder*kDriveEncoderRot2Meter;
    }

    public double getSteerPosition() {
        return steerEncoder*kSteerEncoderRot2Rad;
    }

    public double getDriveVelocity() {
        return driveEncoderVeloRev*kDriveEncoderRot2Meter;
    }

    public double getTurningVelocity() {

        return findSteerVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = steerEncoder/360*2*Math.PI;
        angle = angle - absoluteEncoderOffsetRad;
        angle = (angle + Math.PI) % (2*Math.PI) - Math.PI;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.005) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.setPower(state.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        steerMotor.setPower(steerpidf.calculate(getSteerPosition(), state.angle.getRadians()));
        telemetry.addData("Swerve:" + driveMotor.getDeviceName() + ": state", state.toString());
    }

    public void stop() {
        driveMotor.setPower(0);
        steerMotor.setPower(0);
    }

    private double emaVelocity = 0;  // Stores the exponential moving average
    private final double alpha = 0.2;  // Smoothing factor (adjust as needed)


    public double findSteerVelocity() {
        double currentPosition = getSteerPosition();
        double currentTime = System.nanoTime()/1e-9;
        double deltaPos = currentPosition - prevPosition;
        if (deltaPos == 0) return 0;
        if (Math.abs(currentPosition-prevPosition) > Math.PI) { //Detect wraparound
            if (currentPosition < prevPosition) {
                currentPosition += 2*Math.PI;
            } else {
                prevPosition += 2*Math.PI;
            }
        }
        double deltaTime = currentTime - prevTime;
        if (deltaTime <= 0) return emaVelocity;

        double velocity = deltaPos/deltaTime;

        emaVelocity = alpha * velocity + (1 - alpha) * emaVelocity;

        prevPosition = currentPosition;
        prevTime = currentTime;

        return emaVelocity;
    }
}
