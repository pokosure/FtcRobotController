package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SwerveSystem.Swerve;

import java.util.function.Supplier;

public class Control extends OpMode {
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
    }

    @Override
    public void init() {

    }
    @Override
    public void init_loop() {

    }
    @Override
    public void stop() {

    }
    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }

}
