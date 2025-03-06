package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SwerveSystem.Swerve;

public class RobotContainer {
    private final Swerve swerveSubsystem = new Swerve();
    GamepadEx driverOp = new GamepadEx(gamepad1);
    GamepadEx subOp = new GamepadEx(gamepad2);


    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new Control(swerveSubsystem,
                () -> driverOp.getLeftX(), () -> driverOp.getLeftY(), () -> driverOp.getRightX(), () -> !driverOp.getButton(GamepadKeys.Button.A)));
        configureButtonBindings();
    }

    public void configureButtonBindings() {
        driverOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> swerveSubsystem.zeroHeading());
    }
}
