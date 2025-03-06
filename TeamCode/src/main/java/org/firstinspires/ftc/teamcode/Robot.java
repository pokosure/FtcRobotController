package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SwerveTest")
public class Robot extends CommandOpMode {
    private RobotContainer bot;

    @Override
    public void initialize() {
        bot = new RobotContainer();
    }
}
