package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.frozenmilk.util.units.*;
import dev.frozenmilk.util.*;
import java.lang.Math;


public class Constants{
    // Drivetrain Dimensions
        //length
    Distance distance =
    double kTrackWidth = 12;
        //width
    double kWheelDiameter = 45;
    double kWheelCircumference = kWheelDiameter*Math.PI;
    // Mechanical/Power Settings
    double powerPercentage = 1;
    double kdrivegearratio = 6.93333333333;
    double ksteergearratio = 1;
    //




    //Dependent
    //Mechanical/Power Setting
    double kMaxAngularSpeedRadiansPerSecond = 5900/drivegearratio/60*2*Math.PI;



}
