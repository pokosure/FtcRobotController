package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.frozenmilk.util.units.distance.Distance;
import dev.frozenmilk.util.units.distance.DistanceUnits;
import dev.frozenmilk.util.units.distance.Distances;

import java.lang.Math;


public class Constants{
    // Physical Constants
        //length
    Distance kWheelBase = new Distance(DistanceUnits.INCH, 10.0);
    Distance kTrackWidth = new Distance(DistanceUnits.INCH, 10.0);
    Distance kWheelDiameter = new Distance(DistanceUnits.INCH, 2.0);

    // Mechanical/Power Settings
    double powerPercentage = 1.0;
    double motorrpm = 5900.0;
    double kdrivegearratio = 6.93333333333;
    double ksteergearratio = 1.0;
    //




    //Dependent
    //Physical Constants
    Distance kWheelCircumference = kWheelDiameter.times(Math.PI);
    //Mechanical/Power Setting
    double kMaxAngularSpeedRadiansPerSecond = motorrpm/kdrivegearratio/60.0



}
