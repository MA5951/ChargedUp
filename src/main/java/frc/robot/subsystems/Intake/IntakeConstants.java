// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

/** Add your docs here. */
public class IntakeConstants {
    public final static double CLOSE_POSITION = Math.toRadians(90); 
    public final static double OPEN_POSITION = Math.toRadians(0);
    public final static double POSITION_CONVERSION_FACTOR = ((Math.PI * 0.5) / 1229.125107485757) * 57.296;
    public final static double MIDDLE_POSITION = Math.toRadians(30);

    public static double KG = 0.05 / Math.cos(MIDDLE_POSITION);
    public final static double KP = 1;
    public final static double KI = 0;
    public final static double KD = 0;

    public final static double OPEN_INTAKE_POWER = -0.4;
    public final static double CLOSE_INTAKE_POWER = 0.4;


    public static final int TICKS_PER_ROUND = 42;

    public static final double GEAR_ROTATION_RATIO = 1/84.44;

    public static final double POSITION_TOLORANCE = Math.toRadians(2);

    public static final double INTAKE_POWER = -0.8;
    public static final double OUTAKE_POWER = -INTAKE_POWER;


}