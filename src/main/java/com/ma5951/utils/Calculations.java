// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils;

import edu.wpi.first.math.MathUtil;

public class Calculations {
    private final static double K_ELECTRON_CHARGE = -1.6e-19;

    public static double fromRPMToLinearSpeed(double RPM, double gear) {
        return (RPM / 10) * gear;
    }

    public static double fromLinearSpeedToRPM(double linearSpeed, double gear) {
        return (linearSpeed / gear) * 10;
    }

    public static double toRPMFromEncoder(double Rate, int TPR) {
        return (Rate / TPR) / 60;
    }

    public static double toRPMFromEncoderConnectToTalon(double Rate, int TPR) {
        return ((Rate * 10) / TPR) / 60;
    }

    public static double ForceToVoltage(double force) {
        return MathUtil.clamp(force / K_ELECTRON_CHARGE, RobotConstants.
        MAX_VOLTAGE, -RobotConstants.MAX_VOLTAGE);
    }

    public static double RPMToVoltage(double RPM, double maxRPM) {
        return (RPM / maxRPM) * RobotConstants.MAX_VOLTAGE;
    }
}