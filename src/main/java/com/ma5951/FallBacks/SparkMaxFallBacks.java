// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.FallBacks;

import com.ma5951.utils.Logger;
import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public class SparkMaxFallBacks{

    private CANSparkMax sparkMax;
    private int sparkMaxID;
    private Logger logger;

    public SparkMaxFallBacks(CANSparkMax sparkMax)
    {
        this.sparkMax = sparkMax;
        sparkMaxID = sparkMax.getDeviceId();
        logger = Logger.getInstance();
    }

    public boolean isBrownOut(){
        if(sparkMax.getBusVoltage() < 7.7){
            return true;
        }
        logger.logMessage("spark id: " + sparkMaxID + " is burnOut!");
        return false;
    }

    public boolean isMotorControllerConnect(){
        if(sparkMax.getFirmwareVersion() == 0){
            return true;
        }
        logger.logMessage("motor controller id:" + sparkMaxID + "disconected!");
        return false;
    }

    public boolean isMotorConnect(){
        if(sparkMax.getOutputCurrent() == 0){
            return true;
        }
        logger.logMessage("motor id:" + sparkMaxID + "disconected!");
        return false;
    }
}
