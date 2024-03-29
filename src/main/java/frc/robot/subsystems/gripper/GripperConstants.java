package frc.robot.subsystems.gripper;

import frc.robot.subsystems.arm.ArmConstants;

public final class GripperConstants {
    public static final double MAX_POSE = 4.474350452423096;
    public static final double OPEN_POSITION = 3.002450704574585;
    public static final double CLOSE_POSITION = Math.toRadians(0);
    public static final double INTAKE_POSITION = 1.789304614067078;
    public static final double BEFOR_GRABING_POSE = 1.1088119745254517 -0.2;


    public static final double MAX_ARM_ROTATION_FOR_GRIPPER = 
        ArmConstants.ARM_ROTATION_START_POSE + 0.02;

    public static final double TACHED_CURRENT= 20;

    public static final double HOLDING_POWER = -0.09;

    public static final double CLOSING_POWER = -0.3;
    public static final double OPENING_POWER = -CLOSING_POWER;

    public static final double GRIPPER_TOLERANCE = Math.toRadians(3);

    public static final double GEAR = 1/100;

    public static final int TICKS_PER_ROUND = 42;

    public static final double MAX_CURRENT_ALLOWED = 13;

    public static final double POSITION_CONVERSION_FACTOR =
        Math.PI / 34.404449462890625;

    public static final double kP = 1.8;
    public static final double kI = 0;
    public static final double kD = 0;

}
