package frc.robot.subsystems.gripper;

import frc.robot.subsystems.arm.ArmConstants;

public final class GripperConstants {
    public static final double OPEN_POSITION = Math.toRadians(180);
    public static final double CLOSE_POSITION = Math.toRadians(0);
    public static final double INTAKE_POSITION = Math.toRadians(98.03518730824291);

    public static final double MAX_ARM_ROTATION_FOR_GRIPPER = 
        ArmConstants.ARM_ROTATION_START_POSE + 0.05;

    public static final double CURRENT_JUMP = 0; //TODO

    public static final double CUBE_POWER = -0; //TODO
    public static final double CONE_POWER = -0; //TODO

    public static final double CLOSING_POWER = 0; //TODO
    public static final double OPENING_POWER = -CLOSING_POWER;

    public static final double GRIPPER_TOLERANCE = Math.toRadians(5);

    public static final double GEAR = 1/100;

    public static final double CONE_ANGLE = 0; //TODO

    public static final int TICKS_PER_ROUND = 42;

    public static final double MAX_CURRENT_ALLOWED = 13;

    public static final double MIN_ANGLE_FOR_ARM = 0; // TODO

    public static final double POSITION_CONVERSION_FACTOR =
        Math.PI / 34.404449462890625;

    public static final double kP = 1.55;
    public static final double kI = 0;
    public static final double kD = 0;

}
