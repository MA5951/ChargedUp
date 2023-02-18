package frc.robot.subsystems.arm;

public class ArmConstants {

    public static boolean isThereCone = false;

    public static final int kCPR = 4096;

    public static final double ARM_EXTENSTION_KN = 0; // TODO
    public static final double ARM_EXUTENSTION_KP = 0; // TODO
    public static final double ARM_EXTENSTION_KI = 0; // TODO
    public static final double ARM_EXTENSTION_KD = 0; // TODO
    public static final double ARM_EXTENSTION_TOLERANCE = 0; // TODO meters


    public static final double ARM_ROTATION_KP = 0; // TODO
    public static final double ARM_ROTATION_KI = 0; // TODO
    public static final double ARM_ROTATION_KD = 0; // TODO
    public static final double ARM_ROTATION_TOLERANCE = 0; // TODO radians

    public static final double ARM_ROTATION_START_POSE = 0; // TODO radians
    public static final double ARM_ROTATION_MAX_POSE = 0; // TODO radians
    // public static final double ARM_DISTANCE_FROM_THE_CENTER = 0; // TODO
    // public static final double ARM_HIGHT = 0; // TODO

    public static final double ARM_EXTENSTION_DIAMETER_OF_THE_WHEEL = 0.0323342; // meters
    public static final double ARM_MASS_EXTENSTION = 0.42;

    public static final double ARM_ROTATION_GEAR = 1d / 400;

    public static final double NEO_ARM_ROTATION_KV = 473;
    public static final double NEA_ARM_ROTATION_KT =
        60 / (2 * Math.PI * NEO_ARM_ROTATION_KV);
    public static final double ARM_ROTATION_KT = 
        NEA_ARM_ROTATION_KT * ARM_ROTATION_GEAR;

    public static final double ARM_MASS = 6.8; // kg
    public static final double CONE_MASS = 0.653; // kg
    public static final double armExtestionMass = 2.9; // kg

    public static final double MIN_ROTATION_FOR_EXTENSTION = 0; // TODO radians
    public static final double MIN_EXTENSTION_FOR_ROTATION = 0; // TODO meters
    public static final double MIN_ROTATION_FOR_EXTENSTION_SAFTY_BUFFR = 0; // TODO radians

    public static final double ROTATION_FOR_MID_SCORING = 0; //TODO
    public static final double EXTENSTION_FOT_MID_SCORING = 0; //TODO
    public static final double ROTATION_FOR_HP = 0; //TODO
    public static final double EXTENSTION_FOR_HP = 0; //TODO
}
