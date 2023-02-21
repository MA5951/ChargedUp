package frc.robot.subsystems.arm;

public class ArmConstants {

    public static boolean isThereCone = false;

    public static final int kCPR = 4096;

    public static final double ARM_EXUTENSTION_KP = 3;
    public static final double ARM_EXTENSTION_KI = 0.0005;
    public static final double ARM_EXTENSTION_KD = 0.8;
    public static final double ARM_EXTENSTION_TOLERANCE = 0; // TODO meters


    public static final double ARM_ROTATION_KP = 1.3;
    public static final double ARM_ROTATION_KI = 0.000009;
    public static final double ARM_ROTATION_KD = 0;
    public static final double ARM_ROTATION_TOLERANCE = 0; // TODO radians

    public static final double ARM_ROTATION_START_POSE = -0.987;
    public static final double ARM_ROTATION_MAX_POSE = 0; // TODO radians
    public static final double ARM_EXTENTION_MAX_POSE = 0.42;
    // public static final double ARM_DISTANCE_FROM_THE_CENTER = 0; // TODO
    // public static final double ARM_HIGHT = 0; // TODO

    public static final double ARM_EXTENSTION_DIAMETER_OF_THE_WHEEL = 0.0323342; // meters
    public static final double ARM_MASS_EXTENSTION = 0.42;

    public static final double ARM_ROTATION_GEAR = 1d / 400;

    public static final double ARM_ROTATION_KT = 0.0010600019920617125;

    public static final double ARM_MASS = 6.8; // kg
    public static final double CONE_MASS = 0.653; // kg
    public static final double armExtestionMass = 2.9; // kg

    public static final double MIN_ROTATION_FOR_EXTENSTION = 0; // TODO radians
    public static final double MIN_EXTENSTION_FOR_ROTATION = 0; // TODO meters
    public static final double MIN_ROTATION_FOR_EXTENSTION_SAFTY_BUFFR = 0; // TODO radians
    public static final double ARM_EXTENSTION_FOR_GRABING = 0; // TODO meters

    public static final double ROTATION_MID_FOR_BEFOR_SCORING = 0; //TODO
    public static final double ROTATION_MID_FOR_SCORING = 0; //TODO
    public static final double EXTENSTION_FOT_MID_SCORING = 0; //TODO
    public static final double ROTATION_MID_FOR_BEFOR_SCORING_FROM_THE_BACK = 0; // TODO
    public static final double ROTATION_FOR_MID_SCORING_FROM_THE_BACK = 0; //TODO
    public static final double EXTENSTION_FOT_MID_SCORING_FROM_THE_BACK = 0; //TODO
    public static final double ROTATION_FOR_LOW_SCORING = 0; //TODO
    public static final double EXTENSTION_FOT_LOW_SCORING = 0; //TODO
    public static final double ROTATION_FOR_HP = 0; //TODO
    public static final double EXTENSTION_FOR_HP = 0; //TODO

    public static final double ARM_POS_FOR_INTAKE = 0; //TODO
}
