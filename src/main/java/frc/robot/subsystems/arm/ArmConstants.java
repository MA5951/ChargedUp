package frc.robot.subsystems.arm;

public class ArmConstants {

    public static final int kCPR = 4096;

    public static final double ARM_EXUTENSTION_KP = 3.2;
    public static final double ARM_EXTENSTION_KI = 0.0005;
    public static final double ARM_EXTENSTION_KD = 0.8;
    public static final double ARM_EXTENSTION_TOLERANCE = 0.015;


    public static final double ARM_ROTATION_KP = 1.8;
    public static final double ARM_ROTATION_KI = 0.00001;
    public static final double ARM_ROTATION_KD = 0;
    public static final double ARM_ROTATION_TOLERANCE = Math.toRadians(1.7);

    public static final double ARM_ROTATION_START_POSE = -0.987 -0.26998063921928406;
    public static final double ARM_ROTATION_MAX_POSE = Math.toRadians(208);
    public static final double ARM_EXTENTION_MAX_POSE = 0.43;

    public static final double ARM_EXTENSTION_DIAMETER_OF_THE_WHEEL = 0.0323342; // meters
    public static final double ARM_MASS_EXTENSTION = 0.43;

    public static final double ARM_ROTATION_GEAR = 1d / 400;

    public static final double ARM_ROTATION_KT = 0.0010600019920617125;

    public static final double ARM_MASS = 6.8; // kg
    public static final double CONE_MASS = 0.653; // kg
    public static final double armExtestionMass = 2.9; // kg

    public static final double MIN_ROTATION_FOR_EXTENSTION = -0.4448544383049011;
    public static final double MIN_EXTENSTION_FOR_ROTATION = 0.04;
    public static final double MIN_ROTATION_FOR_EXTENSTION_SAFTY_BUFFR = -0.448544383049011 + 0.05;
    public static final double MIN_ROTATION_FOR_CLOSING_INTAKE = -1.1348544383049011 + 0.05;

    public static final double ARM_EXTENSTION_FOR_GRABING = 0.030089621707797;
    public static final double MIN_ROTATION_FOR_GRABING = -1.181165218353271;

    public static final double ARM_MAX_ROTATION_INTAKE_CLOSED = -1.059980750083923;

    public static final double ROTATION_MID_FOR_BEFORE_SCORING = Math.toRadians(13.535156213103427);
    public static final double ROTATION_MID_FOR_SCORING = Math.toRadians(7.72460941746863);
    public static final double EXTENSTION_FOR_MID_SCORING = 0.41;
    public static final double MAX_ROTATION_FOR_OPEN_GRIPPER = Math.toRadians(-69);
    
    public static final double ROTATION_MID_FOR_BEFORE_SCORING_FROM_THE_BACK = Math.toRadians(162.81641485413274);
    public static final double ROTATION_FOR_MID_SCORING_FROM_THE_BACK = Math.toRadians(170.97851490144268);
    public static final double EXTENSTION_FOR_MID_SCORING_FROM_THE_BACK = 0.237665064334869;
    
    public static final double ROTATION_FOR_LOW_SCORING = Math.toRadians(-16.0839855896399);
    public static final double EXTENSTION_FOR_LOW_SCORING = 0.214644178748131;

    public static final double ROTATION_FOR_LOW_SCORING_FROM_THE_BACK = 2.3886993885040283;
    public static final double EXTENSTION_FOR_LOW_SCORING_FROM_THE_BACK = ARM_EXTENSTION_FOR_GRABING;

    public static final double ROTATION_FOR_HP = 0.367883538722992;
    public static final double EXTENSTION_FOR_HP = 0;

    public static final double ROTATION_AFTER_HP = ROTATION_FOR_HP + 0.1;
}
