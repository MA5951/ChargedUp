package frc.robot.subsystems.arm;

public class ArmConstants {

    public static boolean isThereCone = false;

    public static final int kCPR = 4096;

    public static final double armExtenstionKn = 0; // TODO
    public static final double armExtenstionKp = 0; // TODO
    public static final double armExtenstionKi = 0; // TODO
    public static final double armExtenstionKd = 0; // TODO
    public static final double armExtenstionTolerance = 0; // TODO meters


    public static final double armRotationKp = 0; // TODO
    public static final double armRotationKi = 0; // TODO
    public static final double armRotationKd = 0; // TODO
    public static final double armRotationTolerance = 0; // TODO radians

    public static final double armRotationStartPose = 0; // TODO radians
    public static final double armRotationMaxPose = 0; // TODO radians
    public static final double armDistanceFromTheCenter = 0; // TODO
    public static final double armHight = 0; // TODO

    public static final double armExtenstionDiameterOfTheWheel = 0.0323342; // meters
    public static final double armMaxExtenstion = 0.42;

    public static final double armRotationGear = 1d / 400;

    public static final double neoArmRotationKV = 473;
    public static final double neoArmRotationKT =
        60 / (2 * Math.PI * neoArmRotationKV);
    public static final double armRotationkT = 
        neoArmRotationKT * armRotationGear;

    public static final double armMass = 6.8; // kg
    public static final double coneMass = 0.653; // kg
    public static final double armExtestionMass = 2.9; // kg

    public static final double minRotationForExtenstion = 0; // TODO radians
    public static final double minExtenstionForRotation = 0; // TODO meters
    public static final double minRotationForExtenstionSaftyBuffer = 0; // TODO radians

    public static final double rotationForMidScoring = 0; //TODO
    public static final double extenstionForMidScoring = 0; //TODO
    public static final double rotationForHP = 0; //TODO
    public static final double extenstionForHP = 0; //TODO
}
