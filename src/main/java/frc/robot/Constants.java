// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final class FieldConstants {
    public static final double FIELD_WIDTH_METERS = 8.02;
    public static final double FIELD_LENGTH_METERS = 16.54;
    private static final double blueX = 1.80;
    private static final double redX = 14.74;
    private static final double A1Y = 0.5;
    private static final double A2Y = 1.10;
    private static final double A3Y = 1.60;
    private static final double B1Y = 2.2;
    private static final double B2Y = 2.76;
    private static final double B3Y = 3.30;
    private static final double C1Y = 3.90;
    private static final double C2Y = 4.42;
    private static final double C3Y = 4.92;
    public static final Pose2d[] ScoringPoses = {
      new Pose2d(
        blueX, C3Y, new Rotation2d(Math.toRadians(0))),
      new Pose2d(
        blueX, C2Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        blueX, C1Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        blueX, B3Y, new Rotation2d(Math.toRadians(0)) 
      ),
      new Pose2d(
        blueX, B2Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        blueX, B1Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        blueX, A3Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        blueX, A2Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        blueX, A1Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        redX, C3Y, new Rotation2d(Math.toRadians(180))),
      new Pose2d(
        redX, C2Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        redX, C1Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        redX, B3Y, new Rotation2d(Math.toRadians(180)) 
      ),
      new Pose2d(
        redX, B2Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        redX, B1Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        redX, A3Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        redX, A2Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        redX, A1Y, new Rotation2d(Math.toRadians(180))
      )
    }; // need to check
  }
  public static class pipeLines{
    public static final int apriltagPipeLine = 0; // TODO
    public static final int reflectiveLightPipeLine = 0; //TODO
  }
  public static class cammera {
    public static final double cammeraDisFromCenterInX = 0.2;
    public static final double cammeraDisFromCenterInY = 0.022;
    public static final double cammeraDisFromCenterInZ = 0.7;
    public static final double cammeraRoll = 0;
    public static final double cammeraPitch = Math.PI * 0.5;
    public static final double cammeraYaw = Math.PI;

  }

  public static class PortMap{

    public static class IntakePortMap {
      public final static  int IntakemotorID = 13;//TODO
      public final static int OpenAndCloseIntakeMotorID = 14;//TODO
  
      public final static int isCloseHallEffectChanelle = 1;//TODO
    }

    public static class SpinnerPortMap {
      public static final int stuckIRChanlle = 2;//TODO
      public static final int buttomIRChanlle = 3;//TODO
  
      public static final int motorID = 15;//TODO
    }

    public static class GripperPortMap {
      public static final int GripperMotorId = 16;//TODO
    }

    public static class ChameleonClimbPortMap {
      public static final int GRASPING_MOTOR_ID = 17;//TODO
    }

    public static class ArmPortMap {
      public static final int extenstionMotorID = 18;//TODO
      public static final int extenstionHallEffectID = 4;//TODO
      public static final int rotationMotorID = 19;//TODO
      public static final int rotationHallEffectID = 5;//TODO
    }

    public static class SwervePortMap {
      public final static int leftFrontAbsoluteEncoder = 9;
      public final static int leftFrontDriveID = 1;
      public final static int leftFrontTurningID = 2;
  
      public final static int leftBackAbsoluteEncoder = 10;
      public final static int leftBackDriveID = 3;
      public final static int leftBackTurningID = 4;
  
      public final static int rightFrontAbsoluteEncoder = 11;
      public final static int rightFrontDriveID = 6;
      public final static int rightFrontTurningID = 5;
  
      public final static int rightBackAbsoluteEncoder = 12;
      public final static int rightBackDriveID = 7;
      public final static int rightBackTurningID = 8;
    }
  }
}
