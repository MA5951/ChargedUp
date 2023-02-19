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
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
  public static final class FieldConstants {
    public static final double FIELD_WIDTH_METERS = 8.02;
    public static final double FIELD_LENGTH_METERS = 16.54;
    private static final double BLUE_X = 1.80;
    private static final double RED_X = 14.74;
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
        BLUE_X, C3Y, new Rotation2d(Math.toRadians(0))),
      new Pose2d(
        BLUE_X, C2Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        BLUE_X, C1Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        BLUE_X, B3Y, new Rotation2d(Math.toRadians(0)) 
      ),
      new Pose2d(
        BLUE_X, B2Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        BLUE_X, B1Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        BLUE_X, A3Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        BLUE_X, A2Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        BLUE_X, A1Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        RED_X, C3Y, new Rotation2d(Math.toRadians(180))),
      new Pose2d(
        RED_X, C2Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        RED_X, C1Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        RED_X, B3Y, new Rotation2d(Math.toRadians(180)) 
      ),
      new Pose2d(
        RED_X, B2Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        RED_X, B1Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        RED_X, A3Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        RED_X, A2Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        RED_X, A1Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        BLUE_X, C3Y, new Rotation2d(Math.toRadians(180))),
      new Pose2d(
        BLUE_X, C2Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        BLUE_X, C1Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        BLUE_X, B3Y, new Rotation2d(Math.toRadians(180)) 
      ),
      new Pose2d(
        BLUE_X, B2Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        BLUE_X, B1Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        BLUE_X, A3Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        BLUE_X, A2Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        BLUE_X, A1Y, new Rotation2d(Math.toRadians(180))
      ),
      new Pose2d(
        RED_X, C3Y, new Rotation2d(Math.toRadians(0))),
      new Pose2d(
        RED_X, C2Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        RED_X, C1Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        RED_X, B3Y, new Rotation2d(Math.toRadians(0)) 
      ),
      new Pose2d(
        RED_X, B2Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        RED_X, B1Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        RED_X, A3Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        RED_X, A2Y, new Rotation2d(Math.toRadians(0))
      ),
      new Pose2d(
        RED_X, A1Y, new Rotation2d(Math.toRadians(0))
      )
    }; // need to check
  }
  public static class PipeLines{
    public static final int APRIL_TAG_PIPELINE = 0; // TODO
    public static final int REFLECTIVE_TAPE_PIPELINE = 0; //TODO
  }
  public static class Camera {
    public static final double CAMERA_DISTANCE_FROM_CENTER_IN_X = 0.2;
    public static final double CAMERA_DISTANCE_FROM_CENTER_IN_Y = 0.022;
    public static final double CAMERA_DISTANCE_FROM_CENTER_IN_Z = 0.7;
    public static final double CAMERA_ROLL = 0;
    public static final double CAMERA_PITCH = Math.PI * 0.5;
    public static final double CAMERA_YAW = Math.PI;

  }
}
