// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ma5951.utils.MAShuffleboard;

public class SwerveDrivetrainSubsystem extends SubsystemBase {
  private static SwerveDrivetrainSubsystem swerve;

  public PIDController P_CONTROLLER_X;
  public PIDController P_CONTROLLER_Y;
  public PIDController thetaPID;

  public boolean isXReversed = true;
  public boolean isYReversed = true;
  public boolean isXYReversed = true;

  private static final String KP_X = "kp_x";
  private static final String KP_Y = "kp_y";
  private static final String theta_KP = "theta_KP";
  private static final String theta_KI = "theta_KI";
  private static final String theta_KD = "theta_KD";
  
  public final MAShuffleboard board;

  private final Translation2d frontLeftLocation = new Translation2d(
      -SwerveConstants.width / 2,
      SwerveConstants.length / 2);
  private final Translation2d frontRightLocation = new Translation2d(
      SwerveConstants.width / 2,
      SwerveConstants.length / 2);
  private final Translation2d rearLeftLocation = new Translation2d(
      -SwerveConstants.width / 2,
      -SwerveConstants.length / 2);
  private final Translation2d rearRightLocation = new Translation2d(
      SwerveConstants.width / 2,
      -SwerveConstants.length / 2);

  private final AHRS navx = new AHRS(Port.kMXP);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation,
      rearLeftLocation, rearRightLocation);

  private final static SwerveModule frontLeftModule = new SwerveModuleTalonFX(
      "frontLeftModule",
      SwervePortMap.leftFrontDriveID,
      SwervePortMap.leftFrontTurningID,
      SwervePortMap.leftFrontAbsoluteEncoder,
      SwerveConstants.frontLeftModuleIsDriveMotorReversed,
      SwerveConstants.frontLeftModuleIsTurningMotorReversed,
      SwerveConstants.frontLeftModuleIsAbsoluteEncoderReversed,
      SwerveConstants.frontLeftModuleOffsetEncoder);

  private final static SwerveModule frontRightModule = new SwerveModuleTalonFX(
      "frontRightModule",
      SwervePortMap.rightFrontDriveID,
      SwervePortMap.rightFrontTurningID,
      SwervePortMap.rightFrontAbsoluteEncoder,
      SwerveConstants.frontRightModuleIsDriveMotorReversed,
      SwerveConstants.frontRightModuleIsTurningMotorReversed,
      SwerveConstants.frontRightModuleIsAbsoluteEncoderReversed,
      SwerveConstants.frontRightModuleOffsetEncoder);

  private final static SwerveModule rearLeftModule = new SwerveModuleTalonFX(
      "rearLeftModule",
      SwervePortMap.leftBackDriveID,
      SwervePortMap.leftBackTurningID,
      SwervePortMap.leftBackAbsoluteEncoder,
      SwerveConstants.rearLeftModuleIsDriveMotorReversed,
      SwerveConstants.rearLeftModuleIsTurningMotorReversed,
      SwerveConstants.rearLeftModuleIsAbsoluteEncoderReversed,
      SwerveConstants.rearLeftModuleOffsetEncoder);

  private final static SwerveModule rearRightModule = new SwerveModuleTalonFX(
      "rearRightModule",
      SwervePortMap.rightBackDriveID,
      SwervePortMap.rightBackTurningID,
      SwervePortMap.rightBackAbsoluteEncoder,
      SwerveConstants.rearRightModuleIsDriveMotorReversed,
      SwerveConstants.rearRightModuleIsTurningMotorReversed,
      SwerveConstants.rearRightModuleIsAbsoluteEncoderReversed,
      SwerveConstants.rearRightModuleOffsetEncoder);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics,
    new Rotation2d(0), getSwerveModulePositions());

  private static SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      frontLeftModule.getPosition(), 
      frontRightModule.getPosition(),
      rearLeftModule.getPosition(),
      rearRightModule.getPosition()};
  }

  private static SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
      frontLeftModule.getState(), 
      frontRightModule.getState(),
      rearLeftModule.getState(),
      rearRightModule.getState()
    };
  }

  /** Creates a new DrivetrainSubsystem. */
  public SwerveDrivetrainSubsystem() {
    new Thread(
      ()-> {
        try {
          Thread.sleep(1000);
          resetNavx();
        } catch (Exception e) {
        }
      }
    ).start();
    this.board = new MAShuffleboard("swerve");

    board.addNum(KP_X, SwerveConstants.KP_X);

    P_CONTROLLER_X = new PIDController(board.getNum(KP_X), 0, 0);

    board.addNum(KP_Y, SwerveConstants.KP_Y);

    P_CONTROLLER_Y = new PIDController(board.getNum(KP_Y), 0, 0);

    board.addNum(theta_KP, SwerveConstants.theta_KP);
    board.addNum(theta_KI, SwerveConstants.theta_KI);
    board.addNum(theta_KD, SwerveConstants.theta_KD);

    thetaPID = new PIDController(board.getNum(theta_KP),
     board.getNum(theta_KI), board.getNum(theta_KD));
  }

  public void setNeutralMode(NeutralMode mode) {
    frontLeftModule.setNeutralMode(mode);
    frontRightModule.setNeutralMode(mode);
    rearRightModule.setNeutralMode(mode);
    rearLeftModule.setNeutralMode(mode);
  }

  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    rearLeftModule.resetEncoders();
    rearRightModule.resetEncoders();
  }

  public double getFusedHeading() {
    return navx.getYaw();
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(Math.toRadians(getFusedHeading()));
  }

  public void resetNavx() {
    navx.zeroYaw();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
  }

  public void stop() {
    frontLeftModule.stop();
    frontRightModule.stop();
    rearLeftModule.stop();
    rearRightModule.stop();
  }

  public void setModules(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxVelocity);
    rearLeftModule.setDesiredState(states[0]);
    frontLeftModule.setDesiredState(states[1]);
    rearRightModule.setDesiredState(states[2]);
    frontRightModule.setDesiredState(states[3]);
  }

  public void drive(double x, double y, double omega, boolean fieldRelative) {
    SwerveModuleState[] states = kinematics
        .toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, 
            new Rotation2d(Math.toRadians(-getFusedHeading())))
                : new ChassisSpeeds(x, y, omega));
    setModules(states);
  }

  public Command getAutonomousPathCommand(
    String pathName, boolean isFirst) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(pathName,
     new PathConstraints(
      SwerveConstants.maxVelocity, SwerveConstants.maxAcceleration));
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        if (isFirst) {
          resetOdometry(trajectory.getInitialPose());
        }
      }),
      new PPSwerveControllerCommand(
        trajectory,
        this::getPose,
        getKinematics(),
        P_CONTROLLER_X,
        P_CONTROLLER_Y,
        thetaPID,
        this::setModules,
        this),
      new InstantCommand(this::stop)
    );
  }

  public Command getAutonomousPathCommand(
    String pathName) {
    return getAutonomousPathCommand(pathName, false);
  }

  public static SwerveDrivetrainSubsystem getInstance() {
    if (swerve == null) {
      swerve = new SwerveDrivetrainSubsystem();
    }
    return swerve;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    P_CONTROLLER_X.setP(board.getNum(KP_X));
    P_CONTROLLER_Y.setP(board.getNum(KP_Y));
    thetaPID.setPID(board.getNum(theta_KP), board.getNum(theta_KI), board.getNum(theta_KD));
    odometry.update(getRotation2d(), getSwerveModulePositions());
    Logger.getInstance().recordOutput("Odometry", getPose());
    Logger.getInstance().recordOutput("SwervePositions", getSwerveModuleStates());

    board.addString("point", "(" + getPose().getX() + "," + getPose().getY() + ")");
    board.addNum("angle in degrees", getPose().getRotation().getDegrees());
    board.addNum("angle in radians", getPose().getRotation().getRadians());

    board.addNum("cancoder frontLeft", frontLeftModule.getAbsoluteEncoderPosition());
    board.addNum("cancoder frontRight", frontRightModule.getAbsoluteEncoderPosition());
    board.addNum("cancoder rearLeft", rearLeftModule.getAbsoluteEncoderPosition());
    board.addNum("cancoder rearRight", rearRightModule.getAbsoluteEncoderPosition());

    board.addNum("frontLeft angle", frontLeftModule.getTurningPosition());
    board.addNum("frontRight angle", frontRightModule.getTurningPosition());
    board.addNum("rearLeft angle", rearLeftModule.getTurningPosition());
    board.addNum("rearRight angle", rearRightModule.getTurningPosition());

    board.addNum("frontLeft drive pose", frontLeftModule.getDrivePosition());
    board.addNum("rearLeft drive pose", rearLeftModule.getDrivePosition());
    board.addNum("frontRight drive pose", frontRightModule.getDrivePosition());
    board.addNum("rearRight drive pose", rearRightModule.getDrivePosition());
  }
}