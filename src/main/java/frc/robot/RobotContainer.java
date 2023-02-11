// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autonomous.A1Scoring2;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

import com.ma5951.utils.PhotonVision;
import com.ma5951.utils.RobotConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static CommandPS4Controller COMMAND_PS4_CONTROLLER = 
    new CommandPS4Controller(OperatorConstants.kDriverControllerPort);

  private static AprilTagFieldLayout aprilTagFieldLayout;

  public static PhotonVision photonVision;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    try {
      aprilTagFieldLayout = 
      AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      System.err.println(e);
    }

    photonVision  = new PhotonVision(
      "ma5951", 0.4, 0,
      new Transform3d(
       new Translation3d(
        0.377, -0.017, 0.3225
       ), new Rotation3d(
        0, 0, Math.toRadians(200)
       )),
      aprilTagFieldLayout
       );
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    COMMAND_PS4_CONTROLLER.button(RobotConstants.Y).whileTrue(
      new InstantCommand(SwerveDrivetrainSubsystem.getInstance()::updateOffset));
    COMMAND_PS4_CONTROLLER.R2().whileTrue(
      new InstantCommand(SwerveDrivetrainSubsystem.getInstance()::lowerVelocityTo40)).
    whileFalse(
      new InstantCommand(SwerveDrivetrainSubsystem.getInstance()::returnVelocityToNormal)
    );

    COMMAND_PS4_CONTROLLER.button(RobotConstants.B).whileTrue(
      new InstantCommand(
        () -> {
          SwerveDrivetrainSubsystem.getInstance().setTuRL(0.7);
        }
      )
    ).whileFalse(
      new InstantCommand(
        () -> {
          SwerveDrivetrainSubsystem.getInstance().setTuRL(0);
        }
    ));

    COMMAND_PS4_CONTROLLER.button(RobotConstants.X).whileTrue(
      new InstantCommand(
        () -> {
          SwerveDrivetrainSubsystem.getInstance().setTuRR(0.7);
        }
      )
    ).whileFalse(
      new InstantCommand(
        () -> {
          SwerveDrivetrainSubsystem.getInstance().setTuRR(0);
        }
    ));

    COMMAND_PS4_CONTROLLER.L2().whileTrue(
      new InstantCommand(SwerveDrivetrainSubsystem.getInstance()::lowerVelocityTo22)
    ).whileFalse(
      new InstantCommand(SwerveDrivetrainSubsystem.getInstance()::returnVelocityToNormal)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new A1Scoring2();
  }
}
