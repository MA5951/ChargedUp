// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake.CloseIntake;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.gripper.GripperCloseCommand;
import frc.robot.commands.spinner.SpinnerCommand;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePosition;
import frc.robot.subsystems.Spinner.Spinner;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;


import com.ma5951.utils.PhotonVision;
import com.ma5951.utils.RobotConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.XboxController;
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
  public static final CommandPS4Controller DRIVER_PS4_CONTROLLER = 
    new CommandPS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

  public static final XboxController OPERATOR_XBOX_CONTROLLER = 
    new XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

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
      "ma5951",
      new Transform3d(
       new Translation3d(
        Constants.Camera.CAMERA_DISTANCE_FROM_CENTER_IN_X,
        Constants.Camera.CAMERA_DISTANCE_FROM_CENTER_IN_Y,
        Constants.Camera.CAMERA_DISTANCE_FROM_CENTER_IN_Z
       ), new Rotation3d(
        Constants.Camera.CAMERA_ROLL,
        Constants.Camera.CAMERA_PITCH,
        Constants.Camera.CAMERA_YAW
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

    //=======================================================
    // DRIVER_PS4_CONTROLLER.button(
    //   RobotConstants.PS5.Buttons.CROSS).whileTrue(
    //     new IntakeCommand()
    //     )
    //     .onFalse(
    //       new InstantCommand(
    //       () -> Intake.getInstance().setPower(0)
    //     ));

    //     DRIVER_PS4_CONTROLLER.button(
    //       RobotConstants.PS5.Buttons.TRIANGLE).whileTrue(
    //         new CloseIntake()
    //         )
    //         .onFalse(
    //           new InstantCommand(
    //           () -> IntakePosition.getInstance().setPower(0)
    //         ));

    DRIVER_PS4_CONTROLLER.button(
      RobotConstants.PS5.Buttons.CIRCLE).whileTrue(
        new SpinnerCommand()
      );

    //     DRIVER_PS4_CONTROLLER.button(
    //       RobotConstants.PS5.Buttons.R1).whileTrue(
    //         new GripperCloseCommand()
    //        )
    //         .onFalse(
    //           new InstantCommand(
    //           () -> GripperSubsystem.getInstance().setPower(0)
    //         ));

    //         DRIVER_PS4_CONTROLLER.button(
    //           RobotConstants.PS5.Buttons.L1).whileTrue(
    //             new GripperOpenCommand()
    //             )
    //             .onFalse(
    //               new InstantCommand(
      
    //               () -> GripperSubsystem.getInstance().setPower(0)
    //             ));
    //// ======================================

    DRIVER_PS4_CONTROLLER.button(RobotConstants.PS5.Buttons.TRIANGLE).whileTrue(
      new InstantCommand(SwerveDrivetrainSubsystem.getInstance()::updateOffset));
    DRIVER_PS4_CONTROLLER.R2().whileTrue(
      new InstantCommand(SwerveDrivetrainSubsystem.getInstance()::lowerVelocityTo40)).
    whileFalse(
      new InstantCommand(SwerveDrivetrainSubsystem.getInstance()::returnVelocityToNormal)
    );

    // DRIVER_PS4_CONTROLLER.L2().whileTrue(
    //   new InstantCommand(SwerveDrivetrainSubsystem.getInstance()::lowerVelocityTo22)
    // ).whileFalse(
    //   new InstantCommand(SwerveDrivetrainSubsystem.getInstance()::returnVelocityToNormal)
    // );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
