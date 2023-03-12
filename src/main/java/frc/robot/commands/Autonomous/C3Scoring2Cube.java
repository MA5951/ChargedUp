// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Automations.BeforeScoringAutomation;
import frc.robot.commands.Automations.GrabingAutomation;
import frc.robot.commands.Automations.IntakeAutomation;
import frc.robot.commands.Automations.ResetArmAutomationForAuto;
import frc.robot.commands.Intake.CloseIntake;
import frc.robot.commands.Intake.OpenIntake;
import frc.robot.commands.gripper.GripperCloseCommand;
import frc.robot.commands.gripper.GripperControlCommand;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakePosition;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmExtenstion;
import frc.robot.subsystems.arm.ArmRotation;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class C3Scoring2Cube extends SequentialCommandGroup {
  /** Creates a new A3Scoring2Cube. */
  private SwerveDrivetrainSubsystem swerve = SwerveDrivetrainSubsystem.getInstance();
  public C3Scoring2Cube() {
    addCommands(
      new ParallelDeadlineGroup(
        new WaitUntilCommand(
          () -> 
          IntakePosition.getInstance().getPosition() <
           IntakeConstants.MIDDLE_POSITION),
        new OpenIntake()
      ).alongWith(new GripperCloseCommand()),
      new InstantCommand(() -> 
      ArmRotation.getInstance().setSetpoint(
        ArmConstants.ROTATION_MID_FOR_BEFORE_SCORING)),
      new InstantCommand(() -> 
      ArmExtenstion.getInstance().setSetpoint(0)),
      new WaitUntilCommand(ArmRotation.getInstance()::atPoint),
      new InstantCommand(() -> 
      ArmExtenstion.getInstance().setSetpoint(
        ArmConstants.EXTENSTION_FOR_MID_SCORING)),
      new WaitUntilCommand(ArmExtenstion.getInstance()::atPoint),
      new GripperControlCommand(GripperConstants.OPEN_POSITION),
      new ParallelDeadlineGroup(
        swerve.getAutonomousPathCommand("From C3 to pickup 4", true)
        .alongWith(new ResetArmAutomationForAuto()),
        new IntakeAutomation()),
      swerve.getAutonomousPathCommand("From pickup 4 to C2").alongWith(
      new CloseIntake().andThen(
        new WaitCommand(0.6).andThen(new GrabingAutomation().andThen(new BeforeScoringAutomation())))),
      new GripperControlCommand(GripperConstants.OPEN_POSITION),
      new ResetArmAutomationForAuto()
    );
  }
}
