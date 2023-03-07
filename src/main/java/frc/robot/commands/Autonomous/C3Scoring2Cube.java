// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Automations.BeforeScoringAutomation;
import frc.robot.commands.Automations.GrabingAutomation;
import frc.robot.commands.Automations.IntakeAutomation;
import frc.robot.commands.Automations.ResetArmAutomationForAuto;
import frc.robot.commands.Intake.CloseIntake;
import frc.robot.commands.gripper.GripperCloseCommand;
import frc.robot.commands.gripper.GripperControlCommand;
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
      new GripperCloseCommand(),
      new BeforeScoringAutomation(),
      new GripperControlCommand(GripperConstants.MAX_POSE),
      swerve.getAutonomousPathCommand("From C3 to pickup 4", true)
      .alongWith(
        new ResetArmAutomationForAuto().andThen(new IntakeAutomation())),
      swerve.getAutonomousPathCommand("From pickup 4 to C2").alongWith(
      new CloseIntake().andThen(
        new GrabingAutomation().andThen(new BeforeScoringAutomation()))),
      new GripperControlCommand(GripperConstants.OPEN_POSITION),
      new ResetArmAutomationForAuto()
    );
  }
}
