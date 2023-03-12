// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Automations.ResetArmAutomation;
import frc.robot.commands.Intake.CloseIntake;
import frc.robot.commands.Intake.OpenIntake;
import frc.robot.commands.Swerve.AutoBalance;
import frc.robot.commands.Swerve.LockModules;
import frc.robot.commands.gripper.GripperCloseCommand;
import frc.robot.commands.gripper.GripperControlCommand;
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
public class Score1AndClimb extends SequentialCommandGroup {
  /** Creates a new Score1. */

  private boolean atPoint() {
    return ArmExtenstion.getInstance().atPoint() 
      && ArmRotation.getInstance().atPoint();
  }

  public Score1AndClimb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new GripperCloseCommand(),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(
          () -> IntakePosition.getInstance().getPosition() < 
          IntakeConstants.MIDDLE_POSITION
        ), new OpenIntake()),
      new InstantCommand(
        () -> ArmRotation.getInstance().setSetpoint(
          ArmConstants.ROTATION_MID_FOR_SCORING)
      ),
      new InstantCommand(
        () -> ArmExtenstion.getInstance().setSetpoint(
          ArmConstants.EXTENSTION_FOR_MID_SCORING
        )
      ),
      new CloseIntake(),
      new WaitUntilCommand(this::atPoint),
      new GripperControlCommand(GripperConstants.MAX_POSE),
      new ResetArmAutomation(),
      new ParallelDeadlineGroup(SwerveDrivetrainSubsystem.getInstance()
      .getAutonomousPathCommand("climb", true),
      new CloseIntake().repeatedly()
      ),
      new AutoBalance(),
      new LockModules()
    );
  }
}
