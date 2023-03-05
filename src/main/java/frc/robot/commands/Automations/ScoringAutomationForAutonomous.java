// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Intake.MiddleIntake;
import frc.robot.commands.Intake.OpenIntake;
import frc.robot.commands.gripper.GripperCloseCommand;
import frc.robot.commands.gripper.GripperControlCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmExtenstion;
import frc.robot.subsystems.arm.ArmRotation;
import frc.robot.subsystems.gripper.GripperConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoringAutomationForAutonomous extends SequentialCommandGroup {
  /** Creates a new ScoringAutomationForAutonomous. */
  private boolean startExtation() {
    return ArmRotation.getInstance().getRotation() > Math.toRadians(120);
  }
  private boolean atPoint() {
    return ArmExtenstion.getInstance().atPoint() 
      && ArmRotation.getInstance().atPoint();
  }
  public ScoringAutomationForAutonomous() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new OpenIntake(),
      new GripperCloseCommand(),
      new InstantCommand(
        () ->
        ArmRotation.getInstance().setSetpoint(ArmConstants.ROTATION_FOR_MID_SCORING_FROM_THE_BACK)
      ),
      new WaitUntilCommand(this::startExtation),
      new InstantCommand(
        () ->
        ArmExtenstion.getInstance().setSetpoint(ArmConstants.EXTENSTION_FOR_MID_SCORING_FROM_THE_BACK)
      ),
      new WaitUntilCommand(this::atPoint),
      new GripperControlCommand(GripperConstants.OPEN_POSITION)
    );
  }
}
