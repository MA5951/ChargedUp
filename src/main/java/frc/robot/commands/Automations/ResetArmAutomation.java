// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Intake.CloseIntake;
import frc.robot.commands.gripper.GripperControlCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmRotation;
import frc.robot.subsystems.gripper.GripperConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetArmAutomation extends SequentialCommandGroup {
  /** Creates a new AfterScoringAutomation. */
  public ResetArmAutomation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new SetArmAutomation(0, ArmConstants.ARM_ROTATION_START_POSE),
        new SequentialCommandGroup(
          new GripperControlCommand(GripperConstants.CLOSE_POSITION),
          new WaitUntilCommand(() -> ArmRotation.getInstance().getRotation() 
            <= ArmConstants.ARM_ROTATION_START_POSE + ArmConstants.ARM_ROTATION_TOLERANCE),
          new GripperControlCommand(GripperConstants.OPEN_POSITION)
        )
      ),
      new CloseIntake()
    );
  }
}
