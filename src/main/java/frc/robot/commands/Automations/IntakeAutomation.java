// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.OpenIntake;
import frc.robot.commands.gripper.GripperCloseCommand;
import frc.robot.commands.gripper.GripperOpenCommand;
import frc.robot.commands.spinner.SpinnerCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmRotation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAutomation extends SequentialCommandGroup {
  /** Creates a new IntakeAutomation. */
  public IntakeAutomation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new SetArmAutomation(ArmConstants.ARM_EXTENSTION_FOR_GRABING, ArmConstants.ARM_ROTATION_START_POSE),
        new SequentialCommandGroup(
          new GripperCloseCommand(),
          new WaitUntilCommand(() -> ArmRotation.getInstance().getRotation() <= 
            ArmConstants.ARM_ROTATION_START_POSE + ArmConstants.ARM_ROTATION_TOLERANCE),
          new GripperOpenCommand()
        )
      ),
      new OpenIntake(),
      new ParallelDeadlineGroup(
        new IntakeCommand(),
        new OpenIntake().repeatedly(),
        new SpinnerCommand()
      )
    );
  }
}
