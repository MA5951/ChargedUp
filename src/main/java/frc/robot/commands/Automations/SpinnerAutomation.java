// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Intake.CloseIntake;
import frc.robot.commands.gripper.GripperCloseCommand;
import frc.robot.commands.spinner.SpinnerCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmExtenstion;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpinnerAutomation extends SequentialCommandGroup {
  /** Creates a new AfterIntakeAutomation. */
  public SpinnerAutomation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SpinnerCommand(),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(
            () -> ArmExtenstion.getInstance().setSetpoint(
              ArmConstants.ARM_EXTENSTION_FOR_GRABING)
          ),
          new WaitUntilCommand(
            ArmExtenstion.getInstance()::atPoint
          )
        ),
        new GripperCloseCommand(),
        new CloseIntake()
      )
    );
  }
}
