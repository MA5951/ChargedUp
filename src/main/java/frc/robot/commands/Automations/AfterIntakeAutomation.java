// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.CloseIntake;
import frc.robot.commands.gripper.GripperCloseCommand;
import frc.robot.commands.spinner.SpinnerCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AfterIntakeAutomation extends SequentialCommandGroup {
  /** Creates a new AfterIntakeAutomation. */
  public AfterIntakeAutomation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SpinnerCommand(),
      new ParallelCommandGroup(
        new GripperCloseCommand(),
        new CloseIntake()
      )
    );
  }
}
