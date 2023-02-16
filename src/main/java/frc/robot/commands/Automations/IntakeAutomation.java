// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.OpenIntake;
import frc.robot.commands.spinner.SpinnerCommand;
import frc.robot.subsystems.arm.ArmConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAutomation extends SequentialCommandGroup {
  /** Creates a new IntakeAutomation. */
  public IntakeAutomation(double power) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetArmAutomation(0, ArmConstants.armRotationStartPose),
      new OpenIntake(),
      new ParallelDeadlineGroup(
        new IntakeCommand(power),
        new OpenIntake().repeatedly(),
        new IntakeCommand(power),
        new OpenIntake().repeatedly(),
        new SpinnerCommand()
      )
    );
  }
}
