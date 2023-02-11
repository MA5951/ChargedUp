// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.OpenIntake;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePosition;
import frc.robot.subsystems.Spinner.Spinner;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAutomation extends SequentialCommandGroup {
  /** Creates a new IntakeAutomation. */
  private Intake intake = Intake.getInstance();
  private IntakePosition intakePosition = IntakePosition.getInstance();
  private Spinner spinner = Spinner.getInstance();
  public IntakeAutomation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new OpenIntake(),
      new IntakeCommand(0.5, 0.5),
    );
  }
}
