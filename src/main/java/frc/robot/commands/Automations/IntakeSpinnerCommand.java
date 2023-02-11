// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePosition;
import frc.robot.subsystems.Spinner.Spinner;

public class IntakeSpinnerCommand extends CommandBase {
  /** Creates a new IntakeSpinnerCommand. */
  Intake intake;
  Spinner spinner;
  IntakeCommand intakeCommand;
  public IntakeSpinnerCommand() {
    intake = Intake.getInstance();
    spinner = Spinner.getInstance();
    addRequirements(intake, spinner);
    intakeCommand = new IntakeCommand(0.5, 0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!intake.isGamePiceEntered()) {
      intakeCommand.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
