// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.spinner.SpinnerCommand;
import frc.robot.subsystems.Intake.Intake;

public class IntakeSpinnerCommand extends CommandBase {
  /** Creates a new IntakeSpinnerCommand. */
  Intake intake;
  IntakeCommand intakeCommand;
  SpinnerCommand spinnerCommand;
  public IntakeSpinnerCommand() {
    intake = Intake.getInstance();
    addRequirements(intake);
    intakeCommand = new IntakeCommand(0.5, 0.5);
    spinnerCommand = new SpinnerCommand();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeCommand.initialize();
    spinnerCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spinnerCommand.execute();
    if (!intake.isGamePiceEntered()) {
      intakeCommand.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeCommand.end(false);
    spinnerCommand.end(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
