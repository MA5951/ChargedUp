// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Spinner.Spinner;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  private Intake intake;
  public IntakeCommand() {
    intake = Intake.getInstance();
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setPower(IntakeConstants.INTAKE_POWER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Spinner.getInstance().isGamePiceEntered();
    return false;
  }
}
