// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakePosition;

public class CloseIntake extends CommandBase {
  /** Creates a new OpenIntake. */
  private IntakePosition intakePosition;
  public CloseIntake() {
    intakePosition = IntakePosition.getInstance();
    addRequirements(intakePosition);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakePosition.isAbleToClose()) {
      intakePosition.calculate(IntakeConstants.CLOSE_POSITION);
    } else {
      intakePosition.setPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakePosition.setPower(Math.cos(
      intakePosition.getPosition()) * IntakeConstants.KG);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakePosition.isClose();
  }
}
