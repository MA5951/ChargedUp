// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakePosition;

public class MiddleIntake extends CommandBase {
  /** Creates a new MiddleIntake. */
  private IntakePosition intakePosition;
  public MiddleIntake() {
    intakePosition = IntakePosition.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakePosition);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakePosition.calculate(IntakeConstants.MIDDLE_POSITION);
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
    return intakePosition.isMiddle();
  }
}
