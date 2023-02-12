// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Spinner.Spinner;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  private Intake intake;
  double lowerPower, upperPower;
  public IntakeCommand(double lowerPower, double upperPower) {
    this.lowerPower = lowerPower;
    this.upperPower = upperPower;
    intake = Intake.getInstance();
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setLowerMotorVelocity(lowerPower);
    intake.setUpperMotorVelocity(upperPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setLowerMotorVelocity(0);
    intake.setUpperMotorVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Spinner.getInstance().isGamePiceEntered();
  }
}
