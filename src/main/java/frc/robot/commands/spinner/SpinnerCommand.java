// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.spinner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner.Spinner;

public class SpinnerCommand extends CommandBase {
  /** Creates a new SpinnerCommand. */
  private Spinner spinnerSubsystem;

  // create a boolean variable to store if the reverse spin action has been completed and when it is completed set it to true
  private boolean reverseSpinComplete;
  private boolean reverseSpin;
  private double startEncoder;

  public SpinnerCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    spinnerSubsystem = Spinner.getInstance();

    addRequirements(spinnerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set reverse spin to false
    reverseSpin = false;
    reverseSpinComplete = false;
    startEncoder = spinnerSubsystem.getEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!spinnerSubsystem.getIR()) {
      spinnerSubsystem.setVelocity(-0.5);
    } 
    
    else if (!reverseSpin) {
      spinnerSubsystem.setVelocity(0.5);
      if (spinnerSubsystem.getEncoder() - startEncoder >= 42) {
        reverseSpin = true;
        reverseSpinComplete = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override 
  public void end(boolean interrupted) {
    spinnerSubsystem.setVoltage(0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return reverseSpinComplete;
  }
}
