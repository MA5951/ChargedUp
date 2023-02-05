// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.commands;

import java.util.function.Supplier;

import com.ma5951.utils.subsystem.ControlSubsystemInSubsystemControl;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlCommandInsubsystemControl extends CommandBase {
  /** Creates a new ControlCommand. */
  private ControlSubsystemInSubsystemControl subsystem;
  private Supplier<Double> setPoint;
  public ControlCommandInsubsystemControl(ControlSubsystemInSubsystemControl subsystem,
    Supplier<Double> setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.setPoint = setPoint;
    addRequirements(subsystem);
  }

  public ControlCommandInsubsystemControl(ControlSubsystemInSubsystemControl subsystem,
    double setPoint) {
  // Use addRequirements() here to declare subsystem dependencies.
  this.subsystem = subsystem;
  this.setPoint = () -> setPoint;
  addRequirements(subsystem);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.calculate(setPoint.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !subsystem.canMove();
  }
}
