// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

import com.ma5951.utils.subsystem.MotorSubsystem;

public class MotorCommand extends CommandBase {
  /** Creates a new MAMotorCommands. */
  private MotorSubsystem subsystem;
  private Supplier<Double> power;

  public MotorCommand(MotorSubsystem subsystem, Supplier<Double> power) {
    this.subsystem = subsystem;
    this.power = power;
    addRequirements(subsystem);
  }

  public MotorCommand(MotorSubsystem subsystem, double power) {
    this(subsystem, () -> power);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    subsystem.setPower(power.get());
  }

  @Override
  public boolean isFinished(){
    return !subsystem.canMove();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setPower(0);
  }
}
