// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import com.ma5951.utils.controllers.PIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBallance. */
  private SwerveDrivetrainSubsystem swerve;
  private ArmFeedforward feed;
  private PIDController pid;

  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = SwerveDrivetrainSubsystem.getInstance();
    addRequirements(swerve);
    feed = new ArmFeedforward(0, 0, 0);
    pid = new PIDController(0, 0, 0, 0, 0, -1, 1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(
    pid.calculate(
      swerve.getPitch()) + 
      feed.calculate((Math.PI / 2.0) - Math.toRadians(swerve.getPitch()), 0),
      0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
