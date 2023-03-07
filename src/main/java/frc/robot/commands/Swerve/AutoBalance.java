// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBallance. */
  private SwerveDrivetrainSubsystem swerve;
  private PIDController pid;
  private double time;
  private final double deley;
  private boolean wasAtPoint;

  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = SwerveDrivetrainSubsystem.getInstance();
    addRequirements(swerve);
    pid = new PIDController(0.025, 0, 0.007);
    pid.setTolerance(1);
    deley = 1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(0);
    wasAtPoint = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(
    pid.calculate(
      swerve.getPitch()),
      0, 0, false);
    

    if (pid.atSetpoint() && !wasAtPoint) {
      time = Timer.getFPGATimestamp();
      wasAtPoint = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint() && Timer.getFPGATimestamp() - time > deley;
  }
}
