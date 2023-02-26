// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  private SwerveDrivetrainSubsystem swerve;
  private Supplier<Double> leftSupplier;
  private Supplier<Double> rightSupplier;
  public TankDrive(Supplier<Double> leftSupplier, Supplier<Double> rightSupplier) {
    swerve = SwerveDrivetrainSubsystem.getInstance();
    this.leftSupplier = leftSupplier;
    this.rightSupplier = rightSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = leftSupplier.get() * 
      SwerveDrivetrainSubsystem.getInstance().maxVelocity;
    double rightSpeed = rightSupplier.get() *
      SwerveDrivetrainSubsystem.getInstance().maxVelocity;
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = new SwerveModuleState(
      leftSpeed,
      new Rotation2d(0)
    );
    states[1] = new SwerveModuleState(
      leftSpeed,
      new Rotation2d(0)
    );
    states[2] = new SwerveModuleState(
      rightSpeed,
      new Rotation2d(0)
    );
    states[3] = new SwerveModuleState(
      rightSpeed,
      new Rotation2d(0)
    );
    swerve.setModules(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
