// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class YawPhotoVisionPid extends CommandBase {
  /** Creates a new yawPhotoVisionPid. */
  private SwerveDrivetrainSubsystem swerve;
  private PIDController pidController;
  private Supplier<Integer> getPipeline;
  
  public YawPhotoVisionPid(Supplier<Integer> getPipeline) {
    this.getPipeline = getPipeline;
    swerve = SwerveDrivetrainSubsystem.getInstance();
    pidController = new PIDController(0, 0, 0);
    pidController.setTolerance(0);
    pidController.setSetpoint(0);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.photonVision.changePipeline(getPipeline.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(0, pidController.calculate(
      RobotContainer.photonVision.getYaw()
    ), 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.photonVision.
      changePipeline(Constants.PipeLines.APRIL_TAG_PIPELINE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
