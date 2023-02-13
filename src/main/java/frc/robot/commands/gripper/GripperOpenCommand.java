// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.gripper.GripperSubsystem;

public class GripperOpenCommand extends CommandBase {
  /** Creates a new GripperOpenCommand. */
  private GripperSubsystem gripperSubsystem;

  private double currentPosition;

  public GripperOpenCommand(GripperSubsystem gripperSubsystem) {
    this.gripperSubsystem = gripperSubsystem;
    addRequirements(gripperSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPosition = gripperSubsystem.getCurrentEncoderPosition();
    gripperSubsystem.setPower(GripperConstants.openingPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripperSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 
    Math.abs(
      currentPosition - GripperConstants.openPosition
      ) < GripperConstants.gripperTolerance;
  }
}
