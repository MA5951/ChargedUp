// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.gripper.GripperSubsystem;

public class GripperControlCommand extends CommandBase {
  /** Creates a new GripperControlCommand. */

  private double setPoint;
  private GripperSubsystem gripperSubsystem;

  public GripperControlCommand(double setPoint) {
    this.setPoint = setPoint;
    gripperSubsystem = GripperSubsystem.getInstance();
    addRequirements(gripperSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (setPoint == GripperConstants.MAX_POSE) {
      if (!gripperSubsystem.limitSwitch()) {
        gripperSubsystem.setSetpoint(-1);
        gripperSubsystem.setPower(0.7);
      }
    } else {
      gripperSubsystem.calculate(setPoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripperSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gripperSubsystem.atSetPoint() || (gripperSubsystem.limitSwitch()
    && setPoint == GripperConstants.MAX_POSE);
  }
}
