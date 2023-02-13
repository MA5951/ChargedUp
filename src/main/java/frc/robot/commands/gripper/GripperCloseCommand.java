// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.gripper.GripperSubsystem;

public class GripperCloseCommand extends CommandBase {
  /** Creates a new GripperCommand. */
  // Declare the subsystem
  private GripperSubsystem gripperSubsystem;

  private double lastCurrent;
  private boolean tachedGamePice;

  public GripperCloseCommand() {
    gripperSubsystem = GripperSubsystem.getInstance();
    addRequirements(gripperSubsystem);
  }

  @Override
  public void initialize() {
    lastCurrent = 0;
    tachedGamePice = false;
  }

  @Override
  public void execute() {
    double motorAngleInRadians = gripperSubsystem.getCurrentEncoderPosition();
    double current = gripperSubsystem.getMotorCurrent();
    if (Math.abs(lastCurrent - current) >= GripperConstants.currentJump) {
      tachedGamePice = true;
    }
    if (tachedGamePice) {
      if (motorAngleInRadians <= GripperConstants.coneAngle) {
        gripperSubsystem.setPower(GripperConstants.conePower);
        ArmConstants.isThereCone = true;
      } else {
        gripperSubsystem.setPower(GripperConstants.cubePower);
      }
    } else {
          gripperSubsystem.setPower(GripperConstants.closingPower);
      }
      lastCurrent = current;
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return tachedGamePice;
  }
}
