// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.gripper.GripperCloseCommand;
import frc.robot.commands.gripper.GripperControlCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmExtenstion;
import frc.robot.subsystems.arm.ArmRotation;
import frc.robot.subsystems.gripper.GripperConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabingAutomation extends SequentialCommandGroup {
  /** Creates a new GrabingCommand. */
  private boolean atPoint() {
    return ArmExtenstion.getInstance().atPoint() 
      && ArmRotation.getInstance().atPoint();
  }
  
  public GrabingAutomation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new GripperControlCommand(GripperConstants.BEFOR_GRABING_POSE),
      new InstantCommand(
        () -> ArmExtenstion.getInstance().setSetpoint(
          ArmConstants.ARM_EXTENSTION_FOR_GRABING)),
      new InstantCommand(
        () -> ArmRotation.getInstance().setSetpoint(
          ArmConstants.MIN_ROTATION_FOR_GRABING
        )),
    new WaitUntilCommand(this::atPoint),
    new GripperCloseCommand(),
    new InstantCommand(
      () -> ArmExtenstion.getInstance().setSetpoint(0)
    ),
    new InstantCommand(
      () -> ArmRotation.getInstance().setSetpoint(ArmConstants.ARM_ROTATION_START_POSE)
    ),
    new WaitUntilCommand(this::atPoint)
  );
  }
}
