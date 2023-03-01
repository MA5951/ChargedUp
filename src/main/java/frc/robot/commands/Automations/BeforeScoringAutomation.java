// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Intake.CloseIntake;
import frc.robot.commands.Intake.MiddleIntake;
import frc.robot.subsystems.arm.ArmExtenstion;
import frc.robot.subsystems.arm.ArmRotation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BeforeScoringAutomation extends SequentialCommandGroup {
  /** Creates a new ScoringAutomation. */
  private boolean atPoint() {
    return ArmExtenstion.getInstance().atPoint() 
      && ArmRotation.getInstance().atPoint();
  }
  public BeforeScoringAutomation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MiddleIntake(),
      new setArmForMid(),
      new CloseIntake(),
      new WaitUntilCommand(this::atPoint),
      new setExtenstion()
    );
  }
}