// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Automations.ResetArmAutomation;
import frc.robot.commands.Automations.ScoringAutomationForAutonomous;
import frc.robot.commands.Intake.CloseIntake;
import frc.robot.commands.Swerve.LockModules;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score1AndClimb extends SequentialCommandGroup {
  /** Creates a new Score1. */
  public Score1AndClimb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoringAutomationForAutonomous(),
      new ResetArmAutomation(),
      SwerveDrivetrainSubsystem.getInstance()
        .getAutonomousPathCommand("climb", true).alongWith(new CloseIntake().repeatedly()),
      new LockModules()
    );
  }
}