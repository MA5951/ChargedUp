// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Automations.ResetArmAutomation;
import frc.robot.commands.Automations.ScoringAutomationForAutonomous;
import frc.robot.commands.Swerve.AutoBalance;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class B1Climb extends SequentialCommandGroup {
  /** Creates a new B1Climd. */
  private SwerveDrivetrainSubsystem swerve = SwerveDrivetrainSubsystem.getInstance();
  public B1Climb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoringAutomationForAutonomous(),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          swerve.getAutonomousPathCommand("from B1 to climb", true),
          new AutoBalance()
        ),
        new ResetArmAutomation()
      )
      
    );
  }
}
