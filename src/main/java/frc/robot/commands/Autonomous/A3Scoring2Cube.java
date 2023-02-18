// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Automations.AfterIntakeAutomation;
import frc.robot.commands.Automations.AfterScoringAutomation;
import frc.robot.commands.Automations.IntakeAutomation;
import frc.robot.commands.Automations.ScoringAutomationForAutonomous;
import frc.robot.commands.Intake.CloseIntake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class A3Scoring2Cube extends SequentialCommandGroup {
  /** Creates a new A3Scoring2Cube. */
  private SwerveDrivetrainSubsystem swerve = SwerveDrivetrainSubsystem.getInstance();
  private double time = 0; 
  private boolean shouldIOpenTheIntake() {
    return swerve.getTrajectory("from A3 to pickup 1").getMarkers()
    .get(0).timeSeconds >= Timer.getFPGATimestamp() - time;
  }
  public A3Scoring2Cube() {
    addCommands(
      new ScoringAutomationForAutonomous(),
      new ParallelCommandGroup(
        new AfterIntakeAutomation(),
        swerve.getAutonomousPathCommand("from A3 to pickup 1", true),
        new InstantCommand(() -> time = Timer.getFPGATimestamp()),
        new SequentialCommandGroup(
          new WaitUntilCommand(
            this::shouldIOpenTheIntake
          ),
          new IntakeAutomation(IntakeConstants.intakePower),
          new CloseIntake()
        )
      ),
      new ParallelCommandGroup(
        swerve.getAutonomousPathCommand("From pickup 1 to A2"),
        new AfterIntakeAutomation()
      ),
      new ScoringAutomationForAutonomous(),
      new AfterScoringAutomation()
    );
  }
}
