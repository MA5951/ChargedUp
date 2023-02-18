// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Automations.SpinnerAutomation;
import frc.robot.commands.Automations.ResetArmAutomation;
import frc.robot.commands.Automations.IntakeAutomation;
import frc.robot.commands.Automations.ScoringAutomationForAutonomous;
import frc.robot.commands.Intake.CloseIntake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class C1Scoring2 extends SequentialCommandGroup {
  private SwerveDrivetrainSubsystem swerve = SwerveDrivetrainSubsystem.getInstance();
  private double time = 0; 
  private boolean shouldIOpenTheIntake() {
    return swerve.getTrajectory("from C1 to pickup 4").getMarkers()
    .get(0).timeSeconds >= Timer.getFPGATimestamp() - time;
  }
  public C1Scoring2() {
    addCommands(
      new ScoringAutomationForAutonomous(),
      new ParallelCommandGroup(
        swerve.getAutonomousPathCommand("from C1 to pickup 4", true),
        new InstantCommand(() -> time = Timer.getFPGATimestamp()),
        new SequentialCommandGroup(
          new ResetArmAutomation(),
          new WaitUntilCommand(
            this::shouldIOpenTheIntake
          )
        ),
        new IntakeAutomation(IntakeConstants.intakePower),
        new CloseIntake()
      ),
      swerve.getAutonomousPathCommand("from pickup 4 to C3").alongWith(
        new SpinnerAutomation()
      ),
      new ScoringAutomationForAutonomous(),
      new ResetArmAutomation()
    );
  }
}

