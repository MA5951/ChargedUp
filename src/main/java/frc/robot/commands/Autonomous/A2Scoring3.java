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
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class A2Scoring3 extends SequentialCommandGroup {
  /** Creates a new A2Scoring3. */
  private SwerveDrivetrainSubsystem swerve = SwerveDrivetrainSubsystem.getInstance();
  private double time = 0; 
  private boolean shouldIOpenTheIntake() {
    return swerve.getTrajectory("From A1 to pickup 2").getMarkers()
    .get(0).timeSeconds >= Timer.getFPGATimestamp() - time;
  }
  public A2Scoring3() {
    addCommands(
      new A2Scoring2(),
      new ParallelCommandGroup(
        swerve.getAutonomousPathCommand("from A1 to pickup 2",true),
        new InstantCommand(() -> time = Timer.getFPGATimestamp()),
        new SequentialCommandGroup(
          new WaitUntilCommand(
            this::shouldIOpenTheIntake
          ),
          new IntakeAutomation(),
          new CloseIntake()
        )
      ),
      new ParallelCommandGroup(
        swerve.getAutonomousPathCommand("from pickup 2 to A3"),
        new SpinnerAutomation()
      ),
      new ScoringAutomationForAutonomous(),
      new ResetArmAutomation()
    );
  }
}
