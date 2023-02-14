// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;


import com.ma5951.utils.commands.ControlCommandInsubsystemControl;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.MiddleIntake;
import frc.robot.commands.gripper.GripperOpenCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmExtenstion;
import frc.robot.subsystems.arm.ArmRotation;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoringAutomation extends SequentialCommandGroup {
  /** Creates a new ScoringAutomation. */
  public ScoringAutomation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        SwerveDrivetrainSubsystem.getInstance().getTelopPathCommand(),
        new SetArmAutomation(ArmConstants.extenstionForMidScoring, 
                              ArmConstants.rotationForMidScoring)
      ),
      new ParallelDeadlineGroup(
        new GripperOpenCommand(),
        new ControlCommandInsubsystemControl(
          ArmExtenstion.getInstance(),
          ArmConstants.extenstionForMidScoring).repeatedly(),
        new ControlCommandInsubsystemControl(
          ArmRotation.getInstance(),
          ArmConstants.rotationForMidScoring).repeatedly(),
          new MiddleIntake().repeatedly()
      )
    );
  }
}
