// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.MiddleIntake;
import frc.robot.commands.gripper.GripperControlCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.gripper.GripperConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoringAutomationForAutonomous extends SequentialCommandGroup {
  /** Creates a new ScoringAutomationForAutonomous. */
  public ScoringAutomationForAutonomous() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetArmAutomation(ArmConstants.EXTENSTION_FOT_MID_SCORING_FROM_THE_BACK, 
                              ArmConstants.ROTATION_MID_FOR_BEFORE_SCORING_FROM_THE_BACK),
      new SetArmAutomation(ArmConstants.EXTENSTION_FOT_MID_SCORING_FROM_THE_BACK, 
                            ArmConstants.ROTATION_FOR_MID_SCORING_FROM_THE_BACK),
      new ParallelDeadlineGroup(
        new GripperControlCommand(GripperConstants.OPEN_POSITION),
        new MiddleIntake().repeatedly()
      )
    );
  }
}
