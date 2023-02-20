// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmRotation;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreArmRotation extends InstantCommand {
  private ArmRotation armRotation;
  
  public ScoreArmRotation() {
    // Use addRequirements() here to declare subsystem dependencies.
    armRotation = ArmRotation.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean isReversed = false;
    if (DriverStation.getAlliance() == Alliance.Red) {
      if (Math.abs(
          SwerveDrivetrainSubsystem.getInstance().getPose().getRotation().getDegrees()) > 90) {
        isReversed = true;
      }
    } else {
      if (Math.abs(
          SwerveDrivetrainSubsystem.getInstance().getPose().getRotation().getDegrees()) < 90) {
        isReversed = true;
      }
    }

    double rotationSetpoint = isReversed ? ArmConstants.ROTATION_FOR_MID_SCORING_FROM_THE_BACK : ArmConstants.ROTATION_MID_FOR_SCORING;
    armRotation.setSetpoint(rotationSetpoint);
  }
}