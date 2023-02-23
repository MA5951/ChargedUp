// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.spinner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner.Spinner;
import frc.robot.subsystems.Spinner.SpinnerConstants;


public class SpinnerCommand extends CommandBase {
  /** Creates a new SpinnerCommand. */
  private Spinner spinnerSubsystem;

  private boolean isReversed;
  private boolean isTurnd3;

  public SpinnerCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    spinnerSubsystem = Spinner.getInstance();
    addRequirements(spinnerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isReversed = false;
    isTurnd3 = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    if(spinnerSubsystem.isGamePiceEntered()){
      if(!isTurnd3){
        if (spinnerSubsystem.getPosition() <= 500) {
        spinnerSubsystem.setPower(SpinnerConstants.REVERSED_SPINNER_SPEED);
        } else {
          isTurnd3 = true;
        }
      }
      else{
        spinnerSubsystem.resetEncoder();
        if(!spinnerSubsystem.isStuck() && !isReversed && !isTurnd3){
          spinnerSubsystem.resetEncoder();
        }
        if(!spinnerSubsystem.isStuck() && (spinnerSubsystem.getPosition() >= -500)){
          spinnerSubsystem.setPower(SpinnerConstants.SPINNER_SPEED);
          isReversed = true;
        }
        if(spinnerSubsystem.isStuck()){
          spinnerSubsystem.setPower(SpinnerConstants.REVERSED_SPINNER_SPEED);
        }
      }
      
    } else {
      spinnerSubsystem.setPower(SpinnerConstants.IDLE_REVERSE_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override 
  public void end(boolean interrupted) {
    spinnerSubsystem.setPower(0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return !spinnerSubsystem.isStuck() && spinnerSubsystem.getPosition() <= -500
    && spinnerSubsystem.isGamePiceEntered();
  }
}
