// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.ChameleonClimb;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.ChameleonClimb.ChameleonClimb;
// import frc.robot.subsystems.ChameleonClimb.ChameleonClimbConstants;

// public class ChameleonClimbCommand extends CommandBase {
//   /** Creates a new ChameleonClimbCommand. */
//   private ChameleonClimb chameleonClimb;
//   public ChameleonClimbCommand() {
//     chameleonClimb = ChameleonClimb.getInstance();
//     addRequirements(chameleonClimb);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (chameleonClimb.getCurrentStator() < ChameleonClimbConstants.BUMPER_IN_TOUCH_CURRENT_STATOR){
//       chameleonClimb.setMotor(ChameleonClimbConstants.CLOSING_ON_BUMPER_POWER);
//     } else {
//       chameleonClimb.setMotor(ChameleonClimbConstants.IN_TOUCH_WITH_BUMPER_CONTINUAL_POWER);
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     chameleonClimb.setMotor(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
