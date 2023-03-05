// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.commands.ControlCommandInsubsystemControl;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Spinner.Spinner;
import frc.robot.commands.Swerve.DriveSwerveCommand;
// import frc.robot.subsystems.ChameleonClimb.ChameleonClimb;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePosition;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmExtenstion;
import frc.robot.subsystems.arm.ArmRotation;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    if (DriverStation.getAlliance() == Alliance.Red) {
      Constants.Camera.CAMERA_DISTANCE_FROM_CENTER_IN_X = -Constants.Camera.CAMERA_DISTANCE_FROM_CENTER_IN_X;
    }

    m_robotContainer = new RobotContainer();

    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(80, 60);
    
    // LED.getInstance();
    Intake.getInstance();
    IntakePosition.getInstance();
    Spinner.getInstance();
    ArmRotation.getInstance();
    ArmExtenstion.getInstance();
    // ChameleonClimb.getInstance();
    SwerveDrivetrainSubsystem.getInstance();
    GripperSubsystem.getInstance();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //RobotContainer.photonVision.update();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // SwerveDrivetrainSubsystem.getInstance().setNeutralMode(NeutralMode.Coast);
    IntakePosition.getInstance().setPower(0);
    GripperSubsystem.getInstance().setPower(0);
    ArmExtenstion.getInstance().setSetpoint(0);
    ArmRotation.getInstance().setSetpoint(ArmConstants.ARM_ROTATION_START_POSE);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    CommandScheduler.getInstance().setDefaultCommand(
      ArmExtenstion.getInstance(),
      new ControlCommandInsubsystemControl(
        ArmExtenstion.getInstance(), ArmExtenstion.getInstance()::getSetpoint)
    );

    CommandScheduler.getInstance().setDefaultCommand(
      ArmRotation.getInstance(),
      new ControlCommandInsubsystemControl(
        ArmRotation.getInstance(), ArmRotation.getInstance()::getSetPoint)
    );
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    SwerveDrivetrainSubsystem.getInstance().fixOdometry();

    CommandScheduler.getInstance().setDefaultCommand(
      SwerveDrivetrainSubsystem.getInstance(), new DriveSwerveCommand(
        RobotContainer.DRIVER_PS4_CONTROLLER::getLeftX, 
        RobotContainer.DRIVER_PS4_CONTROLLER::getLeftY,
        RobotContainer.DRIVER_PS4_CONTROLLER::getRightX));
    
    CommandScheduler.getInstance().setDefaultCommand(
      ArmExtenstion.getInstance(),
      new ControlCommandInsubsystemControl(
        ArmExtenstion.getInstance(), ArmExtenstion.getInstance()::getSetpoint)
    );

    CommandScheduler.getInstance().setDefaultCommand(
      ArmRotation.getInstance(),
      new ControlCommandInsubsystemControl(
        ArmRotation.getInstance(), ArmRotation.getInstance()::getSetPoint)
    );
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SwerveDrivetrainSubsystem.getInstance().updateOdometry();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
