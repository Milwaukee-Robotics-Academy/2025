// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.concurrent.Semaphore;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import  edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.CoralEndEffector;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandJoystick driverJoystick = new CommandJoystick(0);
  final CommandXboxController operatorXboxController = new CommandXboxController(1);
  final CommandJoystick driverJoystick2 = new CommandJoystick(2);
  final CommandXboxController driverXboxController = new CommandXboxController(0);
 // final CommandJoystick operatorJoystick = new CommandJoystick(1);
  
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/fleetbot"));
  private final CoralEndEffector m_CoralEndEffector = new CoralEndEffector();
  private final AlgaeManipulator m_AlgaeManipulator = new AlgaeManipulator();
 // private final Vision m_vision;
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */

   //Dual Sticks
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
      () -> driverJoystick.getY(),
      () -> driverJoystick.getX())
      .withControllerRotationAxis(() -> driverJoystick2.getZ())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true); 


      //Controller
      /*  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
      () -> driverXboxController.getLeftY() * -1,
      () -> driverXboxController.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXboxController.getLeftTriggerAxis() - driverXboxController.getRightTriggerAxis())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);*/

  Command driveFieldOrientedAnglularVelocity = m_drivebase.driveFieldOriented(driveAngularVelocity);

  //Dual Sticks
  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
      () -> driverJoystick.getY(),
      () -> driverJoystick.getX())
      .withControllerRotationAxis(() -> driverJoystick2.getZ())
      .deadband(OperatorConstants.DEADBAND)
      .allianceRelativeControl(true); 


  //Controller
  /*   SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
      () -> driverXboxController.getLeftY() * -1,
      () -> driverXboxController.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXboxController.getLeftTriggerAxis() - driverXboxController.getRightTriggerAxis())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);*/

  Command driveFieldOrientedAnglularVelocitySim = m_drivebase.driveFieldOriented(driveAngularVelocitySim);

  private SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("outtakeAndStop", m_CoralEndEffector.outtakeAndStopCommand());
    NamedCommands.registerCommand("IntakewithSensor", m_CoralEndEffector.intakeWithSensorsCommand());
    NamedCommands.registerCommand("IntakeAndStop", m_CoralEndEffector.IntakeAndStopCommand());
    NamedCommands.registerCommand("Outtake", m_CoralEndEffector.outtakeCommand());
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(m_CoralEndEffector);
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
   // m_vision = new Vision();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // (Condition) ? Return-On-True : Return-on-False
    m_drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocitySim);
    m_CoralEndEffector.setDefaultCommand(m_CoralEndEffector.stopCommand());
    m_AlgaeManipulator.setDefaultCommand(m_AlgaeManipulator.stop2Command());

    if (Robot.isSimulation()) {
     driverJoystick.povUp().onTrue(Commands.runOnce(() -> m_drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverJoystick.povDown().whileTrue(m_drivebase.sysIdDriveMotorCommand());
       driverJoystick.axisGreaterThan(2, 0.1).or(driverJoystick.axisGreaterThan(3, 0.1)).whileTrue(
          new RunCommand(() -> {
            m_drivebase.drive(new Translation2d(0, driverJoystick.getX() -+ driverJoystick.getY()),
                0.0, false);
          })); 
    } else {
     driverJoystick2.button(1).onTrue((Commands.runOnce(m_drivebase::zeroGyroWithAlliance)));
     driverXboxController.a().onTrue((Commands.runOnce(m_drivebase::zeroGyroWithAlliance)));
     operatorXboxController.leftTrigger().whileTrue(m_CoralEndEffector.intakeCommand());
     operatorXboxController.rightTrigger().whileTrue(m_CoralEndEffector.outtakeCommand());
     operatorXboxController.b().onTrue(m_CoralEndEffector.operationNOCommand());
     operatorXboxController.a().onTrue(m_CoralEndEffector.stopCommand());
      operatorXboxController.leftTrigger().whileTrue(m_CoralEndEffector.intakeCommand());
      operatorXboxController.rightTrigger().whileTrue(m_CoralEndEffector.outtakeCommand());
      operatorXboxController.povUp().whileTrue(m_AlgaeManipulator.goUpFunctionCommand());
      operatorXboxController.povDown().whileTrue(m_AlgaeManipulator.goDownFunctionCommand());
      operatorXboxController.rightBumper().onTrue(m_CoralEndEffector.outtakeAndStopCommand());
      operatorXboxController.leftBumper().onTrue(m_CoralEndEffector.intakeWithSensorsCommand());
      operatorXboxController.x().whileTrue(m_AlgaeManipulator.intakeCommand());
      operatorXboxController.a().whileTrue(m_AlgaeManipulator.outtakeCommand());
      operatorXboxController.x().onFalse(m_AlgaeManipulator.stopCommand());
    //  autoChooser = AutoBuilder.buildAutoChooser();
   // SmartDashboard.putData("Auto Chooser", autoChooser);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return autoChooser.getSelected();
return new WaitCommand(0);
  }

  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }

  public void periodic() {
    m_drivebase.periodic();
   // m_vision.updatePoseEstimation(m_drivebase.getSwerveDrive());
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(m_CoralEndEffector);
    SmartDashboard.putData(m_AlgaeManipulator);
    SmartDashboard.putData(m_drivebase);
  }
}
