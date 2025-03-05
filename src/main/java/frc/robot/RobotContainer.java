// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.CoralEndEffector;
import java.io.File;


import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(1
  );
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       m_drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/fleetbot"));
  private final CoralEndEffector m_CoralEndEffector = new CoralEndEffector();

  private final AlgaeManipulator m_AlgaeManipulator = new AlgaeManipulator();
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(m_drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox.getHID()::getYButtonPressed,
                                                                 driverXbox.getHID()::getAButtonPressed,
                                                                 driverXbox.getHID()::getXButtonPressed,
                                                                 driverXbox.getHID()::getBButtonPressed
                                                                 
                                                              
                                                                 );

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = m_drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = m_drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = m_drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
                                                                   () -> driverXbox.getLeftY(),
                                                                   () -> driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRightX())
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = m_drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveFieldOrientedAnglularVelocitySim = m_drivebase.driveFieldOriented(driveAngularVelocitySim);

  Command driveSetpointGenSim = m_drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  Trigger leftRigtAdjustTrigger = new Trigger(() -> (MathUtil.applyDeadband(driverXbox.getRawAxis(2)-driverXbox.getRawAxis(3), 0.1)>0.1));

  Trigger coralLoaded = m_CoralEndEffector.coralLoadedTrigger();
  
  Trigger AlgaeManipulatorIntake = new Trigger(driverXbox.leftTrigger());

  Trigger AlgaeManipulatorOutake = new Trigger(driverXbox.rightTrigger());

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(m_CoralEndEffector);
    SmartDashboard.putData(m_AlgaeManipulator);
    

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // (Condition) ? Return-On-True : Return-on-False
    m_drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
                                driveFieldOrientedAnglularVelocity :
                                driveFieldOrientedAnglularVelocitySim);
    m_CoralEndEffector.setDefaultCommand(m_CoralEndEffector.stopCommand());

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> m_drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(m_drivebase.sysIdDriveMotorCommand());
      driverXbox.axisGreaterThan(2, 0.1).or(driverXbox.axisGreaterThan(3, 0.1)).whileTrue(
       new RunCommand ( ()-> {m_drivebase.drive(new Translation2d(0,driverXbox.getLeftTriggerAxis()-driverXbox.getRightTriggerAxis()),0.0,false);
       })
      );
      operatorXbox.leftBumper().whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      operatorXbox.rightBumper().whileTrue(m_CoralEndEffector.outtakeCommand());
      operatorXbox.leftTrigger().whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      operatorXbox.rightTrigger().whileTrue(m_AlgaeManipulator.outtakeCommand());
      operatorXbox.povUp().whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      operatorXbox.povDown().whileTrue(m_AlgaeManipulator.goDownFunctionCommand());
     
      //driverXbox.leftTrigger().whileTrue(m_CoralEndEffector.intakeCommand());
   //   coralLoaded.onTrue(m_CoralEndEffector.stopCommand());
    }
    if (DriverStation.isTest())
    {
      m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      operatorXbox.x().whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      operatorXbox.y().whileTrue(m_drivebase.driveToDistanceCommand(1.0, 0.2));
      operatorXbox.start().onTrue((Commands.runOnce(m_drivebase::zeroGyro)));
      operatorXbox.back().whileTrue(m_drivebase.centerModulesCommand());
      operatorXbox.leftBumper().onTrue(Commands.none());
      operatorXbox.rightBumper().onTrue(Commands.none());
      operatorXbox.leftTrigger().onTrue(Commands.none());
      operatorXbox.rightTrigger().onTrue(Commands.none());
      operatorXbox.povUp().onTrue(Commands.none());
      operatorXbox.povDown().onTrue(Commands.none());
      
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(m_drivebase::zeroGyroWithAlliance)));
      operatorXbox.b().onTrue(m_CoralEndEffector.stopCommand());
      operatorXbox.x().whileTrue(m_CoralEndEffector.intakeCommand());
      operatorXbox.y().toggleOnTrue(m_AlgaeManipulator.lockAlgaeCommand());
      operatorXbox.start().whileTrue(Commands.none());
      operatorXbox.back().whileTrue(Commands.none());
      operatorXbox.rightBumper().onTrue(m_CoralEndEffector.outtakeAndStopCommand());
      operatorXbox.leftBumper().onTrue(m_CoralEndEffector.intakeWithSensorsCommand());
      operatorXbox.leftTrigger().onTrue(m_AlgaeManipulator.intakeCommand());
      operatorXbox.leftTrigger().onFalse(m_AlgaeManipulator.stopCommand());
      operatorXbox.rightTrigger().onTrue(m_AlgaeManipulator.outtakeCommand());
      operatorXbox.rightTrigger().onFalse(m_AlgaeManipulator.stopCommand());
      operatorXbox.povUp().onTrue(m_AlgaeManipulator.goUpFunctionCommand());
      operatorXbox.povUp().onFalse(m_AlgaeManipulator.stop2Command());
      operatorXbox.povDown().onTrue(m_AlgaeManipulator.goDownFunctionCommand());
      operatorXbox.povDown().onFalse(m_AlgaeManipulator.stop2Command());
    }
//check in with team about preference^ bumper preference for lock
    //driverXbox.x().onTrue(Commands.runOnce(m_drivebase::addFakeVisionReading));
      // driverXbox.b().whileTrue(
      //     m_drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //   
       // driverXbox.leftBumper().whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly())
  }


  /**2
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //


     //return m_drivebase.getAutonomousCommand("Right Start Auto");
    return new WaitCommand(1);
  }

  public void setMotorBrake(boolean brake)
  {
    m_drivebase.setMotorBrake(brake);
  }

public void periodic() {
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(m_CoralEndEffector);
}
}
