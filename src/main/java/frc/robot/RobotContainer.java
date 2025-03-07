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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
  //final         CommandXboxController operatorXbox = new CommandXboxController(1);
  
  //--------------------------------------------------------------------------------------+
  
  final          CommandJoystick driverJoystick = new CommandJoystick(0);
  final          CommandJoystick operatorJoystick = new CommandJoystick(1);
   //Put Operator Joystick Here...


  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       m_drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/fleetbot"));
  private final CoralEndEffector m_CoralEndEffector = new CoralEndEffector();
  private final AlgaeManipulator m_AlgaeManipulator = new AlgaeManipulator();
  ////////private final AlgaeManipulator m_AlgaeManipulator = new AlgaeManipulator();
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
 
 /*  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(m_drivebase,
                                                               () -> -MathUtil.applyDeadband(driverJoystick.getLeftY(),
                                                                                              OperatorConstants.LEFT_Y_DEADBAND),
                                                                () -> -MathUtil.applyDeadband(driverJoystick.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverJoystick.getTwistChannel(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox.getHID()::getAButtonPressed,
                                                                 driverJoystick.getHID().button(2),
                                                                 driverJoystick.getHID().button(3),
                                                                 driverJoystick.getHID().button(4)
  );
                                                                 
*/
  
                                                                                                   
                                                                 

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
                                                                () -> driverJoystick.getX() * -1,
                                                                () -> driverJoystick.getY() * -1)
                                                            .withControllerRotationAxis(driverJoystick::getTwistChannel)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverJoystick::getX,
                                                                                             driverJoystick::getY)
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
                                                                   () -> driverJoystick.getY(),
                                                                   () -> driverJoystick.getX())
                                                               .withControllerRotationAxis(() -> driverJoystick.getTwistChannel())
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverJoystick.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverJoystick.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = m_drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveFieldOrientedAnglularVelocitySim = m_drivebase.driveFieldOriented(driveAngularVelocitySim);

  Command driveSetpointGenSim = m_drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  //CommandJoystick leftRigtAdjustTrigger = new CommandJoystick(() -> (MathUtil.applyDeadband(driverJoystick.getRawAxis(2)-driverJoystick.getRawAxis(3), 0.1)>0.1));

  Trigger coralLoaded = m_CoralEndEffector.coralLoadedTrigger();
  
  Trigger AlgaeManipulatorIntake = new Trigger(driverJoystick.button(7));

  Trigger AlgaeManipulatorOutake = new Trigger(driverJoystick.button(8));

  
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
     // driverXbox.start().onTrue(Commands.runOnce(() -> m_drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverJoystick.button(11).whileTrue(m_drivebase.sysIdDriveMotorCommand());
     // driverJoystick.axisGreaterThan(2, 0).or(driverJoystick.axisGreaterThan(3, 0.1)).whileTrue(
       //new RunCommand ( ()-> {m_drivebase.drive(new Translation2d(0,driverXbox.getLeftTriggerAxis()-driverXbox.getRightTriggerAxis()),0.0,false);
      // })
      //);
      operatorJoystick.button(2).whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      operatorJoystick.button(1).whileTrue(m_CoralEndEffector.outtakeCommand());
      operatorJoystick.button(7).whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      operatorJoystick.button(8).whileTrue(m_AlgaeManipulator.outtakeCommand());
      operatorJoystick.axisGreaterThan(2, 0).whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      operatorJoystick.axisLessThan(2, 0).whileTrue(m_AlgaeManipulator.goDownFunctionCommand());
     
      //driverXbox.leftTrigger().whileTrue(m_CoralEndEffector.intakeCommand());
   //   coralLoaded.onTrue(m_CoralEndEffector.stopCommand());
    }
    if (DriverStation.isTest())
    {
      m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      operatorJoystick.button(3).whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      operatorJoystick.button(4).whileTrue(m_drivebase.driveToDistanceCommand(1.0, 0.2));
      //operatorXbox.start().onTrue((Commands.runOnce(m_drivebase::zeroGyro)));
      //operatorXbox.back().whileTrue(m_drivebase.centerModulesCommand());
      operatorJoystick.button(2).onTrue(Commands.none());
      operatorJoystick.button(1).onTrue(Commands.none());
      operatorJoystick.button(7).onTrue(Commands.none());
      operatorJoystick.button(8).onTrue(Commands.none());
      operatorJoystick.axisGreaterThan(2, 0).onTrue(Commands.none());
      operatorJoystick.axisLessThan(2, 0).whileTrue(Commands.none());
     
    } else
    {
      driverJoystick.button(12).onTrue((Commands.runOnce(m_drivebase::zeroGyroWithAlliance)));
      operatorJoystick.button(9).onTrue(m_CoralEndEffector.stopCommand());
      operatorJoystick.button(2).whileTrue(m_CoralEndEffector.intakeCommand());
      operatorJoystick.button(4).toggleOnTrue(m_AlgaeManipulator.lockAlgaeCommand());
      operatorJoystick.button(5).onTrue(m_CoralEndEffector.outtakeAndStopCommand());
      operatorJoystick.button(6).onTrue(m_CoralEndEffector.intakeWithSensorsCommand());
      operatorJoystick.button(7).onTrue(m_AlgaeManipulator.intakeCommand());
      operatorJoystick.button(7).onFalse(m_AlgaeManipulator.stopCommand());
      operatorJoystick.button(8).onTrue(m_AlgaeManipulator.outtakeCommand());
      operatorJoystick.button(8).onFalse(m_AlgaeManipulator.stopCommand());
      operatorJoystick.axisGreaterThan(2, 0).onTrue(m_AlgaeManipulator.goUpFunctionCommand());
      operatorJoystick.axisGreaterThan(2, 0).onFalse(m_AlgaeManipulator.stop2Command());
      operatorJoystick.axisLessThan(2, 0).onTrue(m_AlgaeManipulator.goDownFunctionCommand());
      operatorJoystick.axisLessThan(2, 0).onFalse(m_AlgaeManipulator.stop2Command());
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


     return m_drivebase.getAutonomousCommand("Curvy Auto");
    //return new WaitCommand(1);
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
