// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.AlgaeEndEffector;
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
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/fleetbot"));
  private final CoralEndEffector m_CoralEndEffector = new CoralEndEffector();
  private final AlgaeEndEffector m_AlgaeEndEffector = new AlgaeEndEffector();
  private final Vision m_vision;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);


  Command driveFieldOrientedAnglularVelocity = m_drivebase.driveFieldOriented(driveAngularVelocity);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY(),
      () -> driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .allianceRelativeControl(true);

  SwerveInputStream driveRobotCentricInputStream = driveAngularVelocity.copy()
  .robotRelative(true)
  .allianceRelativeControl(false);

  Command driveRobotCentric = m_drivebase.driveFieldOriented(driveRobotCentricInputStream);
  Command driveFieldOrientedAnglularVelocitySim = m_drivebase.driveFieldOriented(driveAngularVelocitySim);

  Trigger tooCloseToReef = m_drivebase.tooCloseToReefTrigger();

  private SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("outtakeAndStop", m_CoralEndEffector.outtakeAndStopCommand());
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(m_CoralEndEffector);
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    m_vision = new Vision();   
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
    m_AlgaeEndEffector.setDefaultCommand(m_AlgaeEndEffector.stopCommand());

    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(Commands.runOnce(() -> m_drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(m_drivebase.sysIdDriveMotorCommand());
      driverXbox.axisGreaterThan(2, 0.1).or(driverXbox.axisGreaterThan(3, 0.1)).whileTrue(
          new RunCommand(() -> {
            m_drivebase.drive(new Translation2d(0, driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis()),
                0.0, false);
          }));
      driverXbox.leftBumper().whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      driverXbox.rightBumper().whileTrue(m_CoralEndEffector.outtakeCommand());
      driverXbox.leftTrigger().whileTrue(m_CoralEndEffector.intakeCommand());
    } else {
      driverXbox.b().whileTrue(m_CoralEndEffector.outtakeCommand());
      driverXbox.y().whileTrue(m_CoralEndEffector.intakeWithSensorsCommand());
      driverXbox.x().whileTrue(m_CoralEndEffector.spitbackCommand());
     // driverXbox.a().whileTrue(m_drivebase.driveToPose(new Pose2d(17.18, 1.15, Rotation2d.fromDegrees(143.03))));
      driverXbox.start().whileTrue(Commands.runOnce(m_drivebase::zeroGyroWithAlliance));
      driverXbox.a().whileTrue(driveRobotCentric);
    
      // driverXbox.leftBumper().whileTrue(Commands.runOnce(m_drivebase::lock,
      // m_drivebase).repeatedly());
      driverXbox.rightBumper().whileTrue(m_AlgaeEndEffector.goDownFunctionCommand());
      driverXbox.leftBumper().whileTrue(m_AlgaeEndEffector.goUpFunctionCommand());
      driverXbox.leftTrigger().whileTrue(m_AlgaeEndEffector.intakeCommand());
      driverXbox.rightTrigger().whileTrue(m_AlgaeEndEffector.outtakeCommand());
      tooCloseToReef.whileTrue(Commands.runOnce(() -> driverXbox.setRumble(RumbleType.kBothRumble, 1)));
      tooCloseToReef.whileFalse(Commands.runOnce(() -> driverXbox.setRumble(RumbleType.kBothRumble, 0)));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // check in with team about preference^ bumper preference for lock

  }

}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }

  public void periodic() {
    m_vision.updatePoseEstimation(m_drivebase.getSwerveDrive());
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(m_CoralEndEffector);
    SmartDashboard.putData(m_drivebase);
    SmartDashboard.putData(m_AlgaeEndEffector);
  }
}
