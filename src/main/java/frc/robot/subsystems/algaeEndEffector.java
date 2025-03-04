// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class algaeEndEffector extends SubsystemBase {

  private final SparkMax m_wrist;
  private final SparkMax m_intake;
  private final AbsoluteEncoder m_wristEncoder;
  private final RelativeEncoder m_intakeEncoder;
  private SparkClosedLoopController m_wristClosedLoopController;
  private SparkClosedLoopController m_intakeClosedLoopController;

  /** Creates a new algaeEndEffector. */
  public algaeEndEffector() {

    m_wrist = new SparkMax(Constants.Algae.kWristEncoderId, MotorType.kBrushless);
    m_intake = new SparkMax(Constants.Algae.kIntakeMotorId, MotorType.kBrushless);

    m_intakeEncoder = m_intake.getEncoder();
    m_wristEncoder = m_wrist.getAbsoluteEncoder();

  
    m_wristClosedLoopController = m_wrist.getClosedLoopController();

    SparkMaxConfig wristConfig = new SparkMaxConfig();
    wristConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(Constants.Algae.kMaxWristCurrent)
        .inverted(true);

    m_wrist.configure(
        wristConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);


    // Wrist PID
    // mWristPIDController = new ProfiledPIDController(
    //     Constants.Algae.kWristP,
    //     Constants.Algae.kWristI,
    //     Constants.Algae.kWristD,
    //     new TrapezoidProfile.Constraints(
    //         Constants.Algae.kWristMaxVelocity,
    //         Constants.Algae.kWristMaxAcceleration));

    // Wrist Feedforward
    // mWristFeedForward = new ArmFeedforward(
    //     Constants.Algae.kWristKS,
    //     Constants.Algae.kWristKG,
    //     Constants.Algae.kWristKV,
    //     Constants.Algae.kWristKA);

  }



  public void stowWrist(){
    m_wristClosedLoopController.setReference(Constants.Algae.kStowAngle, ControlType.kPosition);
  }

  public void wristForIntake(){
    m_wristClosedLoopController.setReference(Constants.Algae.kGroundIntakeAngle, ControlType.kPosition);
  }

  public void intake(){
    m_intakeClosedLoopController.setReference(Constants.Algae.kGroundIntakeSpeed, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
