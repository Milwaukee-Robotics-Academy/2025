package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeEndEffector extends SubsystemBase {

  // public enum IntakeState {
  //   NONE,
  //   STOW,
  //   INTAKE,
  //   HOLD,
  //   SCORE,
  //   MANUAL
  // }

  // private IntakeState currentState = IntakeState.STOW;
  private SparkMax m_intakeMotor;
  private SparkMax m_armMotor;
  private RelativeEncoder m_intakeEncoder;
  private RelativeEncoder m_armEncoder;
  // private SparkClosedLoopController m_armController;
  // private RelativeEncoder m_intakeEncoder;
  // private SparkAbsoluteEncoder m_armEncoder;
  // private double armTarget = 0;

  public AlgaeEndEffector() {
    m_intakeMotor = new SparkMax(11, MotorType.kBrushless);
    m_armMotor = new SparkMax(12, MotorType.kBrushless);


    m_intakeEncoder = m_intakeMotor.getEncoder();
    m_armEncoder = m_armMotor.getEncoder();

  

    // Setup Configuration of SparkMax Motors
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    globalConfig
        .smartCurrentLimit(40);
    intakeMotorConfig
        .idleMode(IdleMode.kBrake)
        .apply(globalConfig);
    armMotorConfig
        .idleMode(IdleMode.kBrake)
        .apply(globalConfig);
    // Apply motor configuration to SparkMaxes
    m_intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    outputTelemetry();
  }
  public void outputTelemetry() {
    SmartDashboard.putNumber("Arm/Position", m_armEncoder.getPosition());
    SmartDashboard.putNumber("Arm/Current", m_armMotor.getOutputCurrent());
    SmartDashboard.putNumber("Arm/Output", m_armMotor.getAppliedOutput());
    SmartDashboard.putNumber("Arm/Voltage", m_armMotor.getBusVoltage());
    SmartDashboard.putNumber("Intake/Current", m_intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Output", m_intakeMotor.getAppliedOutput());

  }


  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  public void stop() {
    m_armMotor.set(0.0);
    m_intakeMotor.set(0.0);
  }



  public void reset() {
    // Zero arm encoder on initialization
    m_armEncoder.setPosition(0);
  }

  /*---------------------------------- Custom Private Methods ----------------------------------*/

  private void moveArmUp() {
    m_armMotor.set(0.5);
  }

  private void moveArmDown() {

  }

  private void intake(){

  }

  private void stopArm() {

  }

  private void StopIntake() {

  }



  /** ---------------------------------- Public Commands ------------------------------------ */


  public Command stopArmCommand() {
    return new RunCommand(this::stop, this).withName("StopArm");

  }

  public Command moveArmUpCommand() {
    return new RunCommand(this::moveArmUp, this).withName("ArmUp");
  }

}