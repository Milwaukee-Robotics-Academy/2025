package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeEndEffector extends SubsystemBase {

  public enum IntakeState {
    NONE,
    STOW,
    INTAKE,
    HOLD,
    SCORE,
    MANUAL
  }

  private IntakeState currentState = IntakeState.STOW;
  private SparkMax m_intakeMotor;
  private SparkMax m_armMotor;
  private SparkClosedLoopController m_armController;
  private RelativeEncoder m_intakeEncoder;
  private RelativeEncoder m_armEncoder;
  private double armTarget = 0;

  public AlgaeEndEffector() {
    m_intakeMotor = new SparkMax(11, MotorType.kBrushless);
    m_armMotor = new SparkMax(12, MotorType.kBrushless);
    m_armController = m_armMotor.getClosedLoopController();

    m_intakeEncoder = m_intakeMotor.getEncoder();
    m_armEncoder = m_armMotor.getEncoder();

    // Setup Configuration of SparkMax Motors
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    globalConfig
        .smartCurrentLimit(0)
        .idleMode(IdleMode.kBrake);
    intakeMotorConfig
        .apply(globalConfig);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    armMotorConfig
        .apply(globalConfig);
    armMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
    // Apply motor configuration to SparkMaxes
    m_intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Zero arm encoder on initialization
    m_armEncoder.setPosition(0);
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    outputTelemetry();
    SmartDashboard.putString("Algae/State", getState().toString());
  }


  public void stop() {
    m_armEncoder.setPosition(0.0);
    m_intakeMotor.set(0.0);
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber("Arm/Position", m_armEncoder.getPosition());
    SmartDashboard.putNumber("Arm/Target", armTarget);
    SmartDashboard.putNumber("Arm/Current", m_armMotor.getOutputCurrent());
    SmartDashboard.putNumber("Arm/Output", m_armMotor.getAppliedOutput());
    SmartDashboard.putNumber("Arm/Voltage", m_armMotor.getBusVoltage());
    SmartDashboard.putNumber("Intake/Current", m_intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Output", m_intakeMotor.getAppliedOutput());

  }

  public void reset() {
    m_armEncoder.setPosition(0.0);
    m_armController.setReference(0.0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    armTarget=0.0;

  }

  /*---------------------------------- Custom Private Functions ----------------------------------*/

  private void stow() {
    armTarget = Constants.Algae.kStow;
    m_armController.setReference(armTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    m_intakeMotor.set(0);
    currentState = IntakeState.STOW;
  }

  private void grabAlgae() {
    armTarget = Constants.Algae.kIntake;
    m_armController.setReference(armTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    m_intakeMotor.set(Constants.Algae.Intake.kIn);
    currentState = IntakeState.STOW;
  }

  private void score() {
    armTarget = Constants.Algae.kScore;
    m_armController.setReference(armTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    m_intakeMotor.set(Constants.Algae.Intake.kOut);
    currentState = IntakeState.SCORE;
  }

  private void hold() {
    armTarget = Constants.Algae.kHold;
    m_armController.setReference(armTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    m_intakeMotor.set(Constants.Algae.Intake.kHold);
    currentState = IntakeState.HOLD;
  }
  private void idle() {
    m_armController.setReference(armTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    m_intakeMotor.set(0.0);
    currentState = IntakeState.NONE;
  }
  private void groundIntake() {
    this.grabAlgae();
  }

  private void moveToSetpoint(Double setpoint, Double intakeSpeed){
    armTarget = setpoint;
    m_armController.setReference(armTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    m_intakeMotor.set(intakeSpeed);
    currentState = IntakeState.MANUAL;
  }
  private void manual(Double armSpeed, Double intakeSpeed){

    m_armMotor.set(armSpeed);
    m_intakeMotor.set(intakeSpeed);
    currentState = IntakeState.MANUAL;
  }



  /** ---------------------------------- Public Commands ------------------------------------ */
  public Command stowCommand() {
    return new RunCommand(this::stow, this).withName("stow");
  }

  public Command groundIntakeCommand() {
    return new RunCommand(this::groundIntake, this).withName("Intake");
  }

  public Command holdCommand() {
    return new RunCommand(this::hold, this).withName("Hold");
  }

  public Command scoreCommand() {
    return new RunCommand(this::score, this).withName("Score");
  }

  public Command idleCommand() {
    return new RunCommand(this::idle, this).withName("Idle");
  }
  public Command manualControlCommand(Double setpoint, Double intakeSpeed) {
    return new RunCommand(() -> moveToSetpoint(armTarget - setpoint, intakeSpeed), this).withName("Manual");
  }
  

  private IntakeState getState() {
    return currentState;
  }

}