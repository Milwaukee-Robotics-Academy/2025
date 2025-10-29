// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.playingwithfusion.TimeOfFlight;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class CoralEndEffector extends SubsystemBase {
  private SparkMax m_motor_9 = new SparkMax(9, MotorType.kBrushless);
  private SparkMax m_motor_10 = new SparkMax(10, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorLeftConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorControllerConfig motorRightConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(true)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motorLeft = new SparkWrapper(m_motor_9, DCMotor.getNEO(1), motorLeftConfig);
  private final SmartMotorController motorRight = new SparkWrapper(m_motor_10, DCMotor.getNEO(1), motorRightConfig);

  private TimeOfFlight intakeSensor = new TimeOfFlight(0);
  private TimeOfFlight acquiredSensor = new TimeOfFlight(1);

  /** Creates a new EndEffector. */
  public CoralEndEffector() {

  }

  private void intake() {
    m_motor_9.set(0.5);
    m_motor_10.set(0.5);
  }

  private void stop() {
    m_motor_9.set(0);
    m_motor_10.set(0);
  }

  private void outtake() {
    m_motor_9.set(0.5);
    m_motor_10.set(0.2);
  }

  private void spitback() {
    m_motor_9.set(-0.5);
    m_motor_10.set(-0.5);
  }

  private void nudgeForward() {
    m_motor_9.set(0.1);
    m_motor_10.set(0.1);
  }

  public Command intakeCommand() {
    // return new RunCommand(this::intake, this).withName("Intake");

    return startRun(() -> {
      motorLeft.stopClosedLoopController();
      motorRight.stopClosedLoopController();
    }, // Stop the closed loop controller since the motor is in ControlMode.CLOSED_LOOP
        () -> {
          motorLeft.setDutyCycle(0.5);
          motorRight.setDutyCycle(0.5);
        }) // Apply the dutycycle given
        .finallyDo(() -> {
          motorLeft.startClosedLoopController();
          motorRight.startClosedLoopController();
        }) // Start the closed loop controller when this command is interrupted
        .withName("Custom Intake"); // Be nice, give your command name :)

  }

  public Command spitbackCommand() {
    return startRun(() -> {
      motorLeft.stopClosedLoopController();
      motorRight.stopClosedLoopController();
    }, // Stop the closed loop controller since the motor is in ControlMode.CLOSED_LOOP
        () -> {
          motorLeft.setDutyCycle(-0.5);
          motorRight.setDutyCycle(-0.5);
        }) // Apply the dutycycle given
        .finallyDo(() -> {
          motorLeft.startClosedLoopController();
          motorRight.startClosedLoopController();
        }) // Start the closed loop controller when this command is interrupted
        .withName("SpitBack"); // Be nice, give your command name :)
  }

  public Command outtakeCommand() {
    return startRun(() -> {
      motorLeft.stopClosedLoopController();
      motorRight.stopClosedLoopController();
    }, // Stop the closed loop controller since the motor is in ControlMode.CLOSED_LOOP
        () -> {
          motorLeft.setDutyCycle(0.5);
          motorRight.setDutyCycle(0.5);
        }) // Apply the dutycycle given
        .finallyDo(() -> {
          motorLeft.startClosedLoopController();
          motorRight.startClosedLoopController();
        }) // Start the closed loop controller when this command is interrupted
        .withName("Outtake");
  }

  public Command outtakeAndStopCommand() {
    return outtakeCommand()
        .until(() -> !this.acquired())
        .andThen(stopCommand())
        .withName("OuttakeAndStop");
  }

  public Command nudgeForwardCommand() {
    return startRun(() -> {
      motorLeft.stopClosedLoopController();
      motorRight.stopClosedLoopController();
    }, // Stop the closed loop controller since the motor is in ControlMode.CLOSED_LOOP
        () -> {
          motorLeft.setDutyCycle(0.1);
          motorRight.setDutyCycle(0.1);
        }) // Apply the dutycycle given
        .finallyDo(() -> {
          motorLeft.startClosedLoopController();
          motorRight.startClosedLoopController();
        }) // Start the closed loop controller when this command is interrupted
        .withName("Custom Intake"); // Be nice, give your command name :).withName("nudge");
  }

  public Command stopCommand() {
    return run(() -> {
      motorLeft.stopClosedLoopController();
      motorRight.stopClosedLoopController();
    })
        .withName("Stopped");
  }

  public Command intakeWithSensorsCommand() {
    return this.intakeCommand()
        .until(() -> this.atInSensor())
        .andThen(this.nudgeForwardCommand())
        .until(() -> this.acquired())
        .andThen(this.stopCommand()).withName("IntakeWithSensors");
  }

  public Trigger coralLoadedTrigger() {
    return new Trigger(() -> (acquired()));
  }

  private boolean atInSensor() {
    return intakeSensor.getRange() < 90;
  }

  private boolean atOutSensor() {
    return acquiredSensor.getRange() < 90;
  }

  private boolean acquired() {
    if (!atInSensor() && atOutSensor()) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motorLeft.updateTelemetry();
    motorRight.updateTelemetry();

    SmartDashboard.putNumber("Intake sensor", intakeSensor.getRange());
    SmartDashboard.putNumber("Acquired sensor", acquiredSensor.getRange());
    SmartDashboard.putBoolean("At Intake", atInSensor());
    SmartDashboard.putBoolean("At Outtake", atOutSensor());
    SmartDashboard.putBoolean("Acquired", acquired());
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    motorLeft.simIterate();
    motorRight.simIterate();
  }
}
