// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CoralEndEffector extends SubsystemBase {
  private SparkMax m_motor_9;
  private SparkMax m_motor_10;
 
private TimeOfFlight intakeSensor = new TimeOfFlight(0);
private TimeOfFlight acquiredSensor = new TimeOfFlight(1);

  /** Creates a new EndEffector. */
  public CoralEndEffector() {
    m_motor_9 =  new SparkMax(9, MotorType.kBrushless);
    m_motor_10 =  new SparkMax(10, MotorType.kBrushless);
    SparkMaxConfig global_config = new SparkMaxConfig();
    SparkMaxConfig motor_10_config = new SparkMaxConfig();
    SparkMaxConfig motor_9_config = new SparkMaxConfig();
    global_config
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);
    motor_9_config
      .apply(global_config)      
      .inverted(true);
    motor_10_config
      .apply(global_config);
    m_motor_9.configure(motor_9_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    m_motor_10.configure(motor_10_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    SmartDashboard.putNumber("Intake sensor", intakeSensor.getRange());
    SmartDashboard.putNumber("Acquired sensor", acquiredSensor.getRange());
    SmartDashboard.putBoolean("Intake", coralComingIn());
    SmartDashboard.putBoolean("Acquired", coralAcquired());
  }
private void intake(){
  m_motor_9.set(0.5);
  m_motor_10.set(0.5);
}
private void stop(){
  m_motor_9.set(0);
  m_motor_10.set(0);
}
private void outtake(){
  m_motor_9.set(0.5);
  m_motor_10.set(0.2);
}

private void nudgeForward(){
  m_motor_9.set(0.1);
  m_motor_10.set(0.1);
}
public Command intakeCommand(){
  return new RunCommand(this::intake).withName("Intake");
}
public Command outtakeCommand(){
  return new RunCommand(this::outtake).withName("Outtake");
}

public Command nudgeForwardCommand(){
  return new RunCommand(this::nudgeForward).withName("nudge");
}

public Command stopCommand(){
 return new InstantCommand(this::stop).withName("Stopped");
}

public Command intakeWithSensorsCommand(){
  return this.intakeCommand()
  .until(()-> this.atInSensor())
  .andThen(this.nudgeForwardCommand())
  .until(() -> this.atOutSensor())
  .andThen(this.stopCommand());
}

private boolean atInSensor(){
  return intakeSensor.getRange() <50;
}

private boolean atOutSensor(){
  return (acquiredSensor.getRange() <50); 
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake sensor", intakeSensor.getRange());
    SmartDashboard.putNumber("Acquired sensor", acquiredSensor.getRange());
    SmartDashboard.putBoolean("Intake", coralComingIn());
    SmartDashboard.putBoolean("Acquired", coralAcquired());
  }
}
