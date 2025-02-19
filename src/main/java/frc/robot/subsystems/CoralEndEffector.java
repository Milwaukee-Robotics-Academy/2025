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
      .apply(global_config);
    motor_10_config
      .apply(global_config)
      .inverted(true);
    m_motor_9.configure(motor_9_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    m_motor_10.configure(motor_10_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    SmartDashboard.putNumber("Intake sensor", intakeSensor.getRange());
    SmartDashboard.putNumber("Acquired sensor", acquiredSensor.getRange());
    
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
public Command intakeCommand(){
  return new InstantCommand(this::intake).withName("Intake");
}
public Command outtakeCommand(){
  return new InstantCommand(this::outtake).withName("Outtake");
}
public Command stopCommand(){
 return new InstantCommand(this::stop).withName("Stopped");
}

public Command intakeWithSensorsCommand(){
  return this.intakeCommand().until(()-> this.coralComingIn()).andThen(this.intakeCommand().withTimeout(0.3)).andThen(this.stopCommand());
}

//verify if 5 is too much or not enough
private boolean coralComingIn(){
  return intakeSensor.getRange() <50;
}
private boolean coralAcquired(){
  return acquiredSensor.getRange() <50;
}
public Trigger coralLoadedTrigger(){
  return new Trigger(() -> (coralComingIn()));
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake sensor", intakeSensor.getRange());
    SmartDashboard.putNumber("Acquired sensor", acquiredSensor.getRange());
  }
}
