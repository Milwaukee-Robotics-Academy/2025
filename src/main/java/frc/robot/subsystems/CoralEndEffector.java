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
  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;
 
private TimeOfFlight m_intakeSensor = new TimeOfFlight(0);
private TimeOfFlight m_acquiredSensor = new TimeOfFlight(1);

  /** Creates a new EndEffector. */
  public CoralEndEffector() {
    m_leftMotor =  new SparkMax(9, MotorType.kBrushless);
    m_rightMotor =  new SparkMax(10, MotorType.kBrushless);
    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

    rightMotorConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake)  
      .inverted(true);
    leftMotorConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    m_leftMotor.configure(leftMotorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    m_rightMotor.configure(rightMotorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    
  }
private void intake(){
  m_leftMotor.set(0.5);
  m_rightMotor.set(0.5);
}
private void stop(){
  m_leftMotor.set(0);
  m_rightMotor.set(0);
}
private void outtake(){
  m_leftMotor.set(0.5);
  m_rightMotor.set(0.2);
}

private void spitback(){
  m_leftMotor.set(-0.5);
  m_rightMotor.set(-0.5);
}
private void nudgeForward(){
  m_leftMotor.set(0.1);
  m_rightMotor.set(0.1);
}
public Command intakeCommand(){
  return new RunCommand(this::intake, this).withName("Intake");
}
public Command spitbackCommand(){
  return new RunCommand(this::spitback, this).withName("Spitback");
}
public Command outtakeCommand(){
  return new RunCommand(this::outtake, this).withName("Outtake");
}

public Command outtakeAndStopCommand(){
  return outtakeCommand()
  .until(() -> !this.acquired())
  .andThen(stopCommand())
  .withName("OuttakeAndStop");
}

public Command nudgeForwardCommand(){
  return new RunCommand(this::nudgeForward, this).withName("nudge");
}

public Command stopCommand(){
 return new InstantCommand(this::stop, this).withName("Stopped");
}

public Command intakeWithSensorsCommand(){
  return this.intakeCommand()
  .until(()-> this.atInSensor())
  .andThen(this.nudgeForwardCommand())
  .until(() -> this.acquired())
  .andThen(this.stopCommand()).withName("IntakeWithSensors");
}

public Trigger coralLoadedTrigger(){
  return new Trigger(() -> (acquired()));
}

private boolean atInSensor(){
  return m_intakeSensor.getRange() <90;
}

private boolean atOutSensor(){
  return m_acquiredSensor.getRange() <90;
}


private boolean acquired(){
  if (!atInSensor() && atOutSensor()){
    return true;
  }
  return false;
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake sensor", m_intakeSensor.getRange());
    SmartDashboard.putNumber("Acquired sensor", m_acquiredSensor.getRange());
    SmartDashboard.putBoolean("At Intake", atInSensor());
    SmartDashboard.putBoolean("At Outtake", atOutSensor());
    SmartDashboard.putBoolean("Acquired", acquired());
  }
}
