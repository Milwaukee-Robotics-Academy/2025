package frc.robot.subsystems;

import java.security.PrivateKey;

import com.revrobotics.RelativeEncoder;
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


public class AlgaeEndEffector extends SubsystemBase {
  private SparkMax m_motor_11;
  private SparkMax m_motor_12;
  private RelativeEncoder m_encoder_11;
  private RelativeEncoder m_encoder_12;
  
  public enum intakeStates
  {
    UNLOCKED,
    LOCKED
  }

  private intakeStates intakeState;
  private double encoderLockValue;

  public AlgaeEndEffector(){
    m_motor_11 =  new SparkMax(11, MotorType.kBrushless);
    m_motor_12 =  new SparkMax(12, MotorType.kBrushless);
    m_encoder_11 = m_motor_11.getEncoder();
    m_encoder_12 = m_motor_12.getEncoder();
    intakeState = intakeStates.UNLOCKED;
    encoderLockValue = 0;

    //Setup Configuration of SparkMax Motors
    SparkMaxConfig global_config = new SparkMaxConfig();
    SparkMaxConfig motor_12_config = new SparkMaxConfig();
    SparkMaxConfig motor_11_config = new SparkMaxConfig();
    global_config
      .smartCurrentLimit(0)
      .idleMode(IdleMode.kBrake);
    motor_11_config
      .apply(global_config);    
    motor_12_config
      .apply(global_config);
    //Apply motor configuration to SparkMaxes
    m_motor_11.configure(motor_11_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    m_motor_12.configure(motor_12_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
  }

  
  private void LockAlgae(){
    if (intakeState == intakeStates.LOCKED)
    {
      double currentEncoderValue = m_encoder_11.getPosition();
      if(currentEncoderValue > encoderLockValue*1.02)
      {
        m_motor_11.set(-.2);
      }
      else if (currentEncoderValue < encoderLockValue*1.02) 
      {
        m_motor_11.set(.2);
      }
    }
    else
    {
      encoderLockValue = m_encoder_11.getPosition();
      intakeState = intakeStates.LOCKED;
    }

  }

  private void unlockAlgae()
  {
    intakeState = intakeStates.UNLOCKED;
    encoderLockValue = 0;
  }

  private void intake(){
    unlockAlgae();
    m_motor_11.set(0.5);
  }

  private void stopIntake(){
    m_motor_11.set(0);
  }

  private void outtake(){
    unlockAlgae();
    m_motor_11.set(-0.5);
  }

  private void raiseArm(){
    m_motor_12.set(0.25);
  }

  private void stopArm(){
    m_motor_12.set(0);
  }

  private void lowerArm(){
    m_motor_12.set(-0.25);
  }

  public Command lockAlgaeCommand(){
    return new RunCommand(this::LockAlgae, this).withName("LockAlgae");
  }

  public Command intakeCommand(){
    return new RunCommand(this::intake, this).withName("Intake");
  }
  public Command outtakeCommand(){
    return new RunCommand(this::outtake, this).withName("Outtake");
  }

  public Command stopCommand(){
    return new InstantCommand(this::stopIntake, this).withName("StopIntake");
  }
  public Command goUpFunctionCommand(){
    return new RunCommand(this::raiseArm, this).withName("RaiseArm");
  }
  public Command goDownFunctionCommand(){
    return new RunCommand(this::lowerArm, this).withName("LowerArm");
  }
    
  public Command stop2Command(){
    return new InstantCommand(this::stopArm, this).withName("StopArm");
  }

  @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("Intake Algae", m_encoder_11.getPosition());
      SmartDashboard.putNumber("Outtake Algae", m_encoder_12.getPosition());
    
    }
}
 