package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeSubsystemConstants;
import frc.robot.Constants.SimulationRobotConstants;


public class AlgaeEndEffector extends SubsystemBase {
  private SparkMax m_intakeMotor;
  private SparkMax m_armMotor;
  private RelativeEncoder m_intakeEncoder;
  private RelativeEncoder m_armEncoder;
  private SparkClosedLoopController armController;


  // Member variables for subsystem state management
  private boolean stowWhenIdle = true;
  private boolean wasReset = false;


  private DCMotor armMotorModel = DCMotor.getNEO(1);
  private SparkMaxSim armMotorSim;
  private final SingleJointedArmSim m_intakeSim =
      new SingleJointedArmSim(
          armMotorModel,
          SimulationRobotConstants.kIntakeReduction,
          SingleJointedArmSim.estimateMOI(
              SimulationRobotConstants.kIntakeLength, SimulationRobotConstants.kIntakeMass),
          SimulationRobotConstants.kIntakeLength,
          SimulationRobotConstants.kIntakeMinAngleRads,
          SimulationRobotConstants.kIntakeMaxAngleRads,
          true,
          SimulationRobotConstants.kIntakeMinAngleRads,
          0.0,
          0.0);

  // Mechanism2d setup for subsytem
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Ball Intake Root", 25, 25);
  private final MechanismLigament2d m_armTower =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Arm Tower",
              SimulationRobotConstants.kIntakeShortBarLength
                  * SimulationRobotConstants.kPixelsPerMeter,
              Units.radiansToDegrees(SimulationRobotConstants.kIntakeMinAngleRads)));

  @SuppressWarnings("unused")
  private final MechanismLigament2d m_arm =
      m_armTower.append(
          new MechanismLigament2d(
              "Arm",
              SimulationRobotConstants.kIntakeLongBarLength
                  * SimulationRobotConstants.kPixelsPerMeter,
              Units.radiansToDegrees(SimulationRobotConstants.kIntakeBarAngleRads)));


  public AlgaeEndEffector(){
    m_intakeMotor =  new SparkMax(11, MotorType.kBrushless);
    m_armMotor =  new SparkMax(12, MotorType.kBrushless);
    armController = m_armMotor.getClosedLoopController();

    m_intakeEncoder = m_intakeMotor.getEncoder();
    m_armEncoder = m_armMotor.getEncoder();



    //Setup Configuration of SparkMax Motors
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    globalConfig
      .smartCurrentLimit(0)
      .idleMode(IdleMode.kBrake);
    intakeMotorConfig
      .apply(globalConfig);    
    armMotorConfig
      .apply(globalConfig);
    //Apply motor configuration to SparkMaxes
    m_intakeMotor.configure(intakeMotorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    m_armMotor.configure(armMotorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
  
  
  // Display mechanism2d
  SmartDashboard.putData("Algae Subsystem", m_mech2d);

  // Zero arm encoder on initialization
  m_armEncoder.setPosition(0);

  // Initialize Simulation values
  armMotorSim = new SparkMaxSim(m_armMotor, armMotorModel);

  }
  /** Zero the arm encoder when the user button is pressed on the roboRIO */
  private void zeroOnUserButton() {
    if (!wasReset && RobotController.getUserButton()) {
      // Zero the encoder only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasReset = true;
      m_armEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasReset = false;
    }
  }

  /**
   * Command to run the algae intake. This will extend the arm to its "down" position and run the
   * motor at its "forward" power to intake the ball.
   *
   * <p>This will also update the idle state to hold onto the ball when this command is not running.
   */
  public Command runIntakeCommand() {
    return this.run(
        () -> {
          stowWhenIdle = false;
          setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints.kForward);
          setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints.kDown);
        }).withName("Run Intake");
  }

  /**
   * Command to run the algae intake in reverse. This will extend the arm to its "hold" position and
   * run the motor at its "reverse" power to eject the ball.
   *
   * <p>This will also update the idle state to stow the arm when this command is not running.
   */
  public Command reverseIntakeCommand() {
    return this.run(
        () -> {
          stowWhenIdle = true;
          setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints.kReverse);
          setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints.kHold);
        }).withName("Reverse Intake");
  }

  /** Command to force the subsystem into its "stow" state. */
  public Command stowCommand() {
    return this.runOnce(
        () -> {
          stowWhenIdle = true;
        }).withName("stow Algae");
  }

  /**
   * Command to run when the intake is not actively running. When in the "hold" state, the intake
   * will stay in the "hold" position and run the motor at its "hold" power to hold onto the ball.
   * When in the "stow" state, the intake will stow the arm in the "stow" position and stop the
   * motor.
   */
  public Command idleCommand() {
    return this.run(
        () -> {
          if (stowWhenIdle) {
            setIntakePower(0.0);
            setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints.kStow);
          } else {
            setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints.kHold);
            setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints.kHold);
          }
        }).withName("Idle Algae");
  }

  /** Set the intake motor power in the range of [-1, 1]. */
  private void setIntakePower(double power) {
    m_intakeMotor.set(power);
  }

  /** Set the arm motor position. This will use closed loop position control. */
  private void setIntakePosition(double position) {
    armController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    zeroOnUserButton();

    // Display subsystem values
    SmartDashboard.putNumber("Algae/Arm/Position", m_armEncoder.getPosition());
    SmartDashboard.putNumber("Algae/Intake/Applied Output", m_intakeMotor.getAppliedOutput());

    // Update mechanism2d
    m_armTower.setAngle(
        Units.radiansToDegrees(SimulationRobotConstants.kIntakeMinAngleRads)
            + Units.rotationsToDegrees(
                m_armEncoder.getPosition() / SimulationRobotConstants.kIntakeReduction));
  }

  /** Get the current drawn by each simulation physics model */
  public double getSimulationCurrentDraw() {
    return m_intakeSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_intakeSim.setInput(armMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_intakeSim.update(0.020);

    // Iterate the arm SPARK simulation
    armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            m_intakeSim.getVelocityRadPerSec() * SimulationRobotConstants.kArmReduction),
        RobotController.getBatteryVoltage(),
        0.02);

    // SimBattery is updated in Robot.java
  }
}
 