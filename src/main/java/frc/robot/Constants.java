// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(15.7);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class Algae {
    public static final int kIntakeMotorCanId = 13;
    public static final int kPivotMotorCanId = 14;
    public static final double kStow = 0;
    public static final double kIntake = -40;
    public static final double kHold = -40;
    public static final double kScore = -38;
    

    public static final class Intake {
      public static final double kStop = 0;
      public static final double kIn = 0.5;
      public static final double kOut = -0.5;
      public static final double kHold = 0.25;
    }
  }
  

  // public static final class SimulationRobotConstants {
  //   public static final double kPixelsPerMeter = 20;

  //   public static final double kElevatorGearing = 25; // 25:1
  //   public static final double kCarriageMass =
  //       4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
  //   public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
  //   public static final double kMinElevatorHeightMeters = 0.922; // m
  //   public static final double kMaxElevatorHeightMeters = 1.62; // m

  //   public static final double kArmReduction = 60; // 60:1
  //   public static final double kArmLength = 0.433; // m
  //   public static final double kArmMass = 4.3; // Kg
  //   public static final double kMinAngleRads =
  //       Units.degreesToRadians(-50.1); // -50.1 deg from horiz
  //   public static final double kMaxAngleRads =
  //       Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

  //   public static final double kIntakeReduction = 135; // 135:1
  //   public static final double kIntakeLength = 0.4032262; // m
  //   public static final double kIntakeMass = 5.8738; // Kg
  //   public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
  //   public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
  //   public static final double kIntakeShortBarLength = 0.1524;
  //   public static final double kIntakeLongBarLength = 0.3048;
  //   public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
  // }


  public static class Vision {

    public static final String kCameraNameFrontLeft = "Arducam_OV9782_USB_Camera";
    public static final String kCameraNameRearRight = "Arducam_OV9782_USB_Camera (1)";
    // Cam mounted facing forward, 11 inches forward of center, 9 inches left of center, 8 inches up
    // from center.
    public static final Transform3d kRobotToCamFrontLeft = new Transform3d(
      new Translation3d(
          Units.inchesToMeters(11.75), 
          Units.inchesToMeters(9), 
          Units.inchesToMeters(8)),
      new Rotation3d(0, 0, Units.degreesToRadians(-15)));
    public static final Transform3d kRobotToCamRearRight = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(-11.75), 
        Units.inchesToMeters(-9), 
        Units.inchesToMeters(8)),
        new Rotation3d(0, 0, Units.degreesToRadians(172)));
    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(8, 8, 12);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(5, 5, 10);
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
    public static final double kTriggerButtonThreshold = 0.2;
  }
}
