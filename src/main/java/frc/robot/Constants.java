// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class robotConstants {
    public static final String canbusName = "LETHIMCOOK";
    public static final int DriverControllerID = 0;
    public static String mode = "DISABLED";
  }

  // public static enum UpperState {
  //   DEFAULT,
  //   GROUND,
  //   AMP,
  //   BASE,
  //   FAR,
  //   LFIGHT,
  //   MGROUND,
  //   SHOOT,
  //   TRAP,
  //   NULL,
  //   PREENDGAME,
  //   ENDGAME
  // }

  // public static UpperState state;

  public static final class SwerveConstants {
    public static final double axisDeadBand = 0.05; // make sure ur robot won't vibrate cuz the joystick gives a input like 0.002 or sth
    public static final int pigeon1 = 0; // advanced gyro
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = 0.583; // meters, length between two side's wheels, need to adjust
    public static final double wheelBase = 0.583; // meters, length between same side's wheels, need to adjust
    public static final double driveBaseRaius = 0.35234;
    public static final double wheelDiameter = Units.inchesToMeters(4.0); // need to adjust
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;  // open loop means no feedback(PID), closed loop vise versa, not used actually

    public static final double driveGearRatio = (6.12244897959 / 1.0); // 6.12:1 (6.12244897959), for MK4i(L3)
    public static final double angleGearRatio = (150.0 / 7.0 / 1.0); // 150/7 : 1, for MK4i(all)

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0; // setting the nominal voltage(won't really follow anyway)

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 30; //20
    public static final int driveContinuousCurrentLimit = 40; //80, limiting the amps so Neo won't brake

    /* Angle Motor PID Values */
    public static final double angleKP = 0.010625;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.005; // maybe need to adjust

    /* Angle Motor Auto-Facing PID Values */
    public static final double faceKP = 0.8;
    public static final double faceKI = 0.0;
    public static final double faceKD = 0.1;
    public static final double faceiWindup = 0.0;
    public static final double faceiLimit = 0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0025;
    public static final double driveKFF = 0.0; // maybe need to adjust

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27; // feedforward, maybe need to adjust

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio; // like constants in physics

    /* Swerve Profiling Values */
    public static final double maxModuleSpeed = 4.5; // M/S
    public static final double maxAngularVelocity = 13.5; // meters per second

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake; // whether u want to let neo stop slowly individually(coast) or fiercely wholely(brake)

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false; // yeah invert the motor

    /* Angle Encoder Invert */
    public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive; // invert cancoder(in CTREconfig)

    /* Field Oriented */
    public static boolean fieldOriented = false;

    
    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 01;
      public static final int angleMotorID = 02;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.291016);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.167236);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Rear Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.690918);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    /* Rear Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 31;
      public static final int angleMotorID = 32;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.113281);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    public static final Translation2d LFModuleOffset = new Translation2d(0.4, 0.4);
    public static final Translation2d RFModuleOffset = new Translation2d(0.4, -0.4);
    public static final Translation2d LRModuleOffset = new Translation2d(-0.4, 0.4);
    public static final Translation2d RRModuleOffset = new Translation2d(-0.4, -0.4);

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(5.0, 0, 0), // Translation constants 
      new PIDConstants(5.0, 0, 0), // Rotation constants 
      maxModuleSpeed, 
      LFModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );

  }

  public static final class VisionConstants {
    public static final Translation3d zeroTranslation3d = new Translation3d(0, 0, 0);
    public static final Transform3d zeroTransform3d = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    public static final double[][] data = {
      {0.6775, -0.204589},
      {1.6775, -0.002197},
      {2.1775, -0.002197},
      {2.6775, -0.002197},
      {3.1775, -0.002197},
      {3.6775, -0.002197},
      {4.1775, -0.002197}
    };

    public static final Translation2d Speaker_red = new Translation2d(8.036, 1.442);
    public static final Translation2d Speaker_blue = new Translation2d(-8.036, 1.442);
    // 0.6775
  }

  public static final class UpperConstants {
    public static final int rightLimitSwitchID = 8;
    public static final int LeftLimitSwitchID = 7;

    public static final int leftElbowMotorID = 17;
    public static final int rightElbowMotorID = 18;
    public static final int leftShooterMotorID = 21;
    public static final int rightShooterMotorID = 20;
    public static final int elbowCancoderID = 19;
    public static final int intakeMotorID = 22;

    public static final double elbowCancoderOffset = -0.00709;
    public static final double shooter_arm_Angle = 135;

    public static final double elbowKP = 10;
    public static final double elbowKI = 0.0;
    public static final double elbowKD = 5;
    public static final double elbowiWindup = 0.0;
    public static final double elbowiLimit = 0.0;

    public static final double shooterKP = 0.0;
    public static final double shooterKI = 0.0; // test
    public static final double shooterKD = 0.0;
    public static final double shooteriWindup = 0.0; // test
    public static final double shooteriLimit = 0.0; // test

    // 3/15 -0.01279(upper movement)
    public static final double ELBOW_DEFAULT_POS = -0.103102; // -0.014987
    public static final double ELBOW_GROUND_POS = -0.2425;
    public static final double ELBOW_LFIGHT_POS = -0.235;
    public static final double ELBOW_AMP_POS = 0.024319; // 1.0.012939 2.-0.002197
    public static final double ELBOW_BASE_POS = -0.21279;
    public static final double ELBOW_FAR_POS = -0.19779;
    public static final double ELBOW_TRAP_POS = -0.2;
    public static final double ELBOW_PREENDGAME_POS = 0.06221;

    public static final double INTAKE_HOLD_SPEED = 0;
    public static final double INTAKE_GROUND_SPEED = 0.35;
    public static final double INTAKE_SHOOT_SPEED = 1;

    public static final double SHOOTER_GROUND_SPEED = 0.03;
    public static final double SHOOTER_TRAP_SPEED = -0.35;
    public static final double SHOOTER_SHOOT_SPEED = -1;
    public static final double SHOOTER_HOLD_SPEED = 0;
    public static final double SHOOTER_LEGAL_SPEED = 5000;

    public static boolean teleMode = false;

    public static final int ledLength = 33; // need to change
    public static final int ledPwmPort = 7;
  }

  public static final class FieldConstants {
    // auto position values
    // global
    public static final double shootingTime = 0.5;
    public static final double intakeTime = 3;
    public static final double holdTime = 2;

    // left speaker
    public static final double leftSpeakerOffset = 90;

    // mid speaker
    public static final double midSpeakerOffset = 0;

    // right speakerd
    public static final double rightSpeakerOffset = 90;
  }
}