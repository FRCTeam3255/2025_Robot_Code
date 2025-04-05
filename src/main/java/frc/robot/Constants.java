// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.frcteam3255.components.swerve.SN_SwerveConstants;
import com.frcteam3255.components.swerve.SN_SwerveModule;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotMap.mapDrivetrain;

public final class Constants {
  /**
   * Volts
   */
  public static final double MAX_VOLTAGE = 12;

  public static final Transform3d ROBOT_TO_BUMPERS = new Transform3d(0, 0, Units.Meters.convertFrom(5, Units.Inches),
      Rotation3d.kZero);

  public static class constControllers {
    public static final double DRIVER_LEFT_STICK_DEADBAND = 0.05;
    public static final boolean SILENCE_JOYSTICK_WARNINGS = true;

    public static final double HAS_CORAL_RUMBLE_INTENSITY = 1;
    public static final double HAS_ALGAE_RUMBLE_INTENSITY = 1;
    public static final double READY_TO_PLACE_RUMBLE_INTENSITY = 0.5;
    public static final double READY_TO_RAISE_INTENSITY = 0.6;
    public static final double READY_TO_LEAVE_INTENSITY = 1;

  }

  public static class constDrivetrain {
    public static class PRACTICE_BOT {
      public static final SN_SwerveConstants SWERVE_CONSTANTS = new SN_SwerveConstants(
          SN_SwerveConstants.MK4I.FALCON.L2.steerGearRatio,
          0.09779 * Math.PI,
          SN_SwerveConstants.MK4I.FALCON.L2.driveGearRatio,
          SN_SwerveConstants.MK4I.FALCON.L2.maxSpeedMeters);

      public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = -0.109375;
      public static final double BACK_LEFT_ABS_ENCODER_OFFSET = -0.066406;
      public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = -0.049316;
      public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = 0.314209;

      public static SN_SwerveModule[] MODULES = new SN_SwerveModule[] {
          new SN_SwerveModule(0, mapDrivetrain.FRONT_LEFT_DRIVE_CAN, mapDrivetrain.FRONT_LEFT_STEER_CAN,
              mapDrivetrain.FRONT_LEFT_ABSOLUTE_ENCODER_CAN,
              constDrivetrain.PRACTICE_BOT.FRONT_LEFT_ABS_ENCODER_OFFSET),
          new SN_SwerveModule(1, mapDrivetrain.FRONT_RIGHT_DRIVE_CAN, mapDrivetrain.FRONT_RIGHT_STEER_CAN,
              mapDrivetrain.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN,
              constDrivetrain.PRACTICE_BOT.FRONT_RIGHT_ABS_ENCODER_OFFSET),
          new SN_SwerveModule(2, mapDrivetrain.BACK_LEFT_DRIVE_CAN, mapDrivetrain.BACK_LEFT_STEER_CAN,
              mapDrivetrain.BACK_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.PRACTICE_BOT.BACK_LEFT_ABS_ENCODER_OFFSET),
          new SN_SwerveModule(3, mapDrivetrain.BACK_RIGHT_DRIVE_CAN, mapDrivetrain.BACK_RIGHT_STEER_CAN,
              mapDrivetrain.BACK_RIGHT_ABSOLUTE_ENCODER_CAN,
              constDrivetrain.PRACTICE_BOT.BACK_RIGHT_ABS_ENCODER_OFFSET),
      };

    }

    public static final double WHEEL_DIAMETER = Units.Inches.of(3.87).in(Units.Meters);
    public static final Distance WHEEL_RADIUS = Units.Meters.of(WHEEL_DIAMETER / 2);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    /**
     * <p>
     * Observed maximum translational speed while manually driving on the
     * Competition Robot.
     * </p>
     */
    public static final LinearVelocity OBSERVED_DRIVE_SPEED = Units.MetersPerSecond.of(4.5);

    // TODO: Convert all applicable fields to MEASUREs (Standard_Swerve_Code)
    public static final SN_SwerveConstants SWERVE_CONSTANTS = new SN_SwerveConstants(
        SN_SwerveConstants.MK4I.FALCON.L2.steerGearRatio,
        WHEEL_CIRCUMFERENCE,
        SN_SwerveConstants.MK4I.FALCON.L2.driveGearRatio,
        OBSERVED_DRIVE_SPEED.in(MetersPerSecond));

    /**
     * <p>
     * Theoretical maximum translational speed while manually driving on the
     * Competition Robot.
     * </p>
     * <b>Units:</b> Meters Per Second
     */
    public static final double THEORETICAL_MAX_DRIVE_SPEED = SWERVE_CONSTANTS.maxSpeedMeters;

    // In Rotations: Obtain by aligning all of the wheels in the correct direction
    // and
    // copy-pasting the Raw Absolute Encoder value
    public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = -0.809082;
    public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = 0.033936;
    public static final double BACK_LEFT_ABS_ENCODER_OFFSET = 0.894775;
    public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = 0.343750;

    public static SN_SwerveModule[] MODULES = new SN_SwerveModule[] {
        new SN_SwerveModule(0, mapDrivetrain.FRONT_LEFT_DRIVE_CAN, mapDrivetrain.FRONT_LEFT_STEER_CAN,
            mapDrivetrain.FRONT_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.FRONT_LEFT_ABS_ENCODER_OFFSET),
        new SN_SwerveModule(1, mapDrivetrain.FRONT_RIGHT_DRIVE_CAN, mapDrivetrain.FRONT_RIGHT_STEER_CAN,
            mapDrivetrain.FRONT_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.FRONT_RIGHT_ABS_ENCODER_OFFSET),
        new SN_SwerveModule(2, mapDrivetrain.BACK_LEFT_DRIVE_CAN, mapDrivetrain.BACK_LEFT_STEER_CAN,
            mapDrivetrain.BACK_LEFT_ABSOLUTE_ENCODER_CAN, constDrivetrain.BACK_LEFT_ABS_ENCODER_OFFSET),
        new SN_SwerveModule(3, mapDrivetrain.BACK_RIGHT_DRIVE_CAN, mapDrivetrain.BACK_RIGHT_STEER_CAN,
            mapDrivetrain.BACK_RIGHT_ABSOLUTE_ENCODER_CAN, constDrivetrain.BACK_RIGHT_ABS_ENCODER_OFFSET),
    };

    // Physically measured from center to center of the wheels
    // Distance between Left & Right Wheels
    public static final double TRACK_WIDTH = Units.Meters.convertFrom(23.75, Units.Inches);
    // Distance between Front & Back Wheels
    public static final double WHEELBASE = Units.Meters.convertFrom(23.75, Units.Inches);

    // -- CONFIGS --
    // This PID is implemented on each module, not the Drivetrain subsystem.
    public static final double DRIVE_P = 0.18;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0;

    public static final double STEER_P = 100;
    public static final double STEER_I = 0.0;
    public static final double STEER_D = 0.14414076246334312;

    public static final double DRIVE_KS = 0;
    public static final double DRIVE_KA = 0;
    public static final double DRIVE_KV = (1 / OBSERVED_DRIVE_SPEED.in(Units.MetersPerSecond));

    public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue STEER_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;
    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final NeutralModeValue STEER_NEUTRAL_MODE = NeutralModeValue.Coast;
    public static final Current DRIVE_CURRENT_LIMIT = Units.Amps.of(99999);

    public static TalonFXConfiguration DRIVE_CONFIG = new TalonFXConfiguration();
    public static TalonFXConfiguration STEER_CONFIG = new TalonFXConfiguration();
    public static CANcoderConfiguration CANCODER_CONFIG = new CANcoderConfiguration();

    static {
      // TODO: Im sure a lot of this can be cleaned up, but some of them are passed
      // into the super class
      // Into the constructor of drive
      DRIVE_CONFIG.Slot0.kP = DRIVE_P;
      DRIVE_CONFIG.Slot0.kI = DRIVE_I;
      DRIVE_CONFIG.Slot0.kD = DRIVE_D;
      DRIVE_CONFIG.MotorOutput.Inverted = DRIVE_MOTOR_INVERT;
      DRIVE_CONFIG.MotorOutput.NeutralMode = DRIVE_NEUTRAL_MODE;
      DRIVE_CONFIG.Feedback.SensorToMechanismRatio = SWERVE_CONSTANTS.driveGearRatio;
      DRIVE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = false;
      DRIVE_CONFIG.CurrentLimits.SupplyCurrentLimit = DRIVE_CURRENT_LIMIT.in(Units.Amps);

      STEER_CONFIG.Slot0.kP = STEER_P;
      STEER_CONFIG.Slot0.kI = STEER_I;
      STEER_CONFIG.Slot0.kD = STEER_D;
      STEER_CONFIG.MotorOutput.Inverted = STEER_MOTOR_INVERT;
      STEER_CONFIG.MotorOutput.NeutralMode = STEER_NEUTRAL_MODE;
      STEER_CONFIG.Feedback.SensorToMechanismRatio = SWERVE_CONSTANTS.steerGearRatio;
      STEER_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;

      CANCODER_CONFIG.MagnetSensor.SensorDirection = CANCODER_INVERT;
    }

    public static final double MIN_STEER_PERCENT = 0.01;
    public static final double SLOW_MODE_MULTIPLIER = 0.5;
    public static final double MINIMUM_ELEVATOR_MULTIPLIER = 0.1;

    // Rotational speed (degrees per second) while manually driving
    public static final AngularVelocity TURN_SPEED = Units.DegreesPerSecond.of(360);

    /**
     * <p>
     * Pose estimator standard deviation for encoder & gyro data
     * </p>
     * <b>Units:</b> Meters
     */
    public static final double MEASUREMENT_STD_DEVS_POS = 0.05;

    /**
     * <p>
     * Pose estimator standard deviation for encoder & gyro data
     * </p>
     * <b>Units:</b> Radians
     */
    public static final double MEASUREMENT_STD_DEV_HEADING = Units.Radians.convertFrom(5, Units.Degrees);

    public static class AUTO {
      // This PID is implemented on the Drivetrain subsystem
      public static final double AUTO_DRIVE_P = 9;
      public static final double AUTO_DRIVE_I = 0;
      public static final double AUTO_DRIVE_D = 0;
      public static final PIDConstants AUTO_DRIVE_PID = new PIDConstants(constDrivetrain.AUTO.AUTO_DRIVE_P,
          constDrivetrain.AUTO.AUTO_DRIVE_I,
          constDrivetrain.AUTO.AUTO_DRIVE_D);

      public static final double AUTO_STEER_P = 5.6; // 5.7 is also pretty good if we begin seeing undershooting
      public static final double AUTO_STEER_I = 0.0;
      public static final double AUTO_STEER_D = 0.0;
      public static final PIDConstants AUTO_STEER_PID = new PIDConstants(constDrivetrain.AUTO.AUTO_STEER_P,
          constDrivetrain.AUTO.AUTO_STEER_I,
          constDrivetrain.AUTO.AUTO_STEER_D);

      public static final Mass MASS = Units.Kilograms.of(20);
      // TODO: Calcuate the real vaule
      public static final double MOI = 8.0;
      public static final double WHEEL_COF = 1.0;
      public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(1);
      public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(WHEEL_RADIUS, OBSERVED_DRIVE_SPEED, WHEEL_COF,
          DRIVE_MOTOR,
          DRIVE_CURRENT_LIMIT, 1);

      public static final Translation2d[] MODULE_OFFSETS = {
          new Translation2d(WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
          new Translation2d(WHEELBASE / 2.0, -TRACK_WIDTH / 2.0),
          new Translation2d(-WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
          new Translation2d(-WHEELBASE / 2.0, -TRACK_WIDTH / 2.0) };

      public static final RobotConfig ROBOT_CONFIG = new RobotConfig(MASS.in(Kilograms), MOI, MODULE_CONFIG,
          MODULE_OFFSETS);

    }

    public static class TELEOP_AUTO_ALIGN {
      public static final Distance MAX_AUTO_DRIVE_REEF_DISTANCE = Units.Meters.of(2);
      public static final Distance MAX_AUTO_DRIVE_ALGAE_DISTANCE = Units.Meters.of(2);
      public static final LinearVelocity MIN_DRIVER_OVERRIDE = constDrivetrain.OBSERVED_DRIVE_SPEED.div(10);

      public static final PIDController TRANS_CONTROLLER = new PIDController(
          4,
          0,
          0);
      public static final Distance AT_POINT_TOLERANCE = Units.Inches.of(0.5);

      public static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
          3, 0, 0, new TrapezoidProfile.Constraints(TURN_SPEED.in(Units.DegreesPerSecond),
              Math.pow(TURN_SPEED.in(Units.DegreesPerSecond), 2)));
      public static final Angle AT_ROTATION_TOLERANCE = Units.Degrees.of(1);

      public static final Distance AUTO_ALIGNMENT_CORAL_TOLERANCE = Units.Inches.of(1);
      public static final Distance AUTO_ALIGNMENT_ALGAE_TOLERANCE = Units.Inches.of(2);

      static {
        TRANS_CONTROLLER.setTolerance(AT_POINT_TOLERANCE.in(Units.Meters));

        ROTATION_CONTROLLER.enableContinuousInput(0, 360);
        ROTATION_CONTROLLER.setTolerance(AT_ROTATION_TOLERANCE.in(Units.Degrees));
      }

      public static HolonomicDriveController TELEOP_AUTO_ALIGN_CONTROLLER = new HolonomicDriveController(
          TRANS_CONTROLLER,
          TRANS_CONTROLLER,
          ROTATION_CONTROLLER);
    }
  }

  public static class constAlgaeIntake {
    public static final double ALGAE_INTAKE_SPEED = 1;
    public static final double ALGAE_OUTTAKE_PROCESSOR_SPEED = -0.15;
    public static final double ALGAE_OUTTAKE_EJECT_SPEED = -0.35;
    public static final double ALGAE_OUTTAKE_NET_SPEED = -0.5;
    public static final double CLIMB_ALGAE_VELOCITY = -0.3;

    public static final Angle INTAKE_DEADZONE_DISTANCE = Units.Degrees.of(1); // TODO: Tune this

    /**
     * The velocity that the motor goes at once it has zeroed (and can no longer
     * continue in that direction)
     */
    public static final AngularVelocity ZEROED_VELOCITY = Units.RotationsPerSecond.of(0.2);

    public static final Angle MAX_POS = Units.Degrees.of(57);
    public static final Angle MIN_POS = Units.Degrees.of(-37);

    public static final Angle ZEROED_MANUAL_POS = Units.Degrees.of(57);
    public static final Angle ZEROED_AUTO_POS = Units.Degrees.of(59);

    /**
     * The elapsed time required to consider the motor as zeroed
     */
    public static final Time ZEROED_TIME = Units.Seconds.of(1);

    public static final Voltage ZEROING_VOLTAGE = Units.Volts.of(1);

    public static final double HOLD_ALGAE_INTAKE_VOLTAGE = 1;
    public static final TalonFXConfiguration ALGAE_ROLLER_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration ALGAE_PIVOT_CONFIG = new TalonFXConfiguration();
    static {
      ALGAE_ROLLER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ALGAE_ROLLER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      ALGAE_PIVOT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ALGAE_PIVOT_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      ALGAE_ROLLER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      ALGAE_ROLLER_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 30;
      ALGAE_ROLLER_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
      ALGAE_ROLLER_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 0.5;

      ALGAE_PIVOT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      ALGAE_PIVOT_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_POS.in(Units.Rotations);
      ALGAE_PIVOT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      ALGAE_PIVOT_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_POS.in(Units.Rotations);

      // Why don't scientists trust atoms? Because they make up everything!
      // Why do crabs never share their things? - Because they are shellfish!

      ALGAE_PIVOT_CONFIG.Feedback.SensorToMechanismRatio = 1000 / 27;

      ALGAE_PIVOT_CONFIG.Slot0.kG = 0.53; // Volts to overcome gravity
      ALGAE_PIVOT_CONFIG.Slot0.kS = 0.5; // Volts to overcome static friction
      ALGAE_PIVOT_CONFIG.Slot0.kV = 0.0; // Volts for a velocity target of 1 rps
      ALGAE_PIVOT_CONFIG.Slot0.kA = 0.0; // Volts for an acceleration of 1 rps/s
      ALGAE_PIVOT_CONFIG.Slot0.kP = 18;
      ALGAE_PIVOT_CONFIG.Slot0.kI = 0.0;
      ALGAE_PIVOT_CONFIG.Slot0.kD = 0.00;
      ALGAE_PIVOT_CONFIG.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      ALGAE_PIVOT_CONFIG.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

      ALGAE_PIVOT_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 9999;
      ALGAE_PIVOT_CONFIG.MotionMagic.MotionMagicAcceleration = 9999;

      ALGAE_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      ALGAE_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 30;
      ALGAE_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLimit = 45;
      ALGAE_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 0.5;
    }

    public static final Distance REQUIRED_ALGAE_DISTANCE = Units.Inches.of(2);

    public static final AngularVelocity ALGAE_INTAKE_HAS_GP_VELOCITY = Units.RotationsPerSecond.of(2102 / 60);

    public static final Current ALGAE_INTAKE_HAS_GP_CURRENT = Units.Amps.of(15);

    public static final Angle CLEANING_REEF_L2_PIVOT_POSITION = Units.Degrees.of(40);
    public static final Angle CLEANING_REEF_L3_PIVOT_POSITION = Units.Degrees.of(40);

    public static final Angle INTAKE_ALGAE_GROUND_PIVOT_POSITION = Units.Degrees.of(-33);
    public static final Angle CORAL_ONLY = MAX_POS; // no sag :p
    public static final Angle COLLECTED_ALGAE = Units.Degrees.of(47.38);
    public static final Angle PREP_ALGAE_ZERO_PIVOT_POSITION = Units.Degrees.of(55);
    public static final Angle PREP_NET_PIVOT_POSITION = Units.Degrees.of(44.14); // the tides comin in hahahah
    public static final Angle PREP_PROCESSOR_PIVOT_POSITION = Units.Degrees.of(-18);
    public static final Angle PREP_PROCESSOR_POS_WITH_CORAL = Units.Degrees.of(14);

    public static final Angle EJECT_ALGAE_PIVOT_POSITION = Units.Degrees.of(15);

    public static final Angle CLIMB_DEPLOY_POSITION = MAX_POS;

    public static final Time ZEROING_TIMEOUT = Units.Seconds.of(3);

    public static final AngularVelocity MANUAL_ZEROING_START_VELOCITY = Units.RotationsPerSecond.of(5);
    public static final AngularVelocity MANUAL_ZEROING_DELTA_VELOCITY = Units.RotationsPerSecond.of(5);

    public static final Transform3d ALGAE_INTAKE_TO_ALGAE = new Transform3d(
        Units.Meters.convertFrom(450, Units.Millimeters), 0,
        Units.Meters.convertFrom(-9, Units.Inches),
        Rotation3d.kZero);

    public static final Angle DEADZONE_DISTANCE = Units.Degrees.of(1);

  }

  public static class constCoralOuttake {
    public static final double CORAL_REVERSE_OUTTAKE_SPEED = -0.7;
    public static final double CORAL_L1_OUTTAKE_SPEED = 0.4;
    public static final double CORAL_OUTTAKE_SPEED = 0.3;
    public static final double CORAL_L4_OUTTAKE_SPEED = 0.4; // perfecto

    public static final double CORAL_INTAKE_SPEED = 1;
    public static final double CORAL_INDEXING_SPEED = 1;

    public static final Distance REQUIRED_CORAL_DISTANCE = Units.Meters.of(0.1);
    public static final Distance INDEXED_CORAL_DISTANCE = Units.Meters.of(0.13);

    public static final Time CORAL_SCORE_TIME = Units.Seconds.of(0.3);

    public static TalonFXConfiguration CORAL_OUTTAKE_CONFIG = new TalonFXConfiguration();
    public static CANrangeConfiguration CORAL_SENSOR_CONFIG = new CANrangeConfiguration();
    static {
      CORAL_OUTTAKE_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      CORAL_OUTTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      CORAL_SENSOR_CONFIG.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
      CORAL_SENSOR_CONFIG.ProximityParams.ProximityThreshold = REQUIRED_CORAL_DISTANCE.in(Units.Meters);

      CORAL_OUTTAKE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      CORAL_OUTTAKE_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 30;
      CORAL_OUTTAKE_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
      CORAL_OUTTAKE_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 0.5;
    }
  }

  public static class constClimber {
    public static final double CLIMBER_MOTOR_DEPLOYING_VELOCITY = 1;
    public static final double CLIMBER_RETRACT_VELOCITY = -1;
    public static final double MANUAL_CLIMBER_MOTOR_DEPLOYING_VELOCITY = 1;
    public static final Angle VALID_NONE_STATE_THRESHOLD = Units.Rotations.of(50);

    public static TalonFXConfiguration CLIMBER_CONFIG = new TalonFXConfiguration();
    public static Angle MAX_POSITION = Units.Rotations.of(166.69);
    public static Angle AT_POSITION_TOLERANCE = Units.Rotations.of(10);

    public static final double MATCH_CLIMBING_TIME = 50.0;

    static {
      CLIMBER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLimit = 85;
      CLIMBER_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 60;

      CLIMBER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      CLIMBER_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      CLIMBER_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_POSITION.in(Units.Rotations);
      CLIMBER_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      CLIMBER_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Rotations.of(0)
          .in(Units.Rotations);

    }
  }

  public static class constElevator {
    public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
    static {
      ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Inches.of(62).in(Units.Inches);
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Inches.of(0)
          .in(Units.Inches);

      ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
      // Elevator motors will provide feedback in INCHES the carriage has moved
      ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio = 0.876;

      ELEVATOR_CONFIG.Slot0.kG = 0.3; // Volts to overcome gravity
      ELEVATOR_CONFIG.Slot0.kS = 0.4; // Volts to overcome static friction
      ELEVATOR_CONFIG.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
      ELEVATOR_CONFIG.Slot0.kA = 0.0; // Volts for an acceleration of 1 rps/
      ELEVATOR_CONFIG.Slot0.kP = 0.85;
      ELEVATOR_CONFIG.Slot0.kI = 0.0;
      ELEVATOR_CONFIG.Slot0.kD = 0.0;
      ELEVATOR_CONFIG.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

      ELEVATOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 0;
      ELEVATOR_CONFIG.MotionMagic.MotionMagicAcceleration = 0;
      ELEVATOR_CONFIG.MotionMagic.MotionMagicExpo_kV = 0.04;
      ELEVATOR_CONFIG.MotionMagic.MotionMagicExpo_kA = 0.005;

      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 30;
      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 1;

    }

    public static TalonFXConfiguration COAST_MODE_CONFIGURATION = new TalonFXConfiguration();
    static {
      COAST_MODE_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      COAST_MODE_CONFIGURATION.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }

    public static final Distance CORAL_L1_HEIGHT = Units.Inches.of(6);
    public static final Distance CORAL_L2_HEIGHT = Units.Inches.of(20);
    public static final Distance CORAL_L3_HEIGHT = Units.Inches.of(36.47);
    public static final Distance CORAL_L4_HEIGHT = Units.Inches.of(61);
    public static final Distance ALGAE_PREP_NET = Units.Inches.of(61);
    public static final Distance ALGAE_PREP_PROCESSOR_HEIGHT = Units.Inches.of(1);
    public static final Distance ALGAE_L3_CLEANING = Units.Inches.of(25);
    public static final Distance ALGAE_L2_CLEANING = Units.Inches.of(9);
    public static final Distance ALGAE_GROUND_INTAKE = Units.Inches.of(0);
    public static final Distance PREP_0 = Units.Inches.of(2.8);
    public static final Distance DEADZONE_DISTANCE = Units.Inches.of(1);
    public static final Distance EJECT_DEADZONE = Units.Inches.of(0.5);
    public static final Distance CORAL_INTAKE_HIGHT = Units.Inches.of(0);
    public static final Distance INIT_TIP_HEIGHT = Units.Inches.of(30);
    public static final Distance AFTER_L1_HEIGHT = Units.Inches.of(19);
    public static final Distance EJECT_HOPPER_HEIGHT = Units.Inches.of(2);
    public static final Distance MAX_HEIGHT = Units.Inches.of(62);

    public static final Time ZEROING_TIMEOUT = Units.Seconds.of(3);

    public static final AngularVelocity MANUAL_ZEROING_START_VELOCITY = Units.RotationsPerSecond.of(5);
    public static final AngularVelocity MANUAL_ZEROING_DELTA_VELOCITY = Units.RotationsPerSecond.of(5);

    /**
     * The voltage supplied to the motor in order to zero
     */
    public static final Voltage ZEROING_VOLTAGE = Units.Volts.of(-1);

    /**
     * The value that the motor reports when it is at it's zeroed position. This
     * may not necessarily be 0 due to mechanical slop
     */
    public static final Distance ZEROED_POS = Units.Meters.of(0);

    /**
     * The velocity that the motor goes at once it has zeroed (and can no longer
     * continue in that direction)
     */
    public static final AngularVelocity ZEROED_VELOCITY = Units.RotationsPerSecond.of(0.2);

    /**
     * The elapsed time required to consider the motor as zeroed
     */
    public static final Time ZEROED_TIME = Units.Seconds.of(1);

    public static final Transform3d CARRIAGE_TO_CORAL = new Transform3d(
        Units.Meters.convertFrom(194, Units.Millimeters), 0,
        Units.Meters.convertFrom(318 + 40, Units.Millimeters),
        new Rotation3d(0, Units.Radians.convertFrom(35, Units.Degrees), 0));
  }

  public static class constField {
    public static Optional<Alliance> ALLIANCE = Optional.empty();
    public static final Distance FIELD_LENGTH = Units.Feet.of(57).plus(Units.Inches.of(6 + 7 / 8));
    public static final Distance FIELD_WIDTH = Units.Feet.of(26).plus(Units.Inches.of(5));

    /**
     * Boolean that controls when the path will be mirrored for the red
     * alliance. This will flip the path being followed to the red side of the
     * field.
     * The origin will remain on the Blue side.
     * 
     * @return If we are currently on Red alliance. Will return false if no alliance
     *         is found
     */
    public static boolean isRedAlliance() {
      var alliance = ALLIANCE;
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };

    /*
     * All poses on the field, defined by their location on the BLUE Alliance
     */
    public static class POSES {
      public static final Pose2d RESET_POSE = new Pose2d(3.169, 4.015, new Rotation2d());
      public static final Pose3d SCORING_ELEMENT_NOT_COLLECTED = new Pose3d(0, 0, -1, Rotation3d.kZero);

      // BRANCH POSES
      private static final Pose2d REEF_A = new Pose2d(3.171, 4.189, Rotation2d.kZero);
      private static final Pose2d REEF_B = new Pose2d(3.171, 3.863, Rotation2d.kZero);
      private static final Pose2d REEF_C = new Pose2d(3.688, 2.968, Rotation2d.fromDegrees(60));
      private static final Pose2d REEF_D = new Pose2d(3.975, 2.803, Rotation2d.fromDegrees(60));
      private static final Pose2d REEF_E = new Pose2d(5.001, 2.804, Rotation2d.fromDegrees(120));
      private static final Pose2d REEF_F = new Pose2d(5.285, 2.964, Rotation2d.fromDegrees(120));
      private static final Pose2d REEF_G = new Pose2d(5.7288, 3.863, Rotation2d.k180deg);
      private static final Pose2d REEF_H = new Pose2d(5.7288, 4.189, Rotation2d.k180deg);
      private static final Pose2d REEF_I = new Pose2d(5.288, 5.083, Rotation2d.fromDegrees(-120));
      private static final Pose2d REEF_J = new Pose2d(5.002, 5.248, Rotation2d.fromDegrees(-120));
      private static final Pose2d REEF_K = new Pose2d(3.972, 5.247, Rotation2d.fromDegrees(-60));
      private static final Pose2d REEF_L = new Pose2d(3.693, 5.079, Rotation2d.fromDegrees(-60));
      private static final Pose2d REEF_CENTER = new Pose2d(4.490, 4.026, Rotation2d.kZero);

      public static final Pose2d ALGAE_AB = REEF_A.interpolate(REEF_B, 0.5);
      public static final Pose2d ALGAE_CD = REEF_C.interpolate(REEF_D, 0.5);
      public static final Pose2d ALGAE_EF = REEF_E.interpolate(REEF_F, 0.5);
      public static final Pose2d ALGAE_GH = REEF_G.interpolate(REEF_H, 0.5);
      public static final Pose2d ALGAE_IJ = REEF_I.interpolate(REEF_J, 0.5);
      public static final Pose2d ALGAE_KL = REEF_K.interpolate(REEF_L, 0.5);
      private static final List<Pose2d> BLUE_REEF_POSES = List.of(REEF_A, REEF_B, REEF_C, REEF_D, REEF_E,
          REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L);
      private static final List<Pose2d> RED_REEF_POSES = getRedReefPoses();

      private static final List<Pose2d> BLUE_ALGAE_POSES = List.of(ALGAE_AB, ALGAE_CD, ALGAE_EF, ALGAE_GH, ALGAE_IJ,
          ALGAE_KL);
      private static final List<Pose2d> RED_ALGAE_POSES = getRedAlgaePoses();

      // Coral Stations
      private static final Pose2d LEFT_CORAL_STATION_FAR = new Pose2d(1.64, 7.33, Rotation2d.fromDegrees(-55));
      private static final Pose2d LEFT_CORAL_STATION_NEAR = new Pose2d(0.71, 6.68, Rotation2d.fromDegrees(-55));
      private static final Pose2d RIGHT_CORAL_STATION_FAR = new Pose2d(1.61, 0.70, Rotation2d.fromDegrees(55));
      private static final Pose2d RIGHT_CORAL_STATION_NEAR = new Pose2d(0.64, 1.37, Rotation2d.fromDegrees(55));

      private static final List<Pose2d> BLUE_CORAL_STATION_POSES = List.of(LEFT_CORAL_STATION_FAR,
          LEFT_CORAL_STATION_NEAR, RIGHT_CORAL_STATION_FAR, RIGHT_CORAL_STATION_NEAR);
      private static final List<Pose2d> RED_CORAL_STATION_POSES = getRedCoralStationPoses();

      // Processor
      private static final Pose2d BLUE_PROCESSOR = new Pose2d(5.986, 0.896, Rotation2d.fromDegrees(-90));
      private static final List<Pose2d> PROCESSOR_POSES = List.of(BLUE_PROCESSOR, getRedAlliancePose(BLUE_PROCESSOR));

      private static final Pose2d[] BLUE_POSES = new Pose2d[] { RESET_POSE, REEF_A, REEF_B, REEF_C, REEF_D, REEF_E,
          REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L, REEF_CENTER, BLUE_PROCESSOR, LEFT_CORAL_STATION_FAR,
          LEFT_CORAL_STATION_NEAR, RIGHT_CORAL_STATION_FAR, RIGHT_CORAL_STATION_NEAR, ALGAE_AB, ALGAE_CD, ALGAE_EF,
          ALGAE_GH, ALGAE_IJ, ALGAE_KL };
      private static final Pose2d[] RED_POSES = getRedPosesFromArray(BLUE_POSES);
    }

    public static Pose2d getRedAlliancePose(Pose2d bluePose) {
      return new Pose2d(FIELD_LENGTH.in(Units.Meters) - (bluePose.getX()),
          FIELD_WIDTH.in(Units.Meters) - bluePose.getY(),
          bluePose.getRotation().plus(Rotation2d.k180deg));
    }

    private static Pose2d[] getRedPosesFromArray(Pose2d[] bluePoseArray) {
      Pose2d[] returnedPoses = new Pose2d[bluePoseArray.length];
      for (int i = 0; i < bluePoseArray.length; i++) {
        returnedPoses[i] = getRedAlliancePose(bluePoseArray[i]);
      }
      return returnedPoses;
    }

    private static Pose2d[] getRedPosesFromList(List<Pose2d> bluePoseList) {
      Pose2d[] returnedPoses = new Pose2d[bluePoseList.size()];
      for (int i = 0; i < bluePoseList.size(); i++) {
        returnedPoses[i] = getRedAlliancePose(bluePoseList.get(i));
      }
      return returnedPoses;
    }

    private static List<Pose2d> getRedReefPoses() {
      Pose2d[] returnedPoses = getRedPosesFromList(POSES.BLUE_REEF_POSES);
      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3], returnedPoses[4],
          returnedPoses[5], returnedPoses[6], returnedPoses[7], returnedPoses[8], returnedPoses[9], returnedPoses[10],
          returnedPoses[11]);
    }

    private static List<Pose2d> getRedCoralStationPoses() {
      Pose2d[] returnedPoses = getRedPosesFromList(POSES.BLUE_CORAL_STATION_POSES);
      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3]);
    }

    private static List<Pose2d> getRedAlgaePoses() {
      Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_ALGAE_POSES.size()];

      for (int i = 0; i < POSES.BLUE_ALGAE_POSES.size(); i++) {
        returnedPoses[i] = getRedAlliancePose(POSES.BLUE_ALGAE_POSES.get(i));
      }

      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3], returnedPoses[4],
          returnedPoses[5]);
    }

    /**
     * Gets the positions of ALL of the field elements on your alliance. All
     * coordinates are in meters and are relative to the blue alliance.
     * <li>0 = Reset Pose</li>
     * <li>1-12 = Reef Branches</li>
     * <li>13 = Reef Center</li>
     * <li>14 = Processor</li>
     * 
     * @see <a href=
     *      "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
     *      Robot Coordinate Systems</a>
     * @return An array of field element positions
     */
    public static Supplier<Pose2d[]> getAllFieldPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> POSES.RED_POSES;

      }
      return () -> POSES.BLUE_POSES;
    }

    /**
     * Gets the positions of all of branches for your alliance. All
     * coordinates are in meters and are relative to the blue alliance.
     * 
     * @see <a href=
     *      https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
     *      Robot Coordinate Systems</a>
     * @return An array of the reef branches for your alliance
     */
    public static Supplier<List<Pose2d>> getReefPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> POSES.RED_REEF_POSES;

      }
      return () -> POSES.BLUE_REEF_POSES;
    }

    public static Supplier<List<Pose2d>> getCoralStationPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> POSES.RED_CORAL_STATION_POSES;
      }
      return () -> POSES.BLUE_CORAL_STATION_POSES;
    }

    public static Supplier<List<Pose2d>> getProcessorPositions() {
      return () -> POSES.PROCESSOR_POSES;
    }

    public static Supplier<List<Pose2d>> getAlgaePositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> POSES.RED_ALGAE_POSES;
      }
      return () -> POSES.BLUE_ALGAE_POSES;
    }
  }

  public static class constVision {
    public static final String[] LIMELIGHT_NAMES = new String[] { "limelight-right", "limelight-left",
        "limelight-back" };

    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * <p>
     * <b>Units:</b> Meters
     */
    public static final double MEGA_TAG2_STD_DEVS_POSITION = 0.7;

    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * </p>
     * <b>Units:</b> Radians
     */
    public static final double MEGA_TAG2_STD_DEVS_HEADING = 9999999;

    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * </p>
     * <b>Units:</b> Meters
     */
    public static final double MEGA_TAG1_STD_DEVS_POSITION = .3;

    public static final double MEGA_TAG1_STD_DEVS_HEADING = .1;
    /**
     * <p>
     * Maximum rate of rotation before we begin rejecting pose updates
     * </p>
     */
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = Units.DegreesPerSecond.of(720);

    /**
     * The area that one tag (if its the only tag in the update) needs to exceed
     * before being accepted
     */
    public static final double AREA_THRESHOLD = 0.1;

    // The below values are accounted for in the limelight interface, NOT in code
    public static class LIMELIGHT_RIGHT {
      public static final Distance LL_FORWARD = Units.Meters.of(0.269494);
      public static final Distance LL_RIGHT = Units.Meters.of(0.307594);
      public static final Distance LL_UP = Units.Meters.of(0.211328);

      public static final Angle LL_ROLL = Units.Degrees.of(180);
      public static final Angle LL_PITCH = Units.Degrees.of(23.17);
      public static final Angle LL_YAW = Units.Degrees.of(51.25);
    }

    public static class LIMELIGHT_LEFT {
      public static final Distance LL_FORWARD = Units.Meters.of(0.269494);
      public static final Distance LL_RIGHT = Units.Meters.of(-0.307594);
      public static final Distance LL_UP = Units.Meters.of(0.211328);

      public static final Angle LL_ROLL = Units.Degrees.of(180);
      public static final Angle LL_PITCH = Units.Degrees.of(23.17);
      public static final Angle LL_YAW = Units.Degrees.of(-51.25);

    }
  }

  public static class constHopper {
    public static final double HOPPER_SPEED = 1;
    public static final double HOPPER_INDEXING_SPEED = 1;
    public static final double HOPPER_EJECTING_SPEED = -1.0;

    public static final TalonFXConfiguration HOPPER_CONFIG = new TalonFXConfiguration();

    static {
      HOPPER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      HOPPER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
      HOPPER_CONFIG.CurrentLimits.StatorCurrentLimit = 60;
    }
  }

  public static class constLED {
    public static final CANdleConfiguration LED_CONFIG = new CANdleConfiguration();
    static {
      LED_CONFIG.brightnessScalar = 1;

    }
    public static final int LED_NUMBER = 8;
    public static final int LED_STRIP_START_INDEX = 1;

    // Climbing
    public static final int[] LED_CLIMBER_DEPLOYING = null;
    public static final int[] LED_CLIMBER_RETRACTING = null;
    public static final int[] LED_MANUAL_CLIMBER_DEPLOYING = null;

    // first scoring element
    public static final int[] LED_CLEANING_L2_REEF = null;
    public static final int[] LED_CLEANING_L3_REEF = null;
    public static final int[] LED_EJECT_CORAL = null;
    public static final int[] LED_INTAKE_CORAL_HOPPER = { 0, 0, 255 };// blue
    public static final int[] LED_INTAKE_ALGAE_GROUND = null;

    // hold scoring elements

    // TODO: Implement CANdle being commanded half & half (top half w/ algae, bottom
    // half w/ coral)
    public static final int[] LED_HAS_ALGAE = { 89, 252, 236 }; // algae-side teal
    public static final int[] LED_HAS_CORAL = { 255, 255, 255 }; // elevator-side white
    public static final int[] LED_HAS_BOTH = null;

    // prep algae
    public static final int[] LED_PREP_ALGAE_ZERO = null;
    public static final int[] LED_PREP_ALGAE_ZERO_WITH_CORAL = null;
    public static final int[] LED_PREP_NET = null;
    public static final int[] LED_PREP_NET_WITH_CORAL = null;
    public static final int[] LED_PREP_PROCESSOR = null;
    public static final int[] LED_PREP_PROCESSOR_WITH_CORAL = null;

    // prep coral
    public static final int[] LED_PREP_CORAL_LV = null;
    public static final int[] LED_PREP_CORAL_LV_WITH_ALGAE = null;
    public static final int[] LED_PREP_CORAL_ZERO = null;
    public static final int[] LED_PREP_CORAL_ZERO_WITH_ALGAE = null;

    // scoring
    public static final int[] LED_SCORING_ALGAE = null;
    public static final int[] LED_SCORING_ALGAE_WITH_CORAL = null;
    public static final int[] LED_PLACE_CORAL = null;
    public static final int[] LED_PLACE_CORAL_WITH_ALGAE = null;

    // second scoring element
    public static final int[] LED_CLEANING_L2_REEF_WITH_CORAL = { 0, 0, 0 };
    public static final int[] LED_CLEANING_L3_REEF_WITH_CORAL = { 0, 0, 0 };
    public static final int[] LED_INTAKE_CORAL_WITH_ALGAE = { 0, 0, 255 };// blue
    // that other guy what a loser
    public static final int[] LED_NONE = { 0, 0, 0 }; // no color

    // Zeroing
    public static final int[] ALGAE_ZERO_FAILED = { 255, 0, 0 }; // red
    public static final int[] ALGAE_ZERO_SUCCESS = { 0, 0, 0 }; // no color
    public static final int[] ELEVATOR_ZERO_FAILED = { 255, 0, 0 }; // red
    public static final int[] ELEVATOR_ZERO_SUCCESS = { 0, 0, 0 }; // no color

    public static final StrobeAnimation READY_TO_LIFT = new StrobeAnimation(252, 179, 89,
        0, 0.05, LED_NUMBER, LED_STRIP_START_INDEX); // orange

    public static final StrobeAnimation READY_TO_PLACE = new StrobeAnimation(200, 89, 252,
        0, 0.05, LED_NUMBER, LED_STRIP_START_INDEX); // purple

    public static final StrobeAnimation READY_TO_LEAVE = new StrobeAnimation(0, 255, 0,
        0, 0.05, LED_NUMBER, LED_STRIP_START_INDEX); // green

    public static final StrobeAnimation ENDGAME_TIMER = new StrobeAnimation(255, 0, 0,
        0, 0.05, LED_NUMBER, LED_STRIP_START_INDEX); // red

  }

}
