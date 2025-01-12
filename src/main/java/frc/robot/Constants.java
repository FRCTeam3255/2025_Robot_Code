// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.frcteam3255.components.swerve.SN_SwerveConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
  /**
   * Volts
   */
  public static final double MAX_VOLTAGE = 12;

  public static class constControllers {
    public static final double DRIVER_LEFT_STICK_DEADBAND = 0.05;
    public static final boolean SILENCE_JOYSTICK_WARNINGS = true;
  }

  public static class constDrivetrain {
    // TODO: Convert all applicable fields to MEASUREs (Standard_Swerve_Code)
    public static final SN_SwerveConstants SWERVE_CONSTANTS = new SN_SwerveConstants(
        SN_SwerveConstants.MK4I.FALCON.L2.steerGearRatio,
        0.09779 * Math.PI,
        SN_SwerveConstants.MK4I.FALCON.L2.driveGearRatio,
        SN_SwerveConstants.MK4I.FALCON.L2.maxSpeedMeters);

    // In Rotations: Obtain by aligning all of the wheels in the correct direction
    // and
    // copy-pasting the Raw Absolute Encoder value
    public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = 0.390625;
    public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET = 0.433594;
    public static final double BACK_LEFT_ABS_ENCODER_OFFSET = 0.450684;
    public static final double BACK_RIGHT_ABS_ENCODER_OFFSET = 0.814209;

    public static final double WHEEL_DIAMETER = 0.09779;
    public static final Distance WHEEL_RADIUS = Units.Meters.of(WHEEL_DIAMETER / 2);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    // Taken from the online listing
    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0;

    /**
     * <p>
     * Theoretical maximum translational speed while manually driving on the
     * Competition Robot.
     * </p>
     * <b>Units:</b> Meters Per Second
     */
    public static final double THEORETICAL_MAX_DRIVE_SPEED = SWERVE_CONSTANTS.maxSpeedMeters;

    /**
     * <p>
     * Observed maximum translational speed while manually driving on the
     * Competition Robot.
     * </p>
     */
    // TODO: Find the actual max speed
    public static final LinearVelocity OBSERVED_DRIVE_SPEED = Units.MetersPerSecond.of(THEORETICAL_MAX_DRIVE_SPEED);
    // Physically measured from center to center of the wheels
    // Distance between Left & Right Wheels
    public static final double TRACK_WIDTH = Units.Meters.convertFrom(23.75, Units.Inches);
    // Distance between Front & Back Wheels
    public static final double WHEELBASE = Units.Meters.convertFrom(23.75, Units.Inches);

    public static final double AT_ROTATION_TOLERANCE = 0.1;
    public static final Distance AT_POINT_TOLERANCE = Units.Meters.of(0.1);

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
    public static final Current DRIVE_CURRENT_LIMIT = Units.Amps.of(90);

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
      DRIVE_CONFIG.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
      DRIVE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      DRIVE_CONFIG.CurrentLimits.SupplyCurrentLimit = DRIVE_CURRENT_LIMIT.in(Units.Amps);

      STEER_CONFIG.Slot0.kP = STEER_P;
      STEER_CONFIG.Slot0.kI = STEER_I;
      STEER_CONFIG.Slot0.kD = STEER_D;
      STEER_CONFIG.MotorOutput.Inverted = STEER_MOTOR_INVERT;
      STEER_CONFIG.MotorOutput.NeutralMode = STEER_NEUTRAL_MODE;
      STEER_CONFIG.Feedback.SensorToMechanismRatio = STEER_GEAR_RATIO;
      STEER_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;

      CANCODER_CONFIG.MagnetSensor.SensorDirection = CANCODER_INVERT;
    }

    // Teleop Snapping to Rotation (Yaw)
    public static final double YAW_SNAP_P = 3;
    public static final double YAW_SNAP_I = 0;
    public static final double YAW_SNAP_D = 0;

    public static final double MIN_STEER_PERCENT = 0.01;
    public static final double SLOW_MODE_MULTIPLIER = 0.5;

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
      public static final double AUTO_DRIVE_P = 8;
      public static final double AUTO_DRIVE_I = 0;
      public static final double AUTO_DRIVE_D = 0;

      public static final double AUTO_STEER_P = 2.5;
      public static final double AUTO_STEER_I = 0.0;
      public static final double AUTO_STEER_D = 0.0;

      // Feet
      public static final double AUTO_MAX_SPEED = 8;
      // Feet per second
      public static final double AUTO_MAX_ACCEL = 6;

      public static final double MASS = 125;
      public static final double MOI = 125;
      public static final double WHEEL_COF = 1.0;
      public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(0);
      public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(WHEEL_RADIUS, OBSERVED_DRIVE_SPEED, WHEEL_COF,
          DRIVE_MOTOR,
          DRIVE_CURRENT_LIMIT, 1);

      public static final Translation2d[] MODULE_OFFSETS = {
          new Translation2d(WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
          new Translation2d(WHEELBASE / 2.0, -TRACK_WIDTH / 2.0),
          new Translation2d(-WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
          new Translation2d(-WHEELBASE / 2.0, -TRACK_WIDTH / 2.0) };

      public static final RobotConfig ROBOT_CONFIG = new RobotConfig(MASS, MOI, MODULE_CONFIG, MODULE_OFFSETS);
    }
  }

  public static class constAlgaeIntake {
    public static final double ALGAE_INTAKE_SPEED = 0.2;
    public static final double ALGAE_OUTTAKE_SPEED = -0.2;
  }

  public static class constCoralOuttake {
    public static final double CORAL_OUTTAKE_SPEED = 0.3;
  }

  public static class consClimber {
    public static final double CLIMBER_MOTOR_VELOCITY = 0.5;
  }

  public static class constElevator {
    public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
    static {
      ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Inches.of(51).in(Units.Inches);
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Inches.of(0)
          .in(Units.Inches);

      ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
      // Elevator motors will provide feedback in INCHES the carriage has moved
      ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio = 0.4545;
      ELEVATOR_CONFIG.Slot0.kG = 0.3;
      ELEVATOR_CONFIG.Slot0.kS = 0.4;
      ELEVATOR_CONFIG.Slot0.kP = 1;
    }

    public static final Distance CORAL_L1_HEIGHT = Units.Inches.of(5);
    public static final Distance CORAL_L2_HEIGHT = Units.Inches.of(10);
    public static final Distance CORAL_L3_HEIGHT = Units.Inches.of(30);
    public static final Distance CORAL_L4_HEIGHT = Units.Inches.of(50);
    public static final Distance ALGAE_PREP_PROCESSOR_HEIGHT = Units.Inches.of(1);
    public static final Distance ALGAE_L3_CLEANING = Units.Inches.of(35);
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

    public static class bluePoses {
      public static final Pose2d RESET_POSE = new Pose2d(0, 0, new Rotation2d());

    }

    public static class redPoses {
      public static final Pose2d RESET_POSE = new Pose2d(FIELD_LENGTH, FIELD_WIDTH, new Rotation2d().fromDegrees(180));
    }

    /**
     * Gets the positions of all of the necessary field elements on the field. All
     * coordinates are in meters and are relative to the blue alliance.
     * 
     * @see <a href=
     *      https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
     *      Robot Coordinate Systems</a>
     * @return An array of field element positions in this order: ResetPose
     */
    public static Supplier<Pose2d[]> getFieldPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> new Pose2d[] { redPoses.RESET_POSE };

      }
      return () -> new Pose2d[] { bluePoses.RESET_POSE };
    }
  }

  public static class constVision {
    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * <p>
     * <b>Units:</b> Meters
     */
    public static final double STD_DEVS_POS = 0.7;

    /**
     * <p>
     * Pose estimator standard deviation for vision data
     * </p>
     * <b>Units:</b> Radians
     */
    public static final double STD_DEVS_HEADING = 9999999;

  }

  public static class constHopper {
    public static final double HOPPER_SPEED = 0.5;

    public static final TalonFXConfiguration HOPPER_CONFIG = new TalonFXConfiguration();

  }
}
