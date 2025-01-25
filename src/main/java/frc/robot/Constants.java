// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.frcteam3255.components.swerve.SN_SwerveConstants;
import com.frcteam3255.components.swerve.SN_SwerveModule;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotMap.mapDrivetrain;

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

    // TODO: Convert all applicable fields to MEASUREs (Standard_Swerve_Code)
    public static final SN_SwerveConstants SWERVE_CONSTANTS = new SN_SwerveConstants(
        SN_SwerveConstants.MK4I.FALCON.L2.steerGearRatio,
        0.09779 * Math.PI,
        SN_SwerveConstants.MK4I.FALCON.L2.driveGearRatio,
        SN_SwerveConstants.MK4I.FALCON.L2.maxSpeedMeters);

    // In Rotations: Obtain by aligning all of the wheels in the correct direction
    // and
    // copy-pasting the Raw Absolute Encoder value
    public static final double FRONT_LEFT_ABS_ENCODER_OFFSET = 0.59082;
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

    public static final double WHEEL_DIAMETER = 0.09779;
    public static final Distance WHEEL_RADIUS = Units.Meters.of(WHEEL_DIAMETER / 2);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

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
      public static final double AUTO_DRIVE_P = 6;
      public static final double AUTO_DRIVE_I = 0;
      public static final double AUTO_DRIVE_D = 0;
      public static final PIDConstants AUTO_DRIVE_PID = new PIDConstants(constDrivetrain.AUTO.AUTO_DRIVE_P,
          constDrivetrain.AUTO.AUTO_DRIVE_I,
          constDrivetrain.AUTO.AUTO_DRIVE_D);

      public static final double AUTO_STEER_P = 2.5;
      public static final double AUTO_STEER_I = 0.0;
      public static final double AUTO_STEER_D = 0.0;
      public static final PIDConstants AUTO_STEER_PID = new PIDConstants(constDrivetrain.AUTO.AUTO_STEER_P,
          constDrivetrain.AUTO.AUTO_STEER_I,
          constDrivetrain.AUTO.AUTO_STEER_D);

      public static final double MASS = 115;
      // TODO: Calcuate the real vaule
      public static final double MOI = 6.8;
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

      public static final RobotConfig ROBOT_CONFIG = new RobotConfig(MASS, MOI, MODULE_CONFIG, MODULE_OFFSETS);

    }

    public static class TELEOP_AUTO_ALIGN {
      // TODO: Test if this actually works LOL
      public static final LinearVelocity DESIRED_AUTO_ALIGN_SPEED = Units.MetersPerSecond
          .of(THEORETICAL_MAX_DRIVE_SPEED / 4);

      public static final Distance MAX_AUTO_DRIVE_DISTANCE = Units.Meters.of(1);
      public static final LinearVelocity MIN_DRIVER_OVERRIDE = constDrivetrain.OBSERVED_DRIVE_SPEED.div(10);

      public static final PIDController TRANS_CONTROLLER = new PIDController(
          3,
          0,
          0);
      public static final Distance AT_POINT_TOLERANCE = Units.Meters.of(0.1);

      public static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
          3, 0, 0, new TrapezoidProfile.Constraints(TURN_SPEED.in(Units.DegreesPerSecond),
              Math.pow(TURN_SPEED.in(Units.DegreesPerSecond), 2)));
      public static final Angle AT_ROTATION_TOLERANCE = Units.Degrees.of(1);

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
    public static final double ALGAE_OUTTAKE_SPEED = -1;

    public static final double HOLD_ALGAE_INTAKE_VOLTAGE = -1;
    public static final TalonFXConfiguration ALGAE_INTAKE_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration ALGAE_PIVOT_CONFIG = new TalonFXConfiguration();
    static {
      ALGAE_INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ALGAE_INTAKE_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      ALGAE_PIVOT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ALGAE_PIVOT_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      ALGAE_PIVOT_CONFIG.Feedback.SensorToMechanismRatio = 1000 / 27;
    }

    public static final Distance REQUIRED_ALGAE_DISTANCE = Units.Inches.of(2);

    public static final AngularVelocity ALGAE_INTAKE_HAS_GP_VELOCITY = Units.RotationsPerSecond.of(-100);
    public static final Current ALGAE_INTAKE_HAS_GP_CURRENT = Units.Amps.of(18);

    public static final Distance ZEROED_POS = Units.Meters.of(0);

    public static final Measure<TimeUnit> ZEROING_TIMEOUT = Units.Seconds.of(3);

    public static final AngularVelocity MANUAL_ZEROING_START_VELOCITY = Units.RotationsPerSecond.of(7);
    public static final AngularVelocity MANUAL_ZEROING_DELTA_VELOCITY = Units.RotationsPerSecond.of(7);
  }

  public static class constCoralOuttake {
    public static final double CORAL_OUTTAKE_SPEED = 0.3;
    public static final double CORAL_INTAKE_SPEED = 0.3;
    public static final Distance REQUIRED_CORAL_DISTANCE = Units.Inches.of(2);

    public static TalonFXConfiguration CORAL_OUTTAKE_CONFIG = new TalonFXConfiguration();
    static {
      CORAL_OUTTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
  }

  public static class constClimber {
    public static final double CLIMBER_MOTOR_VELOCITY = 0.5;

    public static TalonFXConfiguration CLIMBER_CONFIG = new TalonFXConfiguration();
    static {
      CLIMBER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
  }

  public static class constElevator {
    public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
    static {
      ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Inches.of(66).in(Units.Inches);
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Inches.of(0)
          .in(Units.Inches);

      ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
      // Elevator motors will provide feedback in INCHES the carriage has moved
      ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio = 0.4545;
      ELEVATOR_CONFIG.Slot0.kG = 0.3;
      ELEVATOR_CONFIG.Slot0.kS = 0.4;
      // ELEVATOR_CONFIG.Slot0.kP = 1;
      ELEVATOR_CONFIG.Slot0.kP = 0.3;
    }

    public static final Distance CORAL_L1_HEIGHT = Units.Inches.of(9.039062);
    public static final Distance CORAL_L2_HEIGHT = Units.Inches.of(17.946289);
    public static final Distance CORAL_L3_HEIGHT = Units.Inches.of(33.742188);
    public static final Distance CORAL_L4_HEIGHT = Units.Inches.of(58.888916);
    public static final Distance ALGAE_PREP_NET = Units.Inches.of(50);
    public static final Distance ALGAE_PREP_PROCESSOR_HEIGHT = Units.Inches.of(1);
    public static final Distance ALGAE_L3_CLEANING = Units.Inches.of(35);
    public static final Distance ALGAE_L2_CLEANING = Units.Inches.of(25);
    public static final Distance ALGAE_GROUND_INTAKE = Units.Inches.of(0);
    public static final Distance PREP_0 = Units.Inches.of(0);
    public static final Distance DEADZONE_DISTANCE = Units.Inches.of(0.5);
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
      public static final Pose2d RESET_POSE = new Pose2d(0, 0, new Rotation2d());

      // BRANCH POSES
      public static final Pose2d REEF_A = new Pose2d(2.860, 4.187, Rotation2d.fromDegrees(0));
      public static final Pose2d REEF_B = new Pose2d(2.860, 3.857, Rotation2d.fromDegrees(0));
      public static final Pose2d REEF_C = new Pose2d(3.527, 2.694, Rotation2d.fromDegrees(60));
      public static final Pose2d REEF_D = new Pose2d(3.813, 2.535, Rotation2d.fromDegrees(60));
      public static final Pose2d REEF_E = new Pose2d(5.160, 2.529, Rotation2d.fromDegrees(120));
      public static final Pose2d REEF_F = new Pose2d(5.445, 2.694, Rotation2d.fromDegrees(120));
      public static final Pose2d REEF_G = new Pose2d(6.119, 3.857, Rotation2d.fromDegrees(180));
      public static final Pose2d REEF_H = new Pose2d(6.119, 4.187, Rotation2d.fromDegrees(180));
      public static final Pose2d REEF_I = new Pose2d(5.452, 5.343, Rotation2d.fromDegrees(-120));
      public static final Pose2d REEF_J = new Pose2d(5.166, 5.527, Rotation2d.fromDegrees(-120));
      public static final Pose2d REEF_K = new Pose2d(3.826, 5.508, Rotation2d.fromDegrees(-60));
      public static final Pose2d REEF_L = new Pose2d(3.534, 5.368, Rotation2d.fromDegrees(-60));

      private static final List<Pose2d> BLUE_REEF_POSES = List.of(REEF_A, REEF_B, REEF_C, REEF_D, REEF_E,
          REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L);
      private static final List<Pose2d> RED_REEF_POSES = getRedReefPoses();

      private static final Pose2d[] BLUE_POSES = new Pose2d[] { RESET_POSE, REEF_A, REEF_B, REEF_C, REEF_D, REEF_E,
          REEF_F,
          REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L };

      private static final Pose2d[] RED_POSES = getRedAlliancePoses();
    }

    public static Pose2d getRedAlliancePose(Pose2d bluePose) {
      return new Pose2d(FIELD_LENGTH.in(Units.Meters) - (bluePose.getX()),
          FIELD_WIDTH.in(Units.Meters) - bluePose.getY(),
          bluePose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    private static Pose2d[] getRedAlliancePoses() {
      Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_POSES.length];

      for (int i = 0; i < POSES.BLUE_POSES.length; i++) {
        returnedPoses[i] = getRedAlliancePose(POSES.BLUE_POSES[i]);
      }
      return returnedPoses;
    }

    private static List<Pose2d> getRedReefPoses() {
      Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_REEF_POSES.size()];

      for (int i = 0; i < POSES.BLUE_REEF_POSES.size(); i++) {
        returnedPoses[i] = getRedAlliancePose(POSES.BLUE_REEF_POSES.get(i));
      }

      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3], returnedPoses[4],
          returnedPoses[5], returnedPoses[6], returnedPoses[7], returnedPoses[8], returnedPoses[9], returnedPoses[10],
          returnedPoses[11]);
    }

    /**
     * Gets the positions of all of the necessary field elements on the field. All
     * coordinates are in meters and are relative to the blue alliance.
     * 
     * @see <a href=
     *      https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
     *      Robot Coordinate Systems</a>
     * @return An array of field element positions
     */
    public static Supplier<Pose2d[]> getFieldPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> POSES.RED_POSES;

      }
      return () -> POSES.BLUE_POSES;
    }

    /**
     * Gets the positions of all of the necessary field elements on the field. All
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
  }

  public static class constVision {
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
    public static final Distance LL_FORWARD = Units.Meters.of(0.3302);
    public static final Distance LL_RIGHT = Units.Meters.of(-0.2921);
    public static final Distance LL_UP = Units.Meters.of(0.2286);

    public static final Angle LL_ROLL = Units.Degrees.of(0);
    public static final Angle LL_PITCH = Units.Degrees.of(15.92);
    public static final Angle LL_YAW = Units.Degrees.of(-20);

  }

  public static class constHopper {
    public static final double HOPPER_SPEED = 0.5;

    public static final TalonFXConfiguration HOPPER_CONFIG = new TalonFXConfiguration();

  }
}