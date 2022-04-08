package frc.team832.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import frc.team832.lib.motion.PathHelper;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.team832.lib.motors.Gearbox;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.motors.WheeledPowerTrain;
import frc.team832.lib.util.OscarMath;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	private Constants() {
	}

	public static final double LB_IN2_TO_KG_M2 = 0.000292639653;

	public static final int RPD_CAN_ID = 1;
	public static final int RPH_CAN_ID = 1;

	public static final class RobotConstants {
		public static final double RobotBumperedSizeMeters = Units.inchesToMeters(32.25);
		public static final Translation2d RobotBumperEdgeFromCenter = new Translation2d(RobotBumperedSizeMeters / 2,
				RobotBumperedSizeMeters / 2);
		public static final double RobotHalfBumperedWidth = RobotBumperedSizeMeters / 2;
	}

	public static final class FieldConstants {
		public static final Pose2d RightTarmacTrueCorner = new Pose2d(7.012, 1.664, Rotation2d.fromDegrees(67.5));
		public static final Pose2d LeftTarmacTrueCorner = new Pose2d(5.500, 5.075, Rotation2d.fromDegrees(-22.5));
		// public static final Translation2d OuterCornerTrx = new Translation2d()
		// .plus(RobotConstants.RobotBumperEdgeFromCenter)
		// .plus(new Translation2d(RobotConstants.RobotBumperedSizeMeters / 2, 0));

		// public static final Translation2d InnerCornerTrx = new Translation2d()
		// .minus(RobotConstants.RobotBumperEdgeFromCenter)
		// .plus(new Translation2d(RobotConstants.RobotBumperedSizeMeters / 2, 0));

		private static final Translation2d RightOuterTarmacCornerTrx = RightTarmacTrueCorner.getTranslation()
				.plus(new Translation2d(RobotConstants.RobotHalfBumperedWidth, RobotConstants.RobotHalfBumperedWidth));

		private static final Translation2d RightInnerTarmacCornerTrx = RightTarmacTrueCorner.getTranslation()
				.plus(new Translation2d(0, RobotConstants.RobotHalfBumperedWidth))
				.plus(new Translation2d(0.085, 0.125)); // magic fudge factor

		public static final Pose2d RightOuterTarmacCorner = new Pose2d(RightOuterTarmacCornerTrx,
				Rotation2d.fromDegrees(90));
		public static final Pose2d RightInnerTarmacCorner = new Pose2d(RightInnerTarmacCornerTrx,
				Rotation2d.fromDegrees(45));
		public static final Pose2d LeftInnerTarmacCorner = new Pose2d(5.930, 4.680, Rotation2d.fromDegrees(0));
		public static final Pose2d LeftOuterTarmacCorner = new Pose2d(6.083, 5.033, Rotation2d.fromDegrees(-45));
	}

	public static final class DrivetrainConstants {
		/** CAN IDs **/
		public static final int LEFT_MASTER_TALON_ID = 1;
		public static final int LEFT_SLAVE_TALON_ID = 2;
		public static final int RIGHT_MASTER_TALON_ID = 3;
		public static final int RIGHT_SLAVE_TALON_ID = 4;
		public static final int PIGEON_ID = 0;

		/** Power **/
		public static final int CURRENT_LIMIT = 55;

		/** Mechanical Characteristics **/
		public static final Gearbox GEARBOX = Gearbox.fromStages(11.0 / 60.0, 18.0 / 30.0);
		public static final Motor MOTOR = Motor.kFalcon500;
		public static final double WHEEL_DIAMETER_INCHES = 6.25;
		public static final double WHEELBASE_INCHES = 26.0;
		public static final double WHEELBASE_METERS = Units.inchesToMeters(WHEELBASE_INCHES);
		public static final double TRACKWIDTH_METERS = 0.7254;
		public static final WheeledPowerTrain POWER_TRAIN = new WheeledPowerTrain(GEARBOX, MOTOR, 2,
				WHEEL_DIAMETER_INCHES, GEARBOX.totalReduction);
		public static final double MASS_KG = Units.lbsToKilograms(121.9);
		public static final double MOI_KGM2 = 5.120993184;
		// WHEEL CIRCUMFERENCE IN METERES = .4985

		/** System Control Values **/
		public static final double RIGHT_KS = 0.70953;
		public static final double RIGHT_KV = 1.9657;
		public static final double RIGHT_KA = 0.55988;
		public static final SimpleMotorFeedforward RIGHT_FEEDFORWARD = new SimpleMotorFeedforward(RIGHT_KS, RIGHT_KV, RIGHT_KA);
		public static final double RIGHT_KP = 0.83404;

		public static final double LEFT_KS = 0.70902;
		public static final double LEFT_KV = 1.946;
		public static final double LEFT_KA = 0.29475;
		public static final SimpleMotorFeedforward LEFT_FEEDFORWARD = new SimpleMotorFeedforward(LEFT_KS, LEFT_KV, LEFT_KA);
		public static final double LEFT_KP = 0.24586;

		public static final double ANGULAR_KS = 0.47629;
		public static final double ANGULAR_KV = 1.8086;
		public static final double ANGULAR_KA = 1.0154;
		public static final double ANGULAR_KP = 1.4084;

		// TEST PATH FOLLOWING TRAJECTORY
		private static final Pose2d zero_zero_StartPose = new Pose2d();
		private static final Pose2d halfMeterX_Pose = new Pose2d(3, 0, new Rotation2d());
		public static final TrajectoryConfig CALM_TRAJCONFIG = new TrajectoryConfig(2, 3);
		public static final TrajectoryConfig AGGRESSIVE_TRAJCONFIG = new TrajectoryConfig(4, 6);
		public static Trajectory testHalfMeterForwardTraj = PathHelper.generatePath(
			zero_zero_StartPose,
			halfMeterX_Pose,
			CALM_TRAJCONFIG
		);

		// PATHS
		// public static final Path THREE_BALL_AUTO_PATH =
		// Filesystem.getDeployDirectory().toPath().resolve("/deploy/pathplanner/generatedJSON/3BallAuto.wpilib.json");
	}

	public static final class IntakeConstants {
		/** CAN IDs **/
		public static final int INTAKE_MOTOR_TALON_ID = 5;

		/** Power **/
		public static final int CURRENT_LIMIT = 25;

		/** Mechanical Characteristics **/
		public static final double INTAKE_POWER = 0.7;
		public static final double OUTTAKE_POWER = -0.4;
		// public static final double INTAKE_POWER = 0.2;
		// public static final double OUTTAKE_POWER = -0.2;
		public static final double INTAKE_REDUCTION = 0.0 / 0.0;

		/** System Control Values **/
		public static final double KP = 0.0;
		public static final double KS = 0.0;
		public static final double KV = Motor.kFalcon500.KvRPMPerVolt;
		public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(KS, KV);
	}

	public static final class ConveyorConstants {
		/** CAN IDs **/
		public static final int CONVEYOR_MOTOR_TALON_ID = 6;

		/** Power **/
		public static final int CURRENT_LIMIT = 45;

		/** Mechanical Characteristics **/
		public static final double QUEUING_POWER = .3;
		public static final double FEEDING_POWER = .3;
		public static final double OUTTAKE_POWER = -0.3;
		public static final double CONVEYOR_REDUCTION = 0.0 / 0.0;

		/** System Control Values **/
		public static final double KP = 0.0;
		public static final double KS = 0.0;
		public static final double KV = Motor.kFalcon500.KvRPMPerVolt;
		public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(KS, KV);
	}

	public static final class ShooterConstants {
		/** CAN IDs **/
		public static final int FRONT_MOTOR_CAN_ID = 7;
		public static final int REAR_MOTOR_CAN_ID = 10;

		/** Power **/
		public static final int CURRENT_LIMIT = 45;

		public static final double SHOOTER_POWER = .4;
		public static final double SHOOTER_QUEUING_POWER = -.3;

		// MoIs

		/** Mechanical Characteristics **/ // Note: both flywheels are mechanically identical.
		private static final double COLSON_4x2IN_MOI_KG_M2 = 0.929 * LB_IN2_TO_KG_M2;
		public static final double SHOOTER_REDUCTION = 1;
		public static final WheeledPowerTrain POWER_TRAIN = new WheeledPowerTrain(
			Gearbox.fromTotalReduction(SHOOTER_REDUCTION),
			Motor.kFalcon500, 
			1, 3.9, 1);
		public static final double MOI_KGM2 = COLSON_4x2IN_MOI_KG_M2 * 2;

		// public static final double FRONT_KS = 0.3;
		// public static final double FRONT_KV = 1 / Motor.kFalcon500.KvRPMPerVolt;

		// public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0.3,
				// 1 / Motor.kFalcon500.KvRPMPerVolt);

		/** Speeds **/
		public static final double FRONT_RPM_HIGH_FENDER = 2915;
		public static final double REAR_RPM_HIGH_FENDER = 1845;

		public static final double FRONT_RPM_LOW_FENDER = 800;
		public static final double REAR_RPM_LOW_FENDER = 1350;

		public static final double FRONT_RPM_HIGH_TARMAC = 1970;
		public static final double REAR_RPM_HIGH_TARMAC = 3165;

		// high shot maps
		public static final InterpolatingTreeMap<Double, Double> FRONT_SHOOTER_RPM_HIGH_MAP = new InterpolatingTreeMap<>();
		static {
			FRONT_SHOOTER_RPM_HIGH_MAP.put(0.92, 2070.0);
			FRONT_SHOOTER_RPM_HIGH_MAP.put(1.8, 1970.0);
			FRONT_SHOOTER_RPM_HIGH_MAP.put(2.73, 2220.0);
			FRONT_SHOOTER_RPM_HIGH_MAP.put(3.53, 1845.0);
			FRONT_SHOOTER_RPM_HIGH_MAP.put(4.3, 1695.0);
		}
		public static final InterpolatingTreeMap<Double, Double> REAR_SHOOTER_RPM_HIGH_MAP = new InterpolatingTreeMap<>();
		static {
			REAR_SHOOTER_RPM_HIGH_MAP.put(0.92, 3065.0);
			REAR_SHOOTER_RPM_HIGH_MAP.put(1.8, 3165.0);
			REAR_SHOOTER_RPM_HIGH_MAP.put(2.73, 3365.0);
			REAR_SHOOTER_RPM_HIGH_MAP.put(3.53, 4645.0);
			REAR_SHOOTER_RPM_HIGH_MAP.put(4.3, 5980.0);
		}

		public static double HIGH_HOOD_UP_DISTANCE = 1.25;

		public static boolean shouldHoodExtendHigh(double visionDistance) {
			return visionDistance >= HIGH_HOOD_UP_DISTANCE;
		}

		// low shot maps
		public static final InterpolatingTreeMap<Double, Double> FRONT_SHOOTER_RPM_LOW_MAP = new InterpolatingTreeMap<>();
		static {
			FRONT_SHOOTER_RPM_LOW_MAP.put(0.83, 850.0);
			FRONT_SHOOTER_RPM_LOW_MAP.put(1.5, 850.0);
			FRONT_SHOOTER_RPM_LOW_MAP.put(2.1, 850.0);
		}
		public static final InterpolatingTreeMap<Double, Double> REAR_SHOOTER_RPM_LOW_MAP = new InterpolatingTreeMap<>();
		static {
			REAR_SHOOTER_RPM_LOW_MAP.put(0.83, 1870.0);
			REAR_SHOOTER_RPM_LOW_MAP.put(1.5, 2440.0);
			REAR_SHOOTER_RPM_LOW_MAP.put(2.1, 2940.0);
		}

		/** System Control Values **/ // Data from sysid
		public static final double BOTTOM_KS = 0.2;
		public static final double BOTTOM_KV = 0.10848;
		public static final double BOTTOM_KA = 0.0067908;
		public static final SimpleMotorFeedforward FRONT_FEEDFORWARD = new
		SimpleMotorFeedforward(BOTTOM_KS, BOTTOM_KV / 60);
		// public static final SimpleMotorFeedforward FRONT_FEEDFORWARD = FEEDFORWARD;
		public static final double FRONT_KP = 0.11439;

		public static final double TOP_KS = 0.20111;
		public static final double TOP_KV = 0.11354;
		public static final double TOP_KA = 0.0052708;
		public static final SimpleMotorFeedforward REAR_FEEDFORWARD = new
		SimpleMotorFeedforward(TOP_KS, TOP_KV / 60);
		// public static final SimpleMotorFeedforward REAR_FEEDFORWARD = FRONT_FEEDFORWARD;
		public static final double REAR_KP = 0.017345 / 60;
	}

	public static final class ClimbConstants {
		/** CAN IDs **/
		public static final int CLIMB_LEFT_TALON_ID = 8;
		public static final int CLIMB_RIGHT_TALON_ID = 9;

		/** Power **/
		public static final int CURRENT_LIMIT = 45;

		/** Mechanical Characteristics **/
		public static final double LEFT_TO_NEXT_BAR_TARGET = 89.9;
		public static final double RIGHT_TO_NEXT_BAR_TARGET = 94.5;
		public static final double LEFT_TO_NEXT_BAR_WAIT_POINT_TARGET = 86.00 - 10;
		public static final double RIGHT_TO_NEXT_BAR_WAIT_POINT_TARGET = 92.36 - 10;
		public static final double SAFE_TO_EXTEND = 30;
		public static final double LEFT_FREE_HOOK_TARGET = 30;
		public static final double RIGHT_FREE_HOOK_TARGET = 30;
		public static final double RETRACT_TARGET = .75;
		public static final double WAIT_POINT_TARGET = 22;
		public static final double LEFT_MAX_EXTEND_POS = 89.9 - 2; // to mid bar
		public static final double RIGHT_MAX_EXTEND_POS = 94.5 - 2; // to mid bar
		public static final double MIN_EXTEND_POS = 0;
		public static final double GEARBOX_REDUCTION = 10.61;

		/** System Control Values **/
		public static final double LEFT_KP = 0.0;
		public static final double LEFT_KD = 0.0;
		public static final double RIGHT_KP = 0.0;
		public static final double RIGHT_KD = 0.0;
		public static final double LEFT_KS = 0.5625;
		public static final double RIGHT_KS = 0.539;
		public static final double KV = 1 / Motor.kFalcon500.KvRPMPerVolt / 10.61; // 10.61 = gearbox ratio
		public static final double KG = 0.0;
		public static final ElevatorFeedforward LEFT_FEEDFORWARD = new ElevatorFeedforward(LEFT_KS, KG, KV);
		public static final ElevatorFeedforward RIGHT_FEEDFORWARD = new ElevatorFeedforward(RIGHT_KS, KG, KV);
		public static final double MAX_LEFT_ENCODER_VELOCITY = 6226;
		public static final double MAX_RIGHT_ENCODER_VELOCITY = 6259;
	}

	public static class PneumaticsValues {
		/** Solenoid IDs */
		public static final int INTAKE_SOLENOID_ID = 0;
		// public static final int RIGHT_CLIMB_SOLENOID_ID = 1;
		// public static final int LEFT_CLIMB_SOLENOID_ID = 3;
		public static final int CLIMB_SOLENOID_ID = 1;
		public static final int HOOD_SOLENOID_ID = 2;
	}

	public static class VisionConstants {
		public static final double CAMERA_HEIGHT_METERS = .771398; // 30.37 in
		public static final double CAMERA_PITCH_RADIANS = OscarMath.degreesToRadians(45); // 45 degrees
		public static final double TARGET_HEIGHT_METERS = 2.58; // to bottom of target
	}
}
