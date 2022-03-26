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
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	private Constants() {}

	public static final double LB_IN2_TO_KG_M2 = 0.000292639653;

	public static final int RPD_CAN_ID = 1;
	public static final int RPH_CAN_ID = 1;

	public static final class RobotConstants {
		public static final double RobotBumperedSizeMeters = Units.inchesToMeters(32.25);
		public static final Translation2d RobotBumperEdgeFromCenter = new Translation2d(RobotBumperedSizeMeters / 2, RobotBumperedSizeMeters / 2);
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


		public static final Pose2d RightOuterTarmacCorner = new Pose2d(RightOuterTarmacCornerTrx, Rotation2d.fromDegrees(90));
		public static final Pose2d RightInnerTarmacCorner = new Pose2d(RightInnerTarmacCornerTrx, Rotation2d.fromDegrees(45));
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
		public static final Gearbox GEARBOX = new Gearbox(11.0 / 60.0, 18.0 / 30.0);
		public static final Motor MOTOR = Motor.kFalcon500;
		public static final double WHEEL_DIAMETER_INCHES = 6.25;
		public static final double WHEELBASE_INCHES = 26.0;
		public static final double WHEELBASE_METERS = Units.inchesToMeters(WHEELBASE_INCHES);
		public static final WheeledPowerTrain POWER_TRAIN = new WheeledPowerTrain(GEARBOX, MOTOR, 2, WHEEL_DIAMETER_INCHES, 1/GEARBOX.totalReduction);
		public static final double MASS_KG = Units.lbsToKilograms(118.9);
		public static final double MOI_KGM2 = 5.120993184;
		// WHEEL CIRCUMFERENCE IN METERES = .4985

		/** System Control Values **/
		private static final double KS_KA_ADJUSTMENT_OLD = 10.90909;
		private static final double KS_KA_ADJUSTMENT_NEW = 9.09090909;
		public static final double LEFT_KS = 0.6953;
		public static final double LEFT_KV = (2.2981 / KS_KA_ADJUSTMENT_OLD) * KS_KA_ADJUSTMENT_NEW;
		public static final double LEFT_KA = (0.54892 / KS_KA_ADJUSTMENT_OLD) * KS_KA_ADJUSTMENT_NEW;
		public static final SimpleMotorFeedforward LEFT_FEEDFORWARD = new SimpleMotorFeedforward(LEFT_KS, LEFT_KV, LEFT_KA);
		public static final double LEFT_KP = 3.4267;

		public static final double RIGHT_KS = 0.69347;
		public static final double RIGHT_KV = (2.3149 / KS_KA_ADJUSTMENT_OLD) * KS_KA_ADJUSTMENT_NEW;
		public static final double RIGHT_KA = (0.23937 / KS_KA_ADJUSTMENT_OLD) * KS_KA_ADJUSTMENT_NEW;
		public static final SimpleMotorFeedforward RIGHT_FEEDFORWARD = new SimpleMotorFeedforward(RIGHT_KS, RIGHT_KV, RIGHT_KA);
		public static final double RIGHT_KP = 2.9336;

		public static final double ANGULAR_KS = 0.49291;
		public static final double ANGULAR_KV = 0.13429;
		public static final double ANGULAR_KA = 0.0030209;
		public static final double ANGULAR_KP = 0.064151;

		// TEST PATH FOLLOWING TRAJECTORY
		private static final Pose2d zero_zero_StartPose = new Pose2d();
		private static final Pose2d threeMeterX_Pose = new Pose2d(3, 0, new Rotation2d());
		public static final TrajectoryConfig CALM_TRAJCONFIG = new TrajectoryConfig(1.2, 1);
		public static final TrajectoryConfig AGGRESSIVE_TRAJCONFIG = new TrajectoryConfig(4, 6);
		public static Trajectory test3MeterForwardTraj = PathHelper.generatePath(zero_zero_StartPose, threeMeterX_Pose, CALM_TRAJCONFIG);

		// PATHS
		// public static final Path THREE_BALL_AUTO_PATH = Filesystem.getDeployDirectory().toPath().resolve("/deploy/pathplanner/generatedJSON/3BallAuto.wpilib.json");
	}	

	public static final class IntakeConstants {
		/** CAN IDs **/
		public static final int INTAKE_MOTOR_TALON_ID = 5;

		/** Power **/
		public static final int CURRENT_LIMIT = 25;

		/** Mechanical Characteristics **/
		public static final double INTAKE_POWER = 0.7;
		public static final double OUTTAKE_POWER = -0.4;
		public static final double INTAKE_REDUCTION = 0.0 / 0.0; 

		/** System Control Values **/
		public static final double KP = 0.0;
		public static final double KS = 0.0;
		public static final double KV = Motor.kFalcon500.KvRPMPerVolt;
		public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(KS, KV);
	}

	public static final class ConveyerConstants {
		/** CAN IDs **/
		public static final int CONVEYER_MOTOR_TALON_ID = 6;

		/** Power **/
		public static final int CURRENT_LIMIT = 45;

		/** Mechanical Characteristics **/
		public static final double QUEUING_POWER = .3;
		public static final double FEEDING_POWER = .3;
		public static final double OUTTAKE_POWER = -0.3;
		public static final double CONVEYER_REDUCTION = 0.0 / 0.0; 

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
		public static final WheeledPowerTrain POWER_TRAIN = new WheeledPowerTrain(new Gearbox(SHOOTER_REDUCTION), Motor.kFalcon500, 1, 3.9, 1);
		public static final double MOI_KGM2 = COLSON_4x2IN_MOI_KG_M2 * 2;
		public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0, 1 / Motor.kFalcon500.KvRPMPerVolt);

		/** Speeds **/
		// public static final double FRONT_RPM_FENDER = 3165;
		// public static final double REAR_RPM_FENDER = 648;

		public static final double FRONT_RPM_FENDER = 2946;
		public static final double REAR_RPM_FENDER = 971.5;

		public static final double FRONT_RPM_TARMAC = 1943.9;
		public static final double REAR_RPM_TARMAC = 2766.3;

		public static final InterpolatingTreeMap<Double, Double> BOTTOM_SHOOTER_RPM_MAP = new InterpolatingTreeMap<>();
		static {
			BOTTOM_SHOOTER_RPM_MAP.put(0.0, FRONT_RPM_FENDER);
			BOTTOM_SHOOTER_RPM_MAP.put(24.75, 2118.36);
			BOTTOM_SHOOTER_RPM_MAP.put(40.75, 2043.59);
			BOTTOM_SHOOTER_RPM_MAP.put(53.5, 1570.08);
			BOTTOM_SHOOTER_RPM_MAP.put(76.0, 1520.23);
		}

		public static final InterpolatingTreeMap<Double, Double> TOP_SHOOTER_RPM_MAP = new InterpolatingTreeMap<>();
		static {
			TOP_SHOOTER_RPM_MAP.put(0.0, REAR_RPM_FENDER);
			TOP_SHOOTER_RPM_MAP.put(24.75, 1794.38);
			TOP_SHOOTER_RPM_MAP.put(40.75, 2018.67);
			TOP_SHOOTER_RPM_MAP.put(53.5, 2556.95);
			TOP_SHOOTER_RPM_MAP.put(76.0, 3315.59);
		}



		/** System Control Values **/ // Data from sysid
		public static final double BOTTOM_KS = 0.52828;
		public static final double BOTTOM_KV = 0.10848;
		public static final double BOTTOM_KA = 0.0067908;
		// public static final SimpleMotorFeedforward BOTTOM_FEEDFORWARD = new SimpleMotorFeedforward(BOTTOM_KS, BOTTOM_KV / 60.0);
		public static final SimpleMotorFeedforward BOTTOM_FEEDFORWARD = FEEDFORWARD;
		public static final double BOTTOM_KP = 0.11439;

		public static final double TOP_KS = 0.6101;
		public static final double TOP_KV = 0.10885;
		public static final double TOP_KA = 0.0069025;
		// public static final SimpleMotorFeedforward TOP_FEEDFORWARD = new SimpleMotorFeedforward(TOP_KS, TOP_KV / 60.0);
		public static final SimpleMotorFeedforward TOP_FEEDFORWARD = BOTTOM_FEEDFORWARD;
		public static final double TOP_KP = 0.11589 ;
	}

	public static final class ClimbConstants {
		/** CAN IDs **/
		public static final int CLIMB_LEFT_TALON_ID = 8;
		public static final int CLIMB_RIGHT_TALON_ID = 9;

		/** Power **/
		public static final int CURRENT_LIMIT = 45;

		/** Mechanical Characteristics **/
		public static final int EXTEND_TARGET = 0;
		public static final int RETRACT_TARGET = 0;
		public static final int MAX_EXTEND_POS = 0;
		public static final int MIN_EXTEND_POS = 0;

		/** System Control Values **/
		public static final double KP = 0.0;
		public static final double KD = 0.0;
		public static final double KS = 0.0;
		public static final double KV = Motor.kFalcon500.KvRPMPerVolt;
		public static final double KG = 0.0;
		public static final ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(KS, KG, KV);
	}

    public static class PneumaticsValues {
		/** Solenoid IDs*/
		public static final int INTAKE_SOLENOID_ID = 0;
		public static final int RIGHT_CLIMB_SOLENOID_ID = 1;
		public static final int LEFT_CLIMB_SOLENOID_ID = 3;
    }

	public static class VisionConstants {
		public static final double CAMERA_HEIGHT_METERS = .720344; // 28.36 in
		public static final double CAMERA_PITCH_RADIANS = OscarMath.degreesToRadians(45); // 45 degrees
		public static final double TARGET_HEIGHT_METERS = 2.58; // to bottom of target
	}
}
