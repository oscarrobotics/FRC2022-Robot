package frc.team832.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.team832.lib.motors.Gearbox;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.motors.WheeledPowerTrain;

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

	public static final class DrivetrainConstants {
		/** CAN IDs **/ 
		public static final int LEFT_MASTER_TALON_ID = 1;
		public static final int LEFT_SLAVE_TALON_ID = 2;
		public static final int RIGHT_MASTER_TALON_ID = 3;
		public static final int RIGHT_SLAVE_TALON_ID = 4;
		public static final int PIGEON_ID = 0;

		/** Power **/ 
		public static final int CURRENT_LIMIT = 45;

		/** Mechanical Characteristics **/
		public static final Gearbox GEARBOX = new Gearbox(11.0 / 60.0, 16.0 / 32.0);
		public static final Motor MOTOR = Motor.kFalcon500;
		public static final double WHEEL_DIAMETER_INCHES = 5.9;
		public static final double WHEELBASE_INCHES = 26.0;
		public static final double WHEELBASE_METERS = Units.inchesToMeters(WHEELBASE_INCHES);
		public static final WheeledPowerTrain POWER_TRAIN = new WheeledPowerTrain(GEARBOX, MOTOR, 2, WHEEL_DIAMETER_INCHES, 1);

		public static final double MASS_KG = Units.lbsToKilograms(118.9);
		public static final double MOI_KGM2 = 5.120993184;

		/** System Control Values **/
		public static final double LEFT_KS = 0.1;
		public static final double LEFT_KV = (1 / Motor.kFalcon500.KvRPMPerVolt) * GEARBOX.totalReduction;
		public static final SimpleMotorFeedforward LEFT_FEEDFORWARD = new SimpleMotorFeedforward(LEFT_KS, LEFT_KV);
		public static final double LEFT_KP = 0.0;
		public static final SimpleMotorFeedforward LEFT_FEEDFORWARD = new SimpleMotorFeedforward(LEFT_KS, LEFT_KV);

		public static final double RIGHT_KS = 0.1;
		public static final double RIGHT_KV = (1 / Motor.kFalcon500.KvRPMPerVolt) * GEARBOX.totalReduction;
		public static final SimpleMotorFeedforward RIGHT_FEEDFORWARD = new SimpleMotorFeedforward(RIGHT_KS, RIGHT_KV);
		public static final double RIGHT_KP = 0.0;
		public static final SimpleMotorFeedforward RIGHT_FEEDFORWARD = new SimpleMotorFeedforward(RIGHT_KS, RIGHT_KV);
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
		public static final double KP = 0.05;
		public static final double SHOOTER_REDUCTION = 1; 
		public static final WheeledPowerTrain POWER_TRAIN = new WheeledPowerTrain(new Gearbox(SHOOTER_REDUCTION), Motor.kFalcon500, 1, 4, 1);
		public static final double MOI_KGM2 = COLSON_4x2IN_MOI_KG_M2 * 2;
		public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(0, 1 / Motor.kFalcon500.KvRPMPerVolt);
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
        public static final int PCM_MODULE_NUM = 0;

		/** Solenoid IDs*/
        public static final int INTAKE_SOLENOID_ID = 0;
		public static final int RIGHT_CLIMB_SOLENOID_ID = 1;
		public static final int LEFT_CLIMB_SOLENOID_ID = 3;
    }
}
