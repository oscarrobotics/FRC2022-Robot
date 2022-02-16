package frc.team832.robot;

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

	public static final int RPD_CAN_ID = 1;
	public static final int RPH_CAN_ID = 1;

	public static final class DrivetrainConstants {
		/** CAN IDs **/ 
		public static final int LEFT_MASTER_TALON_ID = 0;
		public static final int LEFT_SLAVE_TALON_ID = 1;
		public static final int RIGHT_MASTER_TALON_ID = 2;
		public static final int RIGHT_SLAVE_TALON_ID = 3;
		public static final int PIGEON_ID = 0;

		/** Power **/ 
		public static final int CURRENT_LIMIT = 45;

		/** Mechanical Characteristics **/
		public static final Gearbox GEARBOX = new Gearbox(11.0 / 60.0, 16.0 / 32.0);
		public static final Motor MOTOR = Motor.kFalcon500;
		public static final double WHEEL_DIAMETER_INCHES = 5.9;
		public static final double WHEEBASE_INCHES = 26.0;
		public static final WheeledPowerTrain POWER_TRAIN = new WheeledPowerTrain(GEARBOX, MOTOR, 2, WHEEL_DIAMETER_INCHES);
	}	

	public static final class IntakeConstants {
		/** CAN IDs **/
		public static final int INTAKE_MOTOR_TALON_ID = 4;

		/** Power **/
		public static final int CURRENT_LIMIT = 45;

		/** Mechanical Characteristics **/
		public static final double INTAKE_SPEED = 0;
		public static final double OUTTAKE_SPEED = 0;
	}

	public static final class ConveyerConstants {
		/** CAN IDs **/
		public static final int CONVEYER_MOTOR_TALON_ID = 5;

		/** Power **/
		public static final int CURRENT_LIMIT = 45;

		/** Mechanical Characteristics **/
		public static final double QUEUING_SPEED = 0;
	}

	public static final class ShooterConstants {
		/** CAN IDs **/
		public static final int SHOOTER_MOTOR_TALON_ID = 6;

		/** Power **/
		public static final int CURRENT_LIMIT = 45;

		/** Mechanical Characteristics **/
		public static final double SHOOTER_SPEED = 0;
		public static final double QUEUING_SPEED = 0;
	}

	public static final class ClimbConstants {
			/** CAN IDs **/
			public static final int CLIMB_LEFT_TALON_ID = 7;
			public static final int CLIMB_RIGHT_TALON_ID = 8;

			/** Power **/
			public static final int CURRENT_LIMIT = 45;
	
			/** Mechanical Characteristics **/
			public static final int EXTEND_TARGET = 0;
			public static final int RETRACT_TARGET = 0;
		
	}

    public static class PneumaticsValues {
        public static final int PCM_MODULE_NUM = 0;

		/** Solenoid IDs*/
        public static final int INTAKE_SOLENOID_ID = 0;
		public static final int CLIMB_SOLENOID_ID = 1;
    }
}