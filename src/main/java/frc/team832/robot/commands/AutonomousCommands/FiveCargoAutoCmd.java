package frc.team832.robot.commands.AutonomousCommands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team832.robot.Constants.ShooterConstants;
import frc.team832.robot.commands.AcceptBallCommand;
import frc.team832.robot.commands.QueueBallCommand;
import frc.team832.robot.commands.ShootBallCmd;
import frc.team832.robot.commands.ShootBallVisionCmd;
import frc.team832.robot.subsystems.ConveyorSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class FiveCargoAutoCmd extends SequentialCommandGroup {
	public final Trajectory initialPath;
	public final Trajectory secondPath;
	public final Trajectory thirdPath;

	/**
	 * Five cargo autonomous.
	 * 
	 * Steps: 
	 * 	1) Shoot preload
	 * 	2) ParallelRace intake with first path
	 * 	3) Shoot two
	 * 	4) ParallelRace intake with second path
	 * 	5) Shoot two
	 */
	public FiveCargoAutoCmd(
		DrivetrainSubsystem drivetrain,
		IntakeSubsystem intake,
		ConveyorSubsystem conveyor,
		ShooterSubsystem shooter
	) {
		initialPath = drivetrain.loadPath("5 Ball Auto 01", 2, 2.5, true);
		secondPath = drivetrain.loadPath("5 Ball Auto 02", 2, 2.5, true);
		thirdPath = drivetrain.loadPath("5 Ball Auto 03", 2, 2.5);

		addRequirements(drivetrain, intake, conveyor, shooter);

		addCommands(			
			// new ShootBallVisionCmd(conveyor, shooter, false);
			new ShootBallCmd(conveyor, shooter, ShooterConstants.FRONT_RPM_LOW_FENDER, ShooterConstants.REAR_RPM_LOW_FENDER, true),

			new ParallelRaceGroup(
				new AcceptBallCommand(intake, shooter, conveyor),
				drivetrain.getTrajectoryCommand(initialPath)
			),
			new QueueBallCommand(conveyor, shooter),

			new WaitCommand(.3),

			// drivetrain.getTargetingCommand(() -> 0),
			// new ShootBallVisionCmd(conveyor, shooter, false)
			new ShootBallCmd(conveyor, shooter, ShooterConstants.FRONT_RPM_LOW_FENDER, ShooterConstants.REAR_RPM_LOW_FENDER, true),
			new ParallelRaceGroup(
				new AcceptBallCommand(intake, shooter, conveyor),
				drivetrain.getTrajectoryCommand(secondPath)
			),
			new QueueBallCommand(conveyor, shooter),

			drivetrain.getTrajectoryCommand(thirdPath),
			
			new WaitCommand(.3),
			
			// drivetrain.getTargetingCommand(() -> 0),
			// new ShootBallVisionCmd(conveyor, shooter, false)
			new ShootBallCmd(conveyor, shooter, ShooterConstants.FRONT_RPM_LOW_FENDER, ShooterConstants.REAR_RPM_LOW_FENDER, true)
		);
	}
}
