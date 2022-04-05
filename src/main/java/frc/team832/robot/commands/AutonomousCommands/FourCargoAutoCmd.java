package frc.team832.robot.commands.AutonomousCommands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.robot.commands.AcceptBallCommand;
import frc.team832.robot.commands.ShootBallVisionCmd;
import frc.team832.robot.subsystems.ConveyorSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class FourCargoAutoCmd extends SequentialCommandGroup {
	public final Trajectory initialPath;
	public final Trajectory secondPath;

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
	public FourCargoAutoCmd(
		DrivetrainSubsystem drivetrain,
		IntakeSubsystem intake,
		ConveyorSubsystem conveyor,
		ShooterSubsystem shooter
	) {
		initialPath = drivetrain.initializePaths("4 Ball Auto 01", 4, 6);
		secondPath = drivetrain.initializePaths("4 Ball Auto 02", 4, 6);
		addRequirements(drivetrain, intake, conveyor, shooter);
		addCommands(
			new ParallelRaceGroup(
				new AcceptBallCommand(intake, shooter, conveyor),
				drivetrain.getTrajectoryCommand(initialPath)
			),
			new ShootBallVisionCmd(conveyor, shooter),
			new ParallelRaceGroup(
				new AcceptBallCommand(intake, shooter, conveyor),
				drivetrain.getTrajectoryCommand(secondPath)
			),
			new ShootBallVisionCmd(conveyor, shooter)
		);
	}
	
}
