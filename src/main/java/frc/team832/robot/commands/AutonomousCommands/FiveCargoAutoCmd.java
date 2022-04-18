package frc.team832.robot.commands.AutonomousCommands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team832.robot.Constants.ShooterConstants;
import frc.team832.robot.commands.AcceptBallTimedCmd;
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
		initialPath = drivetrain.loadPath("5 Ball Auto 01", 2, 3.5, true);
		secondPath = drivetrain.loadPath("5 Ball Auto 02", 3, 3.5, true);
		thirdPath = drivetrain.loadPath("5 Ball Auto 03", 3, 6);

		addRequirements(drivetrain, intake, conveyor, shooter);

		addCommands(
			// drivetrain.getTargetingCommand(() -> 0).withTimeout(0.3),
			new InstantCommand(() -> SmartDashboard.putString("Current Cmd: ", "ShootBallVisionCmd 1")),			
			new ShootBallVisionCmd(conveyor, shooter, false),
			// new ShootBallCmd(conveyor, shooter, ShooterConstants.FRONT_RPM_LOW_FENDER, ShooterConstants.REAR_RPM_LOW_FENDER, true),

			new InstantCommand(() -> SmartDashboard.putString("Current Cmd: ", "ParallelRaceGroup (Intake+Path) 1")),			
			new ParallelRaceGroup(
				new AcceptBallCommand(intake, shooter, conveyor),
				drivetrain.getTrajectoryCommand(initialPath)
			),
			new InstantCommand(() -> SmartDashboard.putString("Current Cmd: ", "QueueBallCmd 1")),			
			new QueueBallCommand(conveyor, shooter),

			drivetrain.getTargetingCommand(() -> 0).withTimeout(0.3),
			new InstantCommand(() -> SmartDashboard.putString("Current Cmd: ", "ShootBallVisionCmd 2")),			
			new ShootBallVisionCmd(conveyor, shooter, false),
			new InstantCommand(() -> SmartDashboard.putString("Current Cmd: ", "ShootBallVisionCmd 3")),			
			new ShootBallVisionCmd(conveyor, shooter, false),
			
			// new ShootBallCmd(conveyor, shooter, ShooterConstants.FRONT_RPM_LOW_FENDER, ShooterConstants.REAR_RPM_LOW_FENDER, true),
			new InstantCommand(() -> SmartDashboard.putString("Current Cmd: ", "ParallelRaceGroup (Intake+Path) 2")),			
			new ParallelRaceGroup(
				new AcceptBallCommand(intake, shooter, conveyor),
				new SequentialCommandGroup(
					drivetrain.getTrajectoryCommand(secondPath),
					new WaitCommand(1.25)
				)
			),

			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new AcceptBallTimedCmd(intake, shooter, conveyor, 1),
					new InstantCommand(() -> SmartDashboard.putString("Current Cmd: ", "QueueBallCmd 2")),			
					new QueueBallCommand(conveyor, shooter)
				),
				new InstantCommand(() -> SmartDashboard.putString("Current Cmd: ", "Path 3")),			
				drivetrain.getTrajectoryCommand(thirdPath)
			),

			drivetrain.getTargetingCommand(() -> 0).withTimeout(0.3),
			new InstantCommand(() -> SmartDashboard.putString("Current Cmd: ", "ShootBallVisionCmd 4")),			
			new ShootBallVisionCmd(conveyor, shooter, false),
			new InstantCommand(() -> SmartDashboard.putString("Current Cmd: ", "ShootBallVisionCmd 5")),			
			new ShootBallVisionCmd(conveyor, shooter, false)
			// new ShootBallCmd(conveyor, shooter, ShooterConstants.FRONT_RPM_LOW_FENDER, ShooterConstants.REAR_RPM_LOW_FENDER, true)
		);
	}
}
