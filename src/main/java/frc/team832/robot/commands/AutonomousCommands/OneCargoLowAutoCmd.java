package frc.team832.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.robot.commands.ShootBallCmd;
import frc.team832.robot.subsystems.ConveyorSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class OneCargoLowAutoCmd extends SequentialCommandGroup{
	public OneCargoLowAutoCmd(
		DrivetrainSubsystem drivetrain, IntakeSubsystem intake, 
		ConveyorSubsystem conveyor, ShooterSubsystem shooter) {
			addRequirements(drivetrain, intake, conveyor, shooter);
			addCommands(
				// new ShootBallCmd(conveyor, shooter, () -> 1500, () -> 1200)
				
				// TODO: drive out of tarmac fully with a RamseteCommand
			);
		}
}
