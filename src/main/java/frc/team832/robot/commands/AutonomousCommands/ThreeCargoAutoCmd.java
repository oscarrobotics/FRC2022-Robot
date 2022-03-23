package frc.team832.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.robot.commands.AcceptBallCommand;
import frc.team832.robot.commands.ShootBallCommand;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class ThreeCargoAutoCmd extends SequentialCommandGroup {
    public ThreeCargoAutoCmd(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ConveyerSubsystem conveyer, ShooterSubsystem shooter) {
        addRequirements(drivetrain, intake, conveyer, shooter);
        addCommands(
            // shoot ball

            // follow path to intake 2 cargo
            new ParallelRaceGroup(
                new AcceptBallCommand(intake, shooter, conveyer) 
                // Follow path
            )

            // shoot ball

            // follow path to back out of tarmac

        );
    }
}
