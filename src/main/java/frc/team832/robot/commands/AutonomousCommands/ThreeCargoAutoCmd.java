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

public class ThreeCargoAutoCmd extends SequentialCommandGroup {
    public final Trajectory initialPath;

    public ThreeCargoAutoCmd(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ConveyorSubsystem conveyor, ShooterSubsystem shooter) {
        initialPath = drivetrain.initializePaths("3 Ball Auto", 4, 4);
        addRequirements(drivetrain, intake, conveyor, shooter);
        addCommands(
            // shoot ball
            new ShootBallVisionCmd(conveyor, shooter),

            // follow path to intake 2 cargo
            new ParallelRaceGroup(
                // start intake - ends when path command ends 
                new AcceptBallCommand(intake, shooter, conveyor),
                
                // Follow path
                drivetrain.getTrajectoryCommand(initialPath)
            ),

            // shoot ball
            new ShootBallVisionCmd(conveyor, shooter)

            // follow path to back out of tarmac - might not be needed
        );
    }
}
