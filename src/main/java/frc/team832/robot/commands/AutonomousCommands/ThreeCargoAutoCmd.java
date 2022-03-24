package frc.team832.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.robot.Constants.DrivetrainConstants;
import frc.team832.robot.Constants.ShooterConstants;
import frc.team832.robot.commands.AcceptBallCommand;
import frc.team832.robot.commands.ShootBallManualCmd;
import frc.team832.robot.commands.ShootBallVisionCmd;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class ThreeCargoAutoCmd extends SequentialCommandGroup {
    public ThreeCargoAutoCmd(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ConveyerSubsystem conveyer, ShooterSubsystem shooter) {
        addRequirements(drivetrain, intake, conveyer, shooter);
        addCommands(
            // shoot ball
            new ShootBallVisionCmd(conveyer, shooter),

            // follow path to intake 2 cargo
            new ParallelRaceGroup(
                // start intake - ends when path command ends 
                new AcceptBallCommand(intake, shooter, conveyer),
                
                // Follow path
                drivetrain.getTrajectoryCommand(drivetrain.initializePaths(DrivetrainConstants.THREE_BALL_AUTO_PATH))
            ),

            // shoot ball
            new ShootBallVisionCmd(conveyer, shooter)

            // follow path to back out of tarmac - might not be needed
        );
    }
}
