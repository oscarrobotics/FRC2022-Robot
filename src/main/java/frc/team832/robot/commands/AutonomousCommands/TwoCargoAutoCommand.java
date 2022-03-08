package frc.team832.robot.commands.AutonomousCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team832.robot.commands.AcceptBallCommand;
import frc.team832.robot.commands.ShootBallCommand;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class TwoCargoAutoCommand extends SequentialCommandGroup{
    public TwoCargoAutoCommand(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ConveyerSubsystem conveyer, ShooterSubsystem shooter) {
        addRequirements(drivetrain, intake, conveyer, shooter);
        addCommands(
            // starts intaking and drives backwards at the same time
            new ParallelRaceGroup(
                // intake ball
                new AcceptBallCommand(intake, shooter, conveyer),
                // go backwards
                new SequentialCommandGroup(
                    new InstantCommand(() -> drivetrain.setWheelVolts(-.2, .2)),
                    new WaitUntilCommand(drivetrain::isAtBall),
                    new InstantCommand(() -> drivetrain.setWheelVolts(0.0, 0.0))
                )
            ),

            // go forwards then stop
            new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.setWheelVolts(.2, -.2)),
                new WaitUntilCommand(drivetrain::isAtGoal),
                new InstantCommand(() -> drivetrain.setWheelVolts(0.0, 0.0))
            ), 
            //shoots balls into the goal
            new ShootBallCommand(conveyer, shooter)
        );
    }
}
