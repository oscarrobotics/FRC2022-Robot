package frc.team832.robot.commands.AutonomousCommands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team832.robot.commands.AcceptBallAutoCmd;
import frc.team832.robot.commands.AcceptBallCommand;
import frc.team832.robot.commands.QueueBallCommand;
import frc.team832.robot.commands.ShootBallVisionCmd;
import frc.team832.robot.subsystems.ConveyorSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class TwoCargoAutoCmd extends SequentialCommandGroup{
    public final Trajectory initialPath;
    public TwoCargoAutoCmd(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ConveyorSubsystem conveyor, ShooterSubsystem shooter) {
        initialPath = drivetrain.loadPath("2 Ball Auto 01", 2, 3.5, true);
        addRequirements(drivetrain, intake, conveyor, shooter);
        addCommands(
            // moves dt backward to shoot
            // new InstantCommand(() -> drivetrain.setWheelPower(.5, .5)),
            // new WaitUntilCommand(() -> drivetrain.getLeftMeters() <= -1),
            // new InstantCommand(() -> drivetrain.setWheelPower(0.0, 0.0)),

            //spins shooter and conveyor to contain ball
            // new ShootBallCommand(conveyor, shooter),

            // starts intaking and drives forwards at the same time
            new ParallelRaceGroup(
                // intake ball
                new AcceptBallAutoCmd(intake, shooter, conveyor),
                // go backwards
                new SequentialCommandGroup(
                    new InstantCommand(() -> drivetrain.setWheelPower(-.2, -.2)),
                    // new WaitUntilCommand(() -> drivetrain.getLeftMeters() >= 1.5),
                    new WaitCommand(1.75),
                    new InstantCommand(() -> drivetrain.setWheelPower(0.0, 0.0))
                    // drivetrain.getTargetingCommand(() -> -.2)
                )
                // drivetrain.getTrajectoryCommand(initialPath)
            ),
            new QueueBallCommand(conveyor, shooter),
            new ShootBallVisionCmd(conveyor, shooter, false),
            new ShootBallVisionCmd(conveyor, shooter, false)

             // moves dt backward to shoot
            //  new InstantCommand(() -> drivetrain.setWheelPower(.5, .5)),
            //  new WaitUntilCommand(() -> drivetrain.getLeftMeters() <= -1),
            //  new InstantCommand(() -> drivetrain.setWheelPower(0.0, 0.0)),
 
             //spins shooter and conveyor to contain ball
            //  new ShootBallCommand(conveyor, shooter),

            // go forwards to clear tarmac
            // new InstantCommand(() -> drivetrain.setWheelPower(.5, .5)),
            // new WaitUntilCommand(() -> drivetrain.getLeftMeters() >= 2),
            // new InstantCommand(() -> drivetrain.setWheelPower(0.0, 0.0))
        );
    }
}
