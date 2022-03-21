package frc.team832.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team832.robot.commands.AcceptBallCommand;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class TwoCargoAutoCommand extends SequentialCommandGroup{
    public TwoCargoAutoCommand(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ConveyerSubsystem conveyer, ShooterSubsystem shooter) {
        addRequirements(drivetrain, intake, conveyer, shooter);
        addCommands(
            // moves dt backward to shoot
            new InstantCommand(() -> drivetrain.setWheelPower(.5, .5)),
            new WaitUntilCommand(() -> drivetrain.getLeftMeters() <= -1),
            new InstantCommand(() -> drivetrain.setWheelPower(0.0, 0.0)),

            //spins shooter and conveyer to contain ball
            // new ShootBallCommand(conveyer, shooter),

            // starts intaking and drives forwards at the same time
            new ParallelRaceGroup(
                // intake ball
                new AcceptBallCommand(intake, shooter, conveyer),
                // go backwards
                new SequentialCommandGroup(
                    new InstantCommand(() -> drivetrain.setWheelPower(.2, .2)),
                    new WaitUntilCommand(() -> drivetrain.getLeftMeters() >= 1.5),
                    new InstantCommand(() -> drivetrain.setWheelPower(0.0, 0.0))
                )
            ),

             // moves dt backward to shoot
             new InstantCommand(() -> drivetrain.setWheelPower(.5, .5)),
             new WaitUntilCommand(() -> drivetrain.getLeftMeters() <= -1),
             new InstantCommand(() -> drivetrain.setWheelPower(0.0, 0.0)),
 
             //spins shooter and conveyer to contain ball
            //  new ShootBallCommand(conveyer, shooter),

            // go forwards to clear tarmac
            new InstantCommand(() -> drivetrain.setWheelPower(.5, .5)),
            new WaitUntilCommand(() -> drivetrain.getLeftMeters() >= 2),
            new InstantCommand(() -> drivetrain.setWheelPower(0.0, 0.0))
        );
    }
}
