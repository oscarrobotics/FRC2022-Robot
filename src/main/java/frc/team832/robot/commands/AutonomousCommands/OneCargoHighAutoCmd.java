package frc.team832.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team832.robot.commands.ShootBallCommand;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class OneCargoHighAutoCmd extends SequentialCommandGroup {
    public OneCargoHighAutoCmd(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ConveyerSubsystem conveyer, ShooterSubsystem shooter) {
        addRequirements(drivetrain, intake, conveyer, shooter);
        addCommands(
            //spins shooter and conveyer to contain ball
            // TODO: new ShootBallHigh command, with fender distance as parameter
            new ShootBallCommand(conveyer, shooter, () -> 2600, () -> 2600),

            //moves drivetrain forward to clear tarmac
            // TODO: replace with a path.
            new InstantCommand(() -> drivetrain.setWheelPower(-.45, -.45)),
            new WaitCommand(1.75),
            new InstantCommand(() -> drivetrain.setWheelPower(0.0, 0.0))
        );
    }
}
