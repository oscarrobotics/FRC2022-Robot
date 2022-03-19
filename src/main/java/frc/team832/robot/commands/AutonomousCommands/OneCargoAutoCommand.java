package frc.team832.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team832.robot.Constants;
import frc.team832.robot.commands.ShootBallCommand;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class OneCargoAutoCommand extends SequentialCommandGroup {
    public OneCargoAutoCommand(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ConveyerSubsystem conveyer, ShooterSubsystem shooter) {
        addRequirements(drivetrain, intake, conveyer, shooter);
        addCommands(
            // moves dt backward to shoot
            // new InstantCommand(() -> drivetrain.setWheelPower(.5, .5)),
            // new WaitUntilCommand(() -> drivetrain.getLeftMeters() <= -1),
            // new InstantCommand(() -> drivetrain.setWheelPower(0.0, 0.0)),

            //spins shooter and conveyer to contain ball
            // new ShootBallCommand(conveyer, shooter),

            //moves drivetrain forward to clear tarmac
            new InstantCommand(() -> drivetrain.setWheelPower(-.45, -.45)),
            new WaitCommand(1.75),
            new InstantCommand(() -> drivetrain.setWheelPower(0.0, 0.0))
        );
    }
}
