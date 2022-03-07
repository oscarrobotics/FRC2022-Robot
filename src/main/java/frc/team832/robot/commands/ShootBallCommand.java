package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;
import frc.team832.robot.Constants.ConveyerConstants;
import frc.team832.robot.Constants.ShooterConstants;

public class ShootBallCommand extends SequentialCommandGroup {
    public ShootBallCommand(ConveyerSubsystem conveyer, ShooterSubsystem shooter) {
        addRequirements(conveyer, shooter);
        addCommands(
            new InstantCommand (() -> conveyer.setPower(-ConveyerConstants.CONVEYER_QUEUING_POWER)),
            new WaitCommand(1),
            new InstantCommand (() -> shooter.setPower(ShooterConstants.SHOOTER_POWER)),
            new WaitCommand (2),
            new InstantCommand(() -> conveyer.setPower(ConveyerConstants.CONVEYER_FEEDING_POWER)),
            new WaitCommand(2),

            new InstantCommand (() -> conveyer.setPower(0)),
            new InstantCommand (() -> shooter.setPower(0))
        );
    }
}
