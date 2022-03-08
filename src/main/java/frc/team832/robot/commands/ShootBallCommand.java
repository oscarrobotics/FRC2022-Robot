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
            //conveyer spins opposite way for x seconds to feed the ball into the shooter
            new InstantCommand (() -> conveyer.setPower(-ConveyerConstants.CONVEYER_QUEUING_POWER)),
            new WaitCommand(1),
            //shooter spins to shoot the ball out for x seconds
            new InstantCommand (() -> shooter.setPower(ShooterConstants.SHOOTER_POWER)),
            new WaitCommand (2),
            //conveyer returns to its correct spinning direction
            new InstantCommand(() -> conveyer.setPower(ConveyerConstants.CONVEYER_FEEDING_POWER)),
            new WaitCommand(2),

            //both end after the ball is shot out
            new InstantCommand (() -> conveyer.setPower(0)),
            new InstantCommand (() -> shooter.setPower(0))
        );
    }
}
