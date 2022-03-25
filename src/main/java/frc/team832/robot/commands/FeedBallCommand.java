package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team832.robot.Constants.ConveyerConstants;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class FeedBallCommand extends SequentialCommandGroup{
    public FeedBallCommand(ConveyerSubsystem conveyer, ShooterSubsystem shooter) {
        addRequirements(conveyer);
        addCommands(
            // feeds only 1 ball

            // feeds 1 ball - starts conveyer, waits until current spike from shooting ball, then stops conveyer
            new InstantCommand(() -> conveyer.setPower(ConveyerConstants.FEEDING_POWER)),
            // new WaitUntilCommand(() -> shooter.isStalling()),
            new WaitCommand(.3),
            new InstantCommand(() -> conveyer.setPower(0)),
            new WaitCommand(.5)
        );
    }
}
