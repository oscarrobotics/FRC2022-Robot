package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team832.robot.Constants.ConveyorConstants;
import frc.team832.robot.subsystems.ConveyorSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class FeedBallCommand extends SequentialCommandGroup{
    public FeedBallCommand(ConveyorSubsystem conveyor, ShooterSubsystem shooter) {
        addRequirements(conveyor);
        addCommands(
            // feeds only 1 ball

            // feeds 1 ball - starts conveyor, waits until current spike from shooting ball, then stops conveyor
            new InstantCommand(() -> conveyor.setPower(ConveyorConstants.FEEDING_POWER)),
            // new WaitUntilCommand(() -> shooter.isStalling()),
            new WaitUntilCommand(conveyor::isCargo),
            new WaitUntilCommand(() -> !conveyor.isCargo()),
            new WaitCommand(.3),
            // new WaitCommand(.5),
            new InstantCommand(() -> conveyor.setPower(0)),
            // new WaitCommand(.5)
            new QueueBallCommand(conveyor, shooter)
        );
    }
}
