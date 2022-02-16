package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;
import frc.team832.robot.Constants.ConveyerConstants;
import frc.team832.robot.Constants.ShooterConstants;

public class QueueBallCommand extends SequentialCommandGroup {


    public QueueBallCommand(ConveyerSubsystem conveyer, ShooterSubsystem shooter) {
        addRequirements(conveyer, shooter);
        addCommands(
            new InstantCommand(() -> shooter.setPower(ShooterConstants.QUEUING_SPEED)),
            new InstantCommand(() -> conveyer.setPower(ConveyerConstants.QUEUING_SPEED)),
            new WaitCommand(5),
            new InstantCommand (() -> conveyer.setPower(0)),
            new InstantCommand (() -> shooter.setPower(0))
        );
    }
}