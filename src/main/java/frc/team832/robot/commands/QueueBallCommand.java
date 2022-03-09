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
            //spins shooter and conveyor in opposite directions to hold the ball in place
            new InstantCommand(() -> shooter.setPower(ShooterConstants.SHOOTER_QUEUING_POWER)),
            new InstantCommand(() -> conveyer.setPower(ConveyerConstants.QUEUING_POWER)),
            
            //queues ball for a given amount of seconds before the motors turn off
            new WaitCommand(1),
            new InstantCommand (() -> conveyer.idleConveyer()),
            new InstantCommand (() -> shooter.idleShooter())
        );
    }
}