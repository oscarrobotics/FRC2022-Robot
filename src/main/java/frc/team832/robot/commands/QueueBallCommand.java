package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team832.robot.subsystems.ConveyorSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;
import frc.team832.robot.Constants.ConveyorConstants;

public class QueueBallCommand extends SequentialCommandGroup {
    public QueueBallCommand(ConveyorSubsystem conveyor, ShooterSubsystem shooter) {
        addRequirements(conveyor, shooter);
        addCommands(
            //conveyor spins opposite way for x seconds to feed the ball into the shooter
            new InstantCommand(() -> conveyor.setPower(-ConveyorConstants.QUEUING_POWER)),

            // check if ball is in position via sensor
            new WaitCommand(.4),
            // new WaitUntilCommand(conveyor::isCargo),
            // new WaitCommand(.5),

            new InstantCommand(() -> conveyor.idle())

            //spins shooter and conveyor in opposite directions to hold the ball in place
            // new InstantCommand(() -> shooter.setPower(ShooterConstants.SHOOTER_QUEUING_POWER)),
            // new InstantCommand(() -> conveyor.setPower(ConveyorConstants.QUEUING_POWER)),
            
            // //queues ball for a given amount of seconds before the motors turn off
            // new WaitCommand(1),
            // new InstantCommand (() -> conveyor.idleConveyor()),
            // new InstantCommand (() -> shooter.idleShooter())
        );
    }
}