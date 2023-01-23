package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team832.robot.subsystems.ConveyorSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class AcceptBallTimedCmd extends SequentialCommandGroup {
   public AcceptBallTimedCmd(IntakeSubsystem intake, ShooterSubsystem shooter, ConveyorSubsystem conveyor, double seconds) {
        addRequirements(intake, conveyor, shooter);

        addCommands(
            new ParallelRaceGroup(
                new AcceptBallCommand(intake, shooter, conveyor),
                new WaitCommand(seconds)
            )
        );
    }
}
