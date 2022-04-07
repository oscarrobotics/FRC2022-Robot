package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team832.robot.subsystems.ConveyorSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class ShootBallVisionCmd extends SequentialCommandGroup {
    private final ConveyorSubsystem conveyor;
    private final ShooterSubsystem shooter;

    public ShootBallVisionCmd(ConveyorSubsystem conveyor, ShooterSubsystem shooter, boolean isLow) {
        addRequirements(conveyor, shooter);
        this.conveyor = conveyor;
        this.shooter = shooter;

        addCommands(
            new ParallelRaceGroup(
                new RunCommand(() -> shooter.setVisionRpms(isLow), shooter),
                new SequentialCommandGroup(

                    // shooter spins flywheels to rpms based on distance
                    new WaitCommand(.2),

                    // checks to see if flywheels at target before feeding

                    new WaitUntilCommand(() -> shooter.atTarget()),
                    
                    // feeds 1 ball - starts conveyor, waits until current spike from shooting ball, then stops conveyor
                    new FeedBallCommand(conveyor, shooter)
                )
            )            
        );
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.idle();
        shooter.idle();
    }
}
