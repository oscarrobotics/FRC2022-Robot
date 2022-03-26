package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class ShootBallVisionCmd extends SequentialCommandGroup {
    private final ConveyerSubsystem conveyor;
    private final ShooterSubsystem shooter;
    public ShootBallVisionCmd(ConveyerSubsystem conveyer, ShooterSubsystem shooter) {
        addRequirements(conveyer, shooter);
        this.conveyor = conveyer;
        this.shooter = shooter;
        addCommands(
            // shooter spins flywheels to rpms based on distance
            new InstantCommand(() -> shooter.setVisionRpms(), shooter),

            // checks to see if flywheels at target before feeding
            new WaitCommand(.25),
            new WaitUntilCommand(() -> shooter.atTarget()),
            
            // feeds 1 ball - starts conveyer, waits until current spike from shooting ball, then stops conveyer
            new FeedBallCommand(conveyer, shooter)
        );
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.idleConveyer();
        shooter.idleShooter();
    }
}
