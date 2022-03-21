package frc.team832.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class ShootBallCommand extends SequentialCommandGroup {
    private final ConveyerSubsystem conveyor;
    private final ShooterSubsystem shooter;
    public ShootBallCommand(ConveyerSubsystem conveyer, ShooterSubsystem shooter, DoubleSupplier frontRPM, DoubleSupplier rearRPM) {
        addRequirements(conveyer, shooter);
        this.conveyor = conveyer;
        this.shooter = shooter;
        addCommands(
            //shooter spins flywheels to target rpms
            new InstantCommand(() -> shooter.setRPM(frontRPM.getAsDouble(), rearRPM.getAsDouble()), shooter),

            // checks to see if flywheels at target before feeding
            new WaitUntilCommand(() -> shooter.atTarget()),

            // feeds 1 ball - starts conveyer, waits until current spike from shooting ball, then stops conveyer
            new FeedBallCommand(conveyer, shooter)

            // // checks to see if flywheels at target before feeding
            // new WaitUntilCommand(() -> shooter.atTarget()),

            // // feeds 2nd ball - starts conveyer, waits until current spike from shooting ball, then stops conveyer
            // new FeedBallCommand(conveyer, shooter)
        );
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.idleConveyer();
        shooter.idleShooter();
    }
}
