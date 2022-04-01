package frc.team832.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team832.robot.Constants.ShooterConstants;
import frc.team832.robot.subsystems.ConveyorSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class ShootBallCmd extends SequentialCommandGroup {
    private final ConveyorSubsystem conveyor;
    private final ShooterSubsystem shooter;
    public ShootBallCmd(ConveyorSubsystem conveyor, ShooterSubsystem shooter, DoubleSupplier frontRPM, DoubleSupplier rearRPM, boolean low) {
        addRequirements(conveyor, shooter);
        this.conveyor = conveyor;
        this.shooter = shooter;
        double waitTime;

        if (low) {
            waitTime = 0;
        } else {
            waitTime = .75;
        }

        addCommands(
            //shooter spins flywheels to target rpms
            new InstantCommand(() -> shooter.setRPM(frontRPM.getAsDouble(), rearRPM.getAsDouble()), shooter),

            // checks to see if flywheels at target before feeding
            // new WaitUntilCommand(() -> shooter.atTarget(50)),
            new WaitCommand(waitTime),
            
            // feeds 1 ball - starts conveyor, waits until current spike from shooting ball, then stops conveyor
            new FeedBallCommand(conveyor, shooter)
        );
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.idle();
        shooter.idle();
    }
}