package frc.team832.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team832.robot.Constants.ShooterConstants;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class ShootBallCmd extends SequentialCommandGroup {
    private final ConveyerSubsystem conveyor;
    private final ShooterSubsystem shooter;
    public ShootBallCmd(ConveyerSubsystem conveyer, ShooterSubsystem shooter, Double frontRPM, Double rearRPM) {
        addRequirements(conveyer, shooter);
        this.conveyor = conveyer;
        this.shooter = shooter;

        addCommands(
            //shooter spins flywheels to target rpms
            new InstantCommand(() -> shooter.setRPM(frontRPM, rearRPM), shooter),

            // checks to see if flywheels at target before feeding
            // new WaitUntilCommand(() -> shooter.atTarget(50)),
            new WaitCommand(1.0),
            
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
