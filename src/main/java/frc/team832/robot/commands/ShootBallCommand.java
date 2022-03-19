package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;
import frc.team832.robot.Constants.ConveyerConstants;
import frc.team832.robot.Constants.ShooterConstants;

public class ShootBallCommand extends SequentialCommandGroup {
    public ShootBallCommand(ConveyerSubsystem conveyer, ShooterSubsystem shooter) {
        addRequirements(conveyer, shooter);
        addCommands(
            //shooter spins to shoot the ball out for x seconds
            new InstantCommand (() -> shooter.setBottomPower(ShooterConstants.SHOOTER_POWER)), //.4
            new InstantCommand (() -> shooter.setTopPower(ShooterConstants.SHOOTER_POWER)), //.4
            new WaitCommand (.4),
            //conveyer returns to its correct spinning direction
            new InstantCommand(() -> conveyer.setPower(ConveyerConstants.FEEDING_POWER)),
            new WaitCommand(1.5),

            //both end after the ball is shot out
            new InstantCommand (() -> conveyer.idleConveyer()),
            new InstantCommand (() -> shooter.idleShooter())
        );
    }
}
