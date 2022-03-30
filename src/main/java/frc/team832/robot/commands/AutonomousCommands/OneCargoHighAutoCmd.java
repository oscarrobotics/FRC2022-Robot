package frc.team832.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team832.robot.Constants.ShooterConstants;
import frc.team832.robot.commands.ShootBallCmd;
import frc.team832.robot.commands.ShootBallVisionCmd;
import frc.team832.robot.subsystems.ConveyorSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class OneCargoHighAutoCmd extends SequentialCommandGroup {
    public OneCargoHighAutoCmd(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ConveyorSubsystem conveyor, ShooterSubsystem shooter) {
        addRequirements(drivetrain, intake, conveyor, shooter);
        addCommands(
            //spins shooter and conveyor to contain ball
            // TODO: new ShootBallHigh command, with fender distance as parameter
            // new ShootBallCmd(conveyor, shooter, ShooterConstants.FRONT_RPM_FENDER, ShooterConstants.REAR_RPM_FENDER),

            //moves drivetrain forward to clear tarmac
            // TODO: replace with a path.
            new InstantCommand(() -> drivetrain.setWheelPower(-.45, -.45)),
            new WaitCommand(1.75),
            new InstantCommand(() -> drivetrain.setWheelPower(0.0, 0.0))
        );
    }
}
