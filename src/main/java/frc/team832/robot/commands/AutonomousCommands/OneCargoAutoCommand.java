package frc.team832.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team832.robot.Constants;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class OneCargoAutoCommand extends SequentialCommandGroup {
    public OneCargoAutoCommand(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ConveyerSubsystem conveyer, ShooterSubsystem shooter) {
        addRequirements(drivetrain, intake, conveyer, shooter);
        addCommands(
            new InstantCommand(() -> drivetrain.setWheelVolts(.5, -.5)),
            new WaitCommand(0.75),
            new InstantCommand(() -> drivetrain.setWheelVolts(0.0, 0.0)),
            new InstantCommand(() -> shooter.setPower(Constants.ShooterConstants.SHOOTER_POWER)),
            new InstantCommand(() -> conveyer.setPower(Constants.ConveyerConstants.CONVEYER_FEEDING_POWER)),
            new WaitCommand(.75),
            new InstantCommand(() -> shooter.setPower(0)),
            new InstantCommand(() -> conveyer.setPower(0)),
            new InstantCommand(() -> drivetrain.setWheelVolts(-.5, .5)),
            new WaitCommand(1),
            new InstantCommand(() -> drivetrain.setWheelVolts(0.0, 0.0))
        );
    }
}
