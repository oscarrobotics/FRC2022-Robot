package frc.team832.robot.commands.AutonomousCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team832.robot.commands.AcceptBallCommand;
import frc.team832.robot.commands.ShootBallCommand;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class ThreeCargoAutoCommand extends SequentialCommandGroup{
    public ThreeCargoAutoCommand(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ConveyerSubsystem conveyer, ShooterSubsystem shooter) {
        addRequirements(drivetrain, intake, conveyer, shooter);
        addCommands(
            // 2 ball command
            new TwoCargoAutoCommand(drivetrain, intake, conveyer, shooter)

            // ADD CODE FOR 3rd CARGO
                // TURN ROBOT
                // REPEAT 2 CARGO COMMAND CODE WITH DIFFERENT POSE 2D
        );
    }
}
