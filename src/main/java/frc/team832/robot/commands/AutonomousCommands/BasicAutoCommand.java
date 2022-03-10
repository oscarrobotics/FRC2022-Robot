package frc.team832.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team832.robot.subsystems.DrivetrainSubsystem;

public class BasicAutoCommand extends SequentialCommandGroup{
    public BasicAutoCommand(DrivetrainSubsystem drivetrain) {
        addRequirements(drivetrain);
        addCommands(
            // commands to back robot out of zone
            new InstantCommand(() -> drivetrain.setWheelPower(-.5, .5)),
            new WaitCommand(0.75),
            new InstantCommand(() -> drivetrain.setWheelPower(0.0, 0.0)))
        ;
    } 
}
