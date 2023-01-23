package frc.team832.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team832.robot.subsystems.DrivetrainSubsystem;

public class BasicAutoCmd extends SequentialCommandGroup{
    public BasicAutoCmd(DrivetrainSubsystem drivetrain) {
        addRequirements(drivetrain);
        addCommands(
            // commands to drive robot out of zone
            new InstantCommand(() -> drivetrain.setWheelPower(-.45, -.45)),
            new WaitCommand(1.5),
            new InstantCommand(() -> drivetrain.setWheelPower(0.0, 0.0))
        );
    } 
}
