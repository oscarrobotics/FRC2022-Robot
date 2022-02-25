package frc.team832.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.robot.subsystems.ClimbSubsystem;

public class AutoClimbCommand extends SequentialCommandGroup{
    public AutoClimbCommand(ClimbSubsystem climb) {
        addRequirements(climb);
        addCommands(

        );
    }
}
