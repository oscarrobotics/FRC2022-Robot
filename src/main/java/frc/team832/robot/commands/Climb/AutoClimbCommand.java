package frc.team832.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.robot.subsystems.ClimbSubsystem;

public class AutoClimbCommand extends SequentialCommandGroup{
    public AutoClimbCommand(ClimbSubsystem climb) {
        addRequirements(climb);
        addCommands(
            /*
            1. robot will attach to bar
            2. robot will angle itself so the next arm can extend
            3. robot will straighten itself so it's not swinging around so much
            4. other arm latches onto the next bar
            5. rinse and repeat
            */
            new RetractClimbCommand(climb),
            new PivotClimbCommand(climb),
            new ExtendClimbCommand(climb),
            new StraightenClimbCommand(climb),
            new RetractClimbCommand(climb),
            new PivotClimbCommand(climb),
            new ExtendClimbCommand(climb),
            new StraightenClimbCommand(climb),
            new RetractClimbCommand(climb)
        );
    }
}
