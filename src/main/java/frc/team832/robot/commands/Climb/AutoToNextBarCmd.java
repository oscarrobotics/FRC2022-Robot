package frc.team832.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.robot.Constants.ClimbConstants;
import frc.team832.robot.subsystems.ClimbSubsystem;

public class AutoToNextBarCmd extends SequentialCommandGroup{
    public AutoToNextBarCmd(ClimbSubsystem climb) {
        addRequirements(climb);
        addCommands(
            /*
            1. extend arms to setpoint to free dynamic arms
            2. pivot dynamic arms
            3. extend dynamic arms to next bar
            4. straighten dynamic arms
            5. retract arms all the way
            */
            
            new ExtendClimbCommand(climb, ClimbConstants.FREE_HOOK_TARGET), 
            new PivotClimbCommand(climb),
            // new ExtendClimbCommand(climb, ClimbConstants.TO_NEXT_BAR_TARGET),
            new StraightenClimbCommand(climb),
            new RetractClimbCommand(climb, ClimbConstants.RETRACT_TARGET)
        );
    }
}
