package frc.team832.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.team832.robot.Constants.ClimbConstants.*;
import frc.team832.robot.subsystems.ClimbSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;

public class AutoToNextBarCmd extends SequentialCommandGroup{
    public AutoToNextBarCmd(ClimbSubsystem climb, DrivetrainSubsystem drivetrain) {
        addRequirements(climb);
        addCommands(
            /*
            1. extend arms to setpoint to free dynamic arms
            2. pivot dynamic arms
            3. extend dynamic arms to next bar
            4. straighten dynamic arms
            5. retract arms all the way
            */
            
            // free arms
            new PositionClimbCommand(climb, LEFT_FREE_HOOK_TARGET, RIGHT_FREE_HOOK_TARGET), 
            new WaitCommand(0),
            // pivot arms
            new PivotClimbCommand(climb),
            new WaitCommand(0),
            // to next bar
            new PositionClimbCommand(climb, LEFT_TO_NEXT_BAR_WAIT_POINT_TARGET, RIGHT_TO_NEXT_BAR_WAIT_POINT_TARGET),
            new WaitUntilCommand(() -> drivetrain.getPitch() >= SAFE_TO_EXTEND),
            // straighten
            new StraightenClimbCommand(climb),
            new WaitCommand(0.2),
            // retract to wait point
            new PositionClimbCommand(climb, WAIT_POINT_TARGET, WAIT_POINT_TARGET),
            new WaitCommand(1),
            // retract all the way
            new PositionClimbCommand(climb, RETRACT_TARGET, RETRACT_TARGET)
        );
    }
}
