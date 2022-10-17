package frc.team832.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.team832.robot.Constants.ClimbConstants.*;

import javax.swing.text.Position;

import frc.team832.robot.subsystems.ClimbSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;

    /* finish fully auto climb for GRITS */
public class AutoClimbCmd extends SequentialCommandGroup{
    private final ClimbSubsystem m_climb;
    public AutoClimbCmd(ClimbSubsystem climb, DrivetrainSubsystem drivetrain) {
        m_climb = climb;
        addRequirements(climb);
        addCommands(
            // retract to static arms
            // new PositionClimbCommand(m_climb, RETRACT_TARGET, RETRACT_TARGET),
            // free arms
            //new PositionClimbCommand(m_climb, LEFT_FREE_HOOK_TARGET, RIGHT_FREE_HOOK_TARGET), 
            // pivot arms
            //new PivotClimbCommand(m_climb),
            // to next bar
            new PositionClimbCommand(m_climb, LEFT_TO_NEXT_BAR_WAIT_POINT_TARGET, RIGHT_TO_NEXT_BAR_WAIT_POINT_TARGET),
            new WaitUntilCommand(() -> 
                (drivetrain.getPitch() >= SAFE_TO_EXTEND) && 
                (true

                )
            ),
            new PositionClimbCommand(climb, LEFT_TO_NEXT_BAR_TARGET, RIGHT_TO_NEXT_BAR_TARGET),
            // straighten
            new StraightenClimbCommand(m_climb)
            // new WaitCommand(0.3),
            // // retract to wait point
            // new PositionClimbCommand(m_climb, WAIT_POINT_TARGET, WAIT_POINT_TARGET),
            // new WaitCommand(3),

            // new PositionClimbCommand(m_climb, RETRACT_TARGET, RETRACT_TARGET),
            // new PositionClimbCommand(m_climb, LEFT_FREE_HOOK_TARGET, RIGHT_FREE_HOOK_TARGET), 
            // new PivotClimbCommand(m_climb),

            // new PositionClimbCommand(m_climb, LEFT_TO_NEXT_BAR_WAIT_POINT_TARGET, RIGHT_TO_NEXT_BAR_WAIT_POINT_TARGET),
            // new WaitUntilCommand(() -> drivetrain.getPitch() >= SAFE_TO_EXTEND),
            // new PositionClimbCommand(climb, LEFT_TO_NEXT_BAR_TARGET, RIGHT_TO_NEXT_BAR_TARGET),
            // // straighten
            // new StraightenClimbCommand(m_climb)

        );
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            m_climb.idle(true);
        }
        else {m_climb.idle(false)}
    }
}
