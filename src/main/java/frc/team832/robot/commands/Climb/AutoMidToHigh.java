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

public class AutoMidToHigh extends SequentialCommandGroup{
    private final ClimbSubsystem m_climb;
    public AutoMidToHigh(ClimbSubsystem climb, DrivetrainSubsystem drivetrain) {
        m_climb = climb;
        addRequirements(climb);
        addCommands(

            // retract to static arms
            new PositionClimbCommand(m_climb, RETRACT_TARGET, RETRACT_TARGET),
            // free arms
            new PositionClimbCommand(m_climb, LEFT_FREE_HOOK_TARGET, RIGHT_FREE_HOOK_TARGET), 
            // new WaitCommand(0),
            // pivot arms
            new PivotClimbCommand(m_climb),
            // new WaitCommand(0),
            // to next bar
            new PositionClimbCommand(m_climb, LEFT_TO_NEXT_BAR_TARGET, RIGHT_TO_NEXT_BAR_TARGET),
            // new WaitUntilCommand(() -> drivetrain.getPitch() >= SAFE_TO_EXTEND),
            // straighten
            new StraightenClimbCommand(m_climb),
            new WaitCommand(0.3),
            // retract to wait point
            new PositionClimbCommand(m_climb, WAIT_POINT_TARGET, WAIT_POINT_TARGET),
            new WaitCommand(1),
            // retract all the way
            new PositionClimbCommand(m_climb, RETRACT_TARGET, RETRACT_TARGET), 
            new PositionClimbCommand(m_climb, LEFT_FREE_HOOK_TARGET, RIGHT_FREE_HOOK_TARGET), 
            // new WaitCommand(0),
            // pivot arms
            new PivotClimbCommand(m_climb)
            // new WaitCommand(0),
            // to next bar
            // new PositionClimbCommand(m_climb, LEFT_TO_NEXT_BAR_WAIT_POINT_TARGET, RIGHT_TO_NEXT_BAR_WAIT_POINT_TARGET)
        );
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            m_climb.idle(true);
        }
    }
}
