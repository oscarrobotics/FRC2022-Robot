package frc.team832.robot.commands.Climb;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.ClimbSubsystem;

public class PositionClimbCommand extends CommandBase{
    private final ClimbSubsystem climb;
    private final double m_leftTarget, m_rightTarget;
    
    public PositionClimbCommand(ClimbSubsystem climb, double leftTarget, double rightTarget) {
        this.climb = climb;
        m_leftTarget = leftTarget;
        m_rightTarget = rightTarget;
        addRequirements(climb);
    }

   //Climb system extends arm outwards
    @Override
    public void initialize() {
        climb.setTargetPosition(m_leftTarget, m_rightTarget);
    }

    @Override
    public boolean isFinished() {
        boolean done = climb.atTarget();
        System.out.println("POSCLIMB FINISHED");
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            climb.idle();
        } else {
        }
    }
}
