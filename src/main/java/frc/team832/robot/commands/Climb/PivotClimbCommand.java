package frc.team832.robot.commands.Climb;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.ClimbSubsystem;

public class PivotClimbCommand extends CommandBase{
    private final ClimbSubsystem climb;
    
    public PivotClimbCommand(ClimbSubsystem climb) {
       this.climb = climb;
        addRequirements(climb);
    }
   
    //climb system angles itself properly as to reach the next bar
    @Override
    public void initialize() {
        climb.pivotClimb();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }

    
}
