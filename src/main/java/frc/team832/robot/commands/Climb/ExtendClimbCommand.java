package frc.team832.robot.commands.Climb;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.ClimbSubsystem;

public class ExtendClimbCommand extends CommandBase{
    private final ClimbSubsystem climb;
    private final double target;
    
    public ExtendClimbCommand(ClimbSubsystem climb, double target) {
        this.climb = climb;
        this.target = target;
        addRequirements(climb);
    }

   //Climb system extends arm outwards
    @Override
    public void initialize() {
        climb.extendClimb(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }

    
}
