package frc.team832.robot.commands.Climb;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.ClimbSubsystem;

public class ExtendClimbCommand extends CommandBase{
    private final ClimbSubsystem climb;
    
    public ExtendClimbCommand(ClimbSubsystem climb) {
       this.climb = climb;
        addRequirements(climb);
    }

   //Climb system extends arm outwards
    @Override
    public void initialize() {
        climb.extendClimb();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }

    
}
