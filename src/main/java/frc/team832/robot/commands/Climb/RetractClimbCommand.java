package frc.team832.robot.commands.Climb;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.ClimbSubsystem;

public class RetractClimbCommand extends CommandBase{
    private final ClimbSubsystem climb;
    
    public RetractClimbCommand(ClimbSubsystem climb) {
       this.climb = climb;
        addRequirements(climb);
    }
   
    //climb system brings arm down to latch onto the next bar
    @Override
    public void initialize() {
        climb.retractClimb();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }

    
}
