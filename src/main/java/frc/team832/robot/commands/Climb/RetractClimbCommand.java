package frc.team832.robot.commands.Climb;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.ClimbSubsystem;

public class RetractClimbCommand extends CommandBase{
    private final ClimbSubsystem climb;
    private final double target;

    public RetractClimbCommand(ClimbSubsystem climb, double target) {
        this.climb = climb;
        this.target = target;

        addRequirements(climb);
    }
   
    //climb system brings arm down to latch onto the next bar
    @Override
    public void initialize() {
        climb.retractClimb(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }

    
}
