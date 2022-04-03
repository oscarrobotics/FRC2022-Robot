package frc.team832.robot.commands.Climb;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.ClimbSubsystem;
import frc.team832.robot.Constants.ClimbConstants;

public class ExtendClimbCommand extends CommandBase{
    private final ClimbSubsystem climb;
    private final double pow;
    
    public ExtendClimbCommand(ClimbSubsystem climb, double pow) {
        this.climb = climb;
        this.pow = pow;
        addRequirements(climb);
    }

   //Climb system extends arm outwards
    @Override
    public void initialize() {
        climb.setPower(pow, pow);
    }

    @Override
    public boolean isFinished() {
        return climb.getLeftPosition() >= ClimbConstants.LEFT_MAX_EXTEND_POS;
    }

    @Override
    public void end(boolean interrupted) {
        climb.idle();
    }
}
