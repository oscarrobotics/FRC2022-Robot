package frc.team832.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.Constants.ClimbConstants;
import frc.team832.robot.subsystems.ClimbSubsystem;

public class ZeroClimbCmd extends CommandBase {
    private final ClimbSubsystem climb;
    
    public ZeroClimbCmd(ClimbSubsystem climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        // climb.retractClimb(ClimbConstants.MIN_EXTEND_POS);
    }

    @Override
    public boolean isFinished() {
        return climb.isStalling();
    }

    @Override
    public void end(boolean interrupted) {
        climb.idle();
        climb.zeroClimb();
    }

}
