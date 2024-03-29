package frc.team832.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.ClimbSubsystem;

public class StraightenClimbCommand extends CommandBase{
    private final ClimbSubsystem climb;
    
    public StraightenClimbCommand(ClimbSubsystem climb) {
       this.climb = climb;
        addRequirements(climb);
    }
   
    //climb system straightens the robot to not tilt *too* much when reaching the next bar
    @Override
    public void initialize() {
        climb.straightenClimb();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }

    
}
