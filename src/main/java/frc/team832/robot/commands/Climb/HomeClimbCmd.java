package frc.team832.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.ClimbSubsystem;

public class HomeClimbCmd extends CommandBase {
    private final ClimbSubsystem climb;
    
    public HomeClimbCmd(ClimbSubsystem climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setPower(-.25, -.25);
    }

    @Override
    public boolean isFinished() {
        return climb.isStalling();
    }

    @Override
    public void end(boolean interrupted) {
        climb.reset();
    }

}
