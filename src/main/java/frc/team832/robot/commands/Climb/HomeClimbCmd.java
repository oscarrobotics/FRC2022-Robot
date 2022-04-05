package frc.team832.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.subsystems.ClimbSubsystem;

public class HomeClimbCmd extends CommandBase {
    private final ClimbSubsystem climb;

    private boolean leftHomed, rightHomed;
    
    public HomeClimbCmd(ClimbSubsystem climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        leftHomed = false;
        rightHomed = false;
    }

    @Override
    public void execute() {
        if (climb.isLeftStalling()) {
            leftHomed = true;
            climb.zeroLeft();
        }

        if (climb.isRightStalling()) {
            rightHomed = true;
            climb.zeroRight();
        }

        var leftPower = leftHomed ? 0 : -.25;
        var rightPower = rightHomed ? 0 : -.25;
        climb.setPower(leftPower, rightPower);
    }

    @Override
    public boolean isFinished() {
        return leftHomed && rightHomed;
    }
}
