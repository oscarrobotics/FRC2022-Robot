package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.Constants;
import frc.team832.robot.subsystems.IntakeSubsystem;

//Nicole wrote this :))) <3
public class RejectBall extends CommandBase {
    private final IntakeSubsystem intake;
    
    public RejectBall(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.extendIntake();
        intake.setPower(Constants.IntakeConstants.OUTTAKE_SPEED);
    }

    @Override
    public boolean isFinished() {
        return true;
    }


    //ASK BANKS ABOUT @Override

    //@Override
    public void end() {
        intake.retractIntake();
        intake.setPower(0);
    }
}
