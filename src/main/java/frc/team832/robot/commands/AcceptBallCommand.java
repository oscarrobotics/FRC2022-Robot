package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.Constants;
import frc.team832.robot.subsystems.IntakeSubsystem;

public class AcceptBallCommand extends CommandBase{
    private final IntakeSubsystem intake;
    
    public AcceptBallCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    
    @Override
    public void initialize() {
        intake.extendIntake();
        intake.setPower(Constants.IntakeConstants.INTAKE_POWER);
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }

    @Override
    public void end(boolean interrupted) {
        intake.retractIntake();
        intake.setPower(0);
    }
}
