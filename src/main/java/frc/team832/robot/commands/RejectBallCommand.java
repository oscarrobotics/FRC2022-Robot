package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.Constants;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;

//Nicole wrote this :))) <3
public class RejectBallCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final ConveyerSubsystem conveyer;

    public RejectBallCommand(IntakeSubsystem intake, ConveyerSubsystem conveyer) {
        this.conveyer = conveyer;
        this.intake = intake;
        addRequirements(intake, conveyer);
    }
    
    @Override
    public void initialize() {
        intake.extendIntake();
        intake.setPower(Constants.IntakeConstants.OUTTAKE_SPEED);
        conveyer.setPower(Constants.ConveyerConstants.CONVEYER_OUTTAKE_POWER);
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
