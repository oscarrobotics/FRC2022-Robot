package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.Constants.ConveyerConstants;
import frc.team832.robot.Constants.IntakeConstants;
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
    
    //When the wrong ball is intaken, the intake and conveyer spin the opposite way to spit it out between them
    @Override
    public void initialize() {
        intake.extendIntake();
        intake.setPower(IntakeConstants.INTAKE_POWER);
        conveyer.setPower(ConveyerConstants.OUTTAKE_POWER);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.idleIntake();
        conveyer.idleConveyer();
    }
}
