package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.Constants.ConveyorConstants;
import frc.team832.robot.Constants.IntakeConstants;
import frc.team832.robot.subsystems.ConveyorSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;

//Nicole wrote this :))) <3
public class RejectBallCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final ConveyorSubsystem conveyor;

    public RejectBallCommand(IntakeSubsystem intake, ConveyorSubsystem conveyor) {
        this.conveyor = conveyor;
        this.intake = intake;
        addRequirements(intake, conveyor);
    }
    
    //When the wrong ball is intaken, the intake and conveyor spin the opposite way to spit it out between them
    @Override
    public void initialize() {
        intake.extendIntake();
        intake.setPower(-IntakeConstants.INTAKE_POWER);
        conveyor.setPower(ConveyorConstants.OUTTAKE_POWER);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.idle();
        conveyor.idle();
    }
}
