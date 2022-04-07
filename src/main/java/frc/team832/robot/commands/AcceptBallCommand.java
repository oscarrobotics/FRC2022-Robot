package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team832.robot.Constants.ConveyorConstants;
import frc.team832.robot.Constants.IntakeConstants;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;
import frc.team832.robot.subsystems.ConveyorSubsystem;

public class AcceptBallCommand extends CommandBase{

    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final ConveyorSubsystem conveyor;

    private final boolean isCorrectColor = false;

    public AcceptBallCommand(IntakeSubsystem intake, ShooterSubsystem shooter, ConveyorSubsystem conveyor) {
        this.intake = intake;
        this.shooter = shooter;
        this.conveyor = conveyor;
        addRequirements(intake, conveyor, shooter);
    }
    
    @Override
    public void initialize() {
        intake.extendIntake();
        intake.setPower(IntakeConstants.INTAKE_POWER);
        shooter.setRPM(-1000, -1000);

        conveyor.setPower(ConveyorConstants.QUEUING_POWER);
    }

    @Override
    public boolean isFinished() {
        return conveyor.topIsCargo() && conveyor.bottomIsCargo();
    }

    @Override
    public void end(boolean interrupted) {
        intake.idle();
        shooter.idle();
        conveyor.idle();
    }
}
