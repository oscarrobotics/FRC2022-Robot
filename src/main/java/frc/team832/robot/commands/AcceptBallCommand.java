package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.Constants.ConveyerConstants;
import frc.team832.robot.Constants.IntakeConstants;
import frc.team832.robot.Constants.ShooterConstants;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;
import frc.team832.robot.subsystems.ConveyerSubsystem;

public class AcceptBallCommand extends CommandBase{
  
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final ConveyerSubsystem conveyer;

    public AcceptBallCommand(IntakeSubsystem intake, ShooterSubsystem shooter, ConveyerSubsystem conveyer) {
        this.intake = intake;
        this.shooter = shooter;
        this.conveyer = conveyer;
        addRequirements(intake, shooter, conveyer);
    }
    
    @Override
    public void initialize() {
        intake.extendIntake();
        intake.setPower(IntakeConstants.INTAKE_POWER);
        shooter.setPower(ShooterConstants.SHOOTER_QUEUING_POWER);
        conveyer.setPower(ConveyerConstants.CONVEYER_QUEUING_POWER);
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }

    @Override
    public void end(boolean interrupted) {
        intake.retractIntake();
        intake.setPower(0);
        shooter.setPower(0);
        conveyer.setPower(0);
    }
}
