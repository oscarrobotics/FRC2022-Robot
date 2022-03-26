package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team832.robot.Constants.ConveyerConstants;
import frc.team832.robot.Constants.IntakeConstants;
import frc.team832.robot.Constants.ShooterConstants;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;
import frc.team832.robot.subsystems.ConveyerSubsystem;

public class AcceptBallAutoCmd extends CommandBase{
  
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final ConveyerSubsystem conveyer;

    public AcceptBallAutoCmd(IntakeSubsystem intake, ShooterSubsystem shooter, ConveyerSubsystem conveyer) {
        this.intake = intake;
        this.shooter = shooter;
        this.conveyer = conveyer;
        addRequirements(intake, conveyer);
    }
    
    @Override
    public void initialize() {
        //extends intake outwards
        intake.extendIntake();
        //spins intake to accept the ball into the machine
        intake.setPower(IntakeConstants.INTAKE_POWER);
        //spins internal conveyors to contain the ball
        // shooter.setBottomPower(ShooterConstants.SHOOTER_QUEUING_POWER);
        // conveyer.setPower(ConveyerConstants.QUEUING_POWER);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        //stops spinning
        intake.idleIntake();
        shooter.idleShooter();
        conveyer.idleConveyer();
    }
}
