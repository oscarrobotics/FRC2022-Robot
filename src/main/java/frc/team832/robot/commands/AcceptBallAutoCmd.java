package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team832.robot.Constants.ConveyorConstants;
import frc.team832.robot.Constants.IntakeConstants;
import frc.team832.robot.Constants.ShooterConstants;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;
import frc.team832.robot.subsystems.ConveyorSubsystem;

public class AcceptBallAutoCmd extends CommandBase{
  
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final ConveyorSubsystem conveyor;

    public AcceptBallAutoCmd(IntakeSubsystem intake, ShooterSubsystem shooter, ConveyorSubsystem conveyor) {
        this.intake = intake;
        this.shooter = shooter;
        this.conveyor = conveyor;
        addRequirements(intake, conveyor);
    }
    
    @Override
    public void initialize() {
        //extends intake outwards
        intake.extendIntake();
        new WaitCommand(.2);
        //spins intake to accept the ball into the machine
        intake.setPower(IntakeConstants.INTAKE_POWER);
        //spins internal conveyors to contain the ball
        // shooter.setBottomPower(ShooterConstants.SHOOTER_QUEUING_POWER);
        // conveyor.setPower(ConveyorConstants.QUEUING_POWER);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        //stops spinning
        intake.idle();
        shooter.idle();
        conveyor.idle();
    }
}
