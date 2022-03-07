package frc.team832.robot.subsystems;

public class SuperStructure {
    private final IntakeSubsystem intake;
    private final ConveyerSubsystem conveyer;
    private final ShooterSubsystem shooter;

    public SuperStructure(IntakeSubsystem intake, ConveyerSubsystem conveyer, ShooterSubsystem shooter) {
        this.intake = intake;
        this.conveyer = conveyer;
        this.shooter = shooter;
    }
    
    public void idleAll() {
        shooter.idleShooter();
        conveyer.idleConveyer();
        intake.idleIntake();
    }
}
