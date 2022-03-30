package frc.team832.robot.subsystems;

public class SuperStructure {
    private final IntakeSubsystem intake;
    private final ConveyorSubsystem conveyor;
    private final ShooterSubsystem shooter;

    public SuperStructure(IntakeSubsystem intake, ConveyorSubsystem conveyor, ShooterSubsystem shooter) {
        this.intake = intake;
        this.conveyor = conveyor;
        this.shooter = shooter;
    }
    
    public void idleAll() {
        shooter.idle();
        conveyor.idle();
        intake.idle();
    }
}
