package frc.team832.robot.subsystems;

import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motion.OscarFlywheel;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.lib.power.monitoring.StallDetector;
import frc.team832.lib.power.monitoring.StallDetector.StallDetectorStatus;

import static frc.team832.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    /** Physical objects */
    private final CANTalonFX m_frontMotor = new CANTalonFX(FRONT_MOTOR_CAN_ID);
    private final CANTalonFX m_rearMotor = new CANTalonFX(REAR_MOTOR_CAN_ID);

    private final OscarFlywheel m_frontFlywheel = new OscarFlywheel("ShooterSubsystem/Front Flywheel", m_frontMotor, POWER_TRAIN, BOTTOM_FEEDFORWARD, BOTTOM_KP, MOI_KGM2);
    private final OscarFlywheel m_rearFlywheel = new OscarFlywheel("ShooterSubsystem/Rear Flywheel", m_rearMotor, POWER_TRAIN, TOP_FEEDFORWARD, TOP_KP, MOI_KGM2);

    private final StallDetector stallDetector = new StallDetector(m_frontMotor::getOutputCurrent);

    /** Creates a new ShooterSubsystem **/
    public ShooterSubsystem() {
        DashboardManager.addTab(this);

        m_frontMotor.setNeutralMode(NeutralMode.kCoast);
        m_frontMotor.limitInputCurrent(CURRENT_LIMIT);
        m_frontMotor.setInverted(true);

        m_rearMotor.setNeutralMode(NeutralMode.kCoast);
        m_rearMotor.limitInputCurrent(CURRENT_LIMIT);

        m_frontFlywheel.setClosedLoop(false);
        m_rearFlywheel.setClosedLoop(false);

        // stallDetector.setStallCurrent(7);
    }

    @Override
    public void periodic() {
        m_frontFlywheel.periodic();
        m_rearFlywheel.periodic();
    }

    public void setRPM(double frontTarget, double rearTarget) {
        m_frontFlywheel.setTargetVelocityRpm(frontTarget);
        m_rearFlywheel.setTargetVelocityRpm(rearTarget);
    }

    public void idleShooter() {
        m_frontFlywheel.setTargetVelocityRpm(0);
        m_rearFlywheel.setTargetVelocityRpm(0);
    }

    public boolean atTarget() {
        return m_frontFlywheel.atTarget(50) && m_rearFlywheel.atTarget(50);
    }

    public boolean isStalling() {
        return stallDetector.getStallStatus().isStalled;
    }

    public void setPower(double ignored) {
        // stub method to make this compile for now.
    }
}
