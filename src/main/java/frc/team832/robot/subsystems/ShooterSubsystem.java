package frc.team832.robot.subsystems;

import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motion.OscarFlywheel;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.lib.power.monitoring.StallDetector;
import frc.team832.lib.power.monitoring.StallDetector.StallDetectorStatus;

import static frc.team832.robot.Constants.ShooterConstants.*;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    /** Physical objects */
    private final CANTalonFX m_frontMotor = new CANTalonFX(FRONT_MOTOR_CAN_ID);
    private final CANTalonFX m_rearMotor = new CANTalonFX(REAR_MOTOR_CAN_ID);

    private final OscarFlywheel m_frontFlywheel = new OscarFlywheel("ShooterSubsystem/Front Flywheel", m_frontMotor, POWER_TRAIN, BOTTOM_FEEDFORWARD, BOTTOM_KP, MOI_KGM2);
    private final OscarFlywheel m_rearFlywheel = new OscarFlywheel("ShooterSubsystem/Rear Flywheel", m_rearMotor, POWER_TRAIN, TOP_FEEDFORWARD, TOP_KP, MOI_KGM2);

    private final StallDetector m_frontStallDetector = new StallDetector(m_frontMotor::getOutputCurrent);

    private double m_frontFlywheelTargetRPM, m_frontFlywheelActualRPM, m_rearFlywheelTargetRPM, m_rearFlywheelActualRPM;
    private final NetworkTableEntry dash_frontFlywheelTargetRPM, dash_frontFlywheelActualRPM, dash_rearFlywheelTargetRPM, dash_rearFlywheelActualRPM;
    
    private final PhotonCamera camera;

    /** Creates a new ShooterSubsystem **/
    public ShooterSubsystem(PhotonCamera camera) {
        DashboardManager.addTab(this);

        m_frontMotor.setNeutralMode(NeutralMode.kCoast);
        m_frontMotor.limitInputCurrent(CURRENT_LIMIT);
        m_frontMotor.setInverted(true);

        m_rearMotor.setNeutralMode(NeutralMode.kCoast);
        m_rearMotor.limitInputCurrent(CURRENT_LIMIT);

        m_frontFlywheel.setClosedLoop(false);
        m_rearFlywheel.setClosedLoop(false);

        // stallDetector.setStallCurrent(7);

        dash_frontFlywheelTargetRPM = DashboardManager.addTabItem(this, "Front Flywheel Target RPM", 0.0);
        dash_frontFlywheelActualRPM = DashboardManager.addTabItem(this, "Front Flywheel Actual RPM", 0.0);;
        dash_rearFlywheelTargetRPM = DashboardManager.addTabItem(this, "Rear Flywheel Target RPM", 0.0);;
        dash_rearFlywheelActualRPM = DashboardManager.addTabItem(this, "Rear Flywheel Actual RPM", 0.0);;

        this.camera = camera;
    }

    @Override
    public void periodic() {
        m_frontFlywheel.periodic();
        m_rearFlywheel.periodic();

        updateDashboardData();
    }

    public void updateDashboardData() {
        dash_frontFlywheelTargetRPM.setDouble(m_frontFlywheelTargetRPM);
        dash_frontFlywheelActualRPM.setDouble(m_frontMotor.getSensorVelocity());
        dash_rearFlywheelTargetRPM.setDouble(m_rearFlywheelTargetRPM);
        dash_rearFlywheelActualRPM.setDouble(m_rearMotor.getSensorVelocity());
    }

    public void setRPM(double frontTarget, double rearTarget) {
        m_frontFlywheel.setTargetVelocityRpm(frontTarget);
        m_rearFlywheel.setTargetVelocityRpm(rearTarget);
    }

    public void idleShooter() {
        m_frontFlywheel.setTargetVelocityRpm(0);
        m_rearFlywheel.setTargetVelocityRpm(0);
    }

    public void setRPMForDistanceHigh(double feet) {
        var frontRpm = FRONT_SHOOTER_RPM_MAP.get(feet);
        var rearRpm = REAR_SHOOTER_RPM_MAP.get(feet);
        setRPM(frontRpm, rearRpm);
    }

    public boolean atTarget() {
        return m_frontFlywheel.atTarget(50) && m_rearFlywheel.atTarget(50);
    }

    public boolean isStalling() {
        return m_frontStallDetector.getStallStatus().isStalled;
    }

    public void setPower(double ignored) {
        // stub method to make this compile for now.
    }
}
