package frc.team832.robot.subsystems;

import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motion.OscarFlywheel;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.lib.power.monitoring.StallDetector;
import frc.team832.lib.power.monitoring.StallDetector.StallDetectorStatus;
import frc.team832.robot.Constants.ShooterConstants;
import frc.team832.robot.Constants.VisionConstants;

import static frc.team832.robot.Constants.ShooterConstants.*;
import static frc.team832.robot.Constants.PneumaticsValues.*;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    /** Physical objects */
    private final CANTalonFX m_frontMotor = new CANTalonFX(FRONT_MOTOR_CAN_ID);
    private final CANTalonFX m_rearMotor = new CANTalonFX(REAR_MOTOR_CAN_ID);

    private final OscarFlywheel m_frontFlywheel = new OscarFlywheel("ShooterSubsystem/Front Flywheel", m_frontMotor, POWER_TRAIN, FRONT_FEEDFORWARD, FRONT_KP, MOI_KGM2);
    private final OscarFlywheel m_rearFlywheel = new OscarFlywheel("ShooterSubsystem/Rear Flywheel", m_rearMotor, POWER_TRAIN, REAR_FEEDFORWARD, REAR_KP, MOI_KGM2);

    private final Solenoid m_hoodSolenoid = new Solenoid(PneumaticsModuleType.REVPH, HOOD_SOLENOID_ID);

    private final StallDetector m_frontStallDetector = new StallDetector(m_frontMotor::getOutputCurrent);

    private double m_frontFlywheelTargetRPM, m_frontFlywheelActualRPM, m_rearFlywheelTargetRPM, m_rearFlywheelActualRPM;
    private final NetworkTableEntry dash_frontFlywheelTargetRPM, dash_frontFlywheelActualRPM, dash_rearFlywheelTargetRPM, dash_rearFlywheelActualRPM, dash_isHoodExtend;
    private final NetworkTableEntry dash_frontAtTarget, dash_rearAtTarget;
    
    private final PhotonCamera gloworm;
    private PhotonTrackedTarget target = new PhotonTrackedTarget();
    private double distanceToTargetMeters;
    
    /** Creates a new ShooterSubsystem **/
    public ShooterSubsystem(PhotonCamera gloworm) {     //motor power limits; Open Loop (no need for feedback)
        DashboardManager.addTab(this);
        
        m_frontMotor.wipeSettings();
        m_rearMotor.wipeSettings();

        m_frontMotor.setNeutralMode(NeutralMode.kCoast);
        m_frontMotor.limitInputCurrent(CURRENT_LIMIT);

        m_rearMotor.setNeutralMode(NeutralMode.kCoast);
        m_rearMotor.limitInputCurrent(CURRENT_LIMIT);

        m_frontFlywheel.setClosedLoop(false);
        m_rearFlywheel.setClosedLoop(false);

        // m_frontStallDetector.setStallCurrent(7);
        
        /** Network tables for shooter **/
        dash_frontFlywheelTargetRPM = DashboardManager.addTabItem(this, "Front Flywheel Target RPM", 0.0);
        dash_frontFlywheelActualRPM = DashboardManager.addTabItem(this, "Front Flywheel Actual RPM", 0.0);
        dash_rearFlywheelTargetRPM = DashboardManager.addTabItem(this, "Rear Flywheel Target RPM", 0.0);
        dash_rearFlywheelActualRPM = DashboardManager.addTabItem(this, "Rear Flywheel Actual RPM", 0.0);
        dash_isHoodExtend = DashboardManager.addTabBooleanBox(this, "Hood is Extended");

        dash_frontAtTarget = DashboardManager.addTabBooleanBox(this, "FrontAtTarget");
        dash_rearAtTarget = DashboardManager.addTabBooleanBox(this, "RearAtTarget");

        this.gloworm = gloworm;
    }

    @Override
    public void periodic() {
        m_frontFlywheel.periodic();
        m_rearFlywheel.periodic();

        updateVisionDistance();
        updateDashboardData();
    }
    /** Updates info about actual and target RPM on dashboard **/
    public void updateDashboardData() {
        dash_frontFlywheelTargetRPM.setDouble(m_frontFlywheelTargetRPM);
        dash_frontFlywheelActualRPM.setDouble(m_frontMotor.getSensorVelocity());
        dash_rearFlywheelTargetRPM.setDouble(m_rearFlywheelTargetRPM);
        dash_rearFlywheelActualRPM.setDouble(m_rearMotor.getSensorVelocity());
        dash_isHoodExtend.setBoolean(m_hoodSolenoid.get());
        dash_frontAtTarget.setBoolean(frontAtTarget(50));
        dash_rearAtTarget.setBoolean(rearAtTarget(50));

    }
    /** Changes RPM to target **/
    public void setRPM(double frontTarget, double rearTarget) {
        m_frontFlywheelTargetRPM = frontTarget;
        m_rearFlywheelTargetRPM = rearTarget;
        m_frontFlywheel.setTargetVelocityRpm(frontTarget);
        m_rearFlywheel.setTargetVelocityRpm(rearTarget);
    }

    public void idle() {
        setRPM(0, 0);
    }

    /** Checks if all wheels are at the target **/
    public boolean frontAtTarget(double tolerance) {
        return m_frontFlywheel.atTarget(tolerance);
    }

    public boolean rearAtTarget(double tolerance) {
        return m_rearFlywheel.atTarget(tolerance);
    }

    public boolean atTarget() {
        return atTarget(100);
    }

    public boolean atTarget(double tolerance) {
        return frontAtTarget(tolerance) && rearAtTarget(tolerance);
    }

    public boolean isStalling() {
        return m_frontStallDetector.getStallStatus().isStalled;
    }

    /** Checks vision to calculate the dist. to target **/
    public void updateVisionDistance() {
        PhotonPipelineResult latestResult = gloworm.getLatestResult();
    
        if (latestResult.hasTargets()) {
            target = latestResult.getBestTarget();

            distanceToTargetMeters = PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.CAMERA_HEIGHT_METERS, 
                VisionConstants.TARGET_HEIGHT_METERS, 
                VisionConstants.CAMERA_PITCH_RADIANS, 
                Units.degreesToRadians(target.getPitch())
            );
        }
        
        SmartDashboard.putNumber("distance to target", distanceToTargetMeters);
    }

    /** Checks whether to set the RPM to low/high goal **/
    public void setVisionRpms(boolean isLow) {
        double frontRpm, rearRpm;

        updateVisionDistance();

        boolean extendHood = isLow ? true : shouldHoodExtendHigh(distanceToTargetMeters);
        setHood(extendHood);

        if(isLow) {
            frontRpm = FRONT_SHOOTER_RPM_LOW_MAP.get(distanceToTargetMeters);
            rearRpm = REAR_SHOOTER_RPM_LOW_MAP.get(distanceToTargetMeters);
        } else {
            frontRpm = FRONT_SHOOTER_RPM_HIGH_MAP.get(distanceToTargetMeters);
            rearRpm = REAR_SHOOTER_RPM_HIGH_MAP.get(distanceToTargetMeters);
        }

        SmartDashboard.putBoolean("hood from vision", extendHood);
        SmartDashboard.putNumber("front rpm via vision", frontRpm);
        SmartDashboard.putNumber("rear rpm via vision", rearRpm);
        setRPM(frontRpm, rearRpm);
    }

    /** Gets target RPM based on vision distance to target **/
    public double getFrontVisionRPM() {
        var frontRpm = FRONT_SHOOTER_RPM_HIGH_MAP.get(distanceToTargetMeters);
        return frontRpm;
    }
    public double getRearVisionRPM() {
        var rearRpm = REAR_SHOOTER_RPM_HIGH_MAP.get(distanceToTargetMeters);
        return rearRpm;
    }

    /** sets hood based on low or high **/
    public void setHood(boolean high) {
        m_hoodSolenoid.set(high);
    }

    /** Checks the odometry of sensor on motor to surface **/
    public double getRearSurfaceSpeed() {
        return m_rearMotor.getSensorVelocity() * 2.5 * 3.14;
    }

    public double getFrontSurfaceSpeed() {
        return m_frontMotor.getSensorVelocity() * 4 * 3.14;
    }
}
