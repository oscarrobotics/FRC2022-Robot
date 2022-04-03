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

    private final OscarFlywheel m_frontFlywheel = new OscarFlywheel("ShooterSubsystem/Front Flywheel", m_frontMotor, POWER_TRAIN, BOTTOM_FEEDFORWARD, BOTTOM_KP, MOI_KGM2);
    private final OscarFlywheel m_rearFlywheel = new OscarFlywheel("ShooterSubsystem/Rear Flywheel", m_rearMotor, POWER_TRAIN, TOP_FEEDFORWARD, TOP_KP, MOI_KGM2);

    private final Solenoid m_hoodSolenoid = new Solenoid(PneumaticsModuleType.REVPH, HOOD_SOLENOID_ID);

    private final StallDetector m_frontStallDetector = new StallDetector(m_frontMotor::getOutputCurrent);

    private double m_frontFlywheelTargetRPM, m_frontFlywheelActualRPM, m_rearFlywheelTargetRPM, m_rearFlywheelActualRPM;
    private final NetworkTableEntry dash_frontFlywheelTargetRPM, dash_frontFlywheelActualRPM, dash_rearFlywheelTargetRPM, dash_rearFlywheelActualRPM;
    
    private final PhotonCamera gloworm;
    private PhotonTrackedTarget target = new PhotonTrackedTarget();
    private double distanceToTargetMeters;
    
    /** Creates a new ShooterSubsystem **/
    public ShooterSubsystem(PhotonCamera gloworm) {
        DashboardManager.addTab(this);

        m_frontMotor.setNeutralMode(NeutralMode.kCoast);
        m_frontMotor.limitInputCurrent(CURRENT_LIMIT);

        m_rearMotor.setNeutralMode(NeutralMode.kCoast);
        m_rearMotor.limitInputCurrent(CURRENT_LIMIT);

        m_frontFlywheel.setClosedLoop(false);
        m_rearFlywheel.setClosedLoop(false);

        // m_frontStallDetector.setStallCurrent(7);

        dash_frontFlywheelTargetRPM = DashboardManager.addTabItem(this, "Front Flywheel Target RPM", 0.0);
        dash_frontFlywheelActualRPM = DashboardManager.addTabItem(this, "Front Flywheel Actual RPM", 0.0);;
        dash_rearFlywheelTargetRPM = DashboardManager.addTabItem(this, "Rear Flywheel Target RPM", 0.0);;
        dash_rearFlywheelActualRPM = DashboardManager.addTabItem(this, "Rear Flywheel Actual RPM", 0.0);;

        this.gloworm = gloworm;
    }

    @Override
    public void periodic() {
        m_frontFlywheel.periodic();
        m_rearFlywheel.periodic();

        updateDashboardData();
        updateVision();
        updateVisionDistance();
    }

    public void updateDashboardData() {
        dash_frontFlywheelTargetRPM.setDouble(m_frontFlywheelTargetRPM);
        dash_frontFlywheelActualRPM.setDouble(m_frontMotor.getSensorVelocity());
        dash_rearFlywheelTargetRPM.setDouble(m_rearFlywheelTargetRPM);
        dash_rearFlywheelActualRPM.setDouble(m_rearMotor.getSensorVelocity());
    }

    public void setRPM(double frontTarget, double rearTarget) {
        m_frontFlywheelTargetRPM = frontTarget;
        m_rearFlywheelTargetRPM = rearTarget;
        m_frontFlywheel.setTargetVelocityRpm(frontTarget);
        m_rearFlywheel.setTargetVelocityRpm(rearTarget);
    }

    public void idle() {
        m_frontFlywheel.setTargetVelocityRpm(0);
        m_rearFlywheel.setTargetVelocityRpm(0);
    }

    public void setRPMForDistanceHigh(double feet) {
        var frontRpm = BOTTOM_SHOOTER_RPM_MAP.get(feet);
        var rearRpm = TOP_SHOOTER_RPM_MAP.get(feet);
        setRPM(frontRpm, rearRpm);
    }

    public boolean atTarget() {
        return atTarget(50);
    }

    public boolean atTarget(double error) {
        return m_frontFlywheel.atTarget(error) && m_rearFlywheel.atTarget(error);
    }

    public boolean isStalling() {
        return m_frontStallDetector.getStallStatus().isStalled;
    }

    public void setPower(double ignored) {
        // stub method to make this compile for now.
    }

    public void updateVision() {
        PhotonPipelineResult latestResult = gloworm.getLatestResult();
    
        if (latestResult.hasTargets()) {
            target = latestResult.getBestTarget();
        }
    }

    public void updateVisionDistance() {
        distanceToTargetMeters = PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.CAMERA_HEIGHT_METERS, 
            VisionConstants.TARGET_HEIGHT_METERS, 
            VisionConstants.CAMERA_PITCH_RADIANS, 
            Units.degreesToRadians(target.getPitch())
          );
        SmartDashboard.putNumber("distance to target", distanceToTargetMeters);
    }

    public void setVisionRpms() {
        updateVisionDistance();
        var frontRpm = BOTTOM_SHOOTER_RPM_MAP.get(Units.metersToInches(distanceToTargetMeters));
        var rearRpm = TOP_SHOOTER_RPM_MAP.get(Units.metersToInches(distanceToTargetMeters));
        SmartDashboard.putNumber("front rpm via vision", frontRpm);
        SmartDashboard.putNumber("rear rpm via vision", rearRpm);
        setRPM(frontRpm, rearRpm);
    }

    public double getFrontVisionRPM() {
        var frontRpm = BOTTOM_SHOOTER_RPM_MAP.get(distanceToTargetMeters);
        return frontRpm;
    }
    public double getRearVisionRPM() {
        var rearRpm = TOP_SHOOTER_RPM_MAP.get(distanceToTargetMeters);
        return rearRpm;
    }

    public void extendHood() {
        m_hoodSolenoid.set(true);
    }

    public void retractHood() {
        m_hoodSolenoid.set(false);
    }
}
