// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team832.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunEndCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.drive.OscarDTCharacteristics;
import frc.team832.lib.drive.OscarDrivetrain;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants.ClimbConstants;
import frc.team832.robot.Constants.DrivetrainConstants;
import frc.team832.robot.Constants.VisionConstants;

import static frc.team832.robot.Constants.DrivetrainConstants.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.PathPlanner;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DrivetrainSubsystem extends SubsystemBase {

  /** Physical devices **/
  public final CANTalonFX m_leftMasterMotor = new CANTalonFX(LEFT_MASTER_TALON_ID);
  public final CANTalonFX m_leftSlaveMotor = new CANTalonFX(LEFT_SLAVE_TALON_ID);
  public final CANTalonFX m_rightMasterMotor = new CANTalonFX(RIGHT_MASTER_TALON_ID);
  public final CANTalonFX m_rightSlaveMotor = new CANTalonFX(RIGHT_SLAVE_TALON_ID);
  private final WPI_Pigeon2 m_imu = new WPI_Pigeon2(PIGEON_ID);




  private double imu_prevPitch;




  // private final Gyro m_imu;

  private final OscarDrivetrain m_drivetrain;

  private final PhotonCamera gloworm;
  private PhotonTrackedTarget target = new PhotonTrackedTarget();
  
  private double m_aimKp = 0.35;
  private double m_aimKd = 0;
  private PIDController targetingPID = new PIDController(m_aimKp, 0, m_aimKd);

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem(PhotonCamera gloworm) {
    if (!m_leftMasterMotor.getCANConnection()) {
      System.out.println("[DRIVETRAIN] LeftMasterMotor not on CAN!");
    }
    if (!m_leftSlaveMotor.getCANConnection()) {
      System.out.println("[DRIVETRAIN] LeftSlaveMotor not on CAN!");
    }
    if (!m_rightMasterMotor.getCANConnection()) {
      System.out.println("[DRIVETRAIN] RightMasterMotor not on CAN!");
    }
    if (!m_rightSlaveMotor.getCANConnection()) {
      System.out.println("[DRIVETRAIN] RightSlaveMotor not on CAN!");
    }

    m_leftSlaveMotor.wipeSettings();
    m_rightSlaveMotor.wipeSettings();

    // set current limits
    m_leftMasterMotor.limitOutputCurrent(CURRENT_LIMIT);
    m_leftSlaveMotor.limitOutputCurrent(CURRENT_LIMIT);
    m_rightMasterMotor.limitOutputCurrent(CURRENT_LIMIT);
    m_rightSlaveMotor.limitOutputCurrent(CURRENT_LIMIT);

    // invert left side
    m_leftMasterMotor.setInverted(true);

    // set to brake mode
    m_leftMasterMotor.setNeutralMode(NeutralMode.kBrake);
    m_leftSlaveMotor.setNeutralMode(NeutralMode.kBrake);
    m_rightMasterMotor.setNeutralMode(NeutralMode.kBrake);
    m_rightSlaveMotor.setNeutralMode(NeutralMode.kBrake);

    // zero encodors
    m_leftMasterMotor.rezeroSensor();
    m_rightMasterMotor.rezeroSensor();
    // m_leftSlaveMotor.rezeroSensor();
    // m_rightSlaveMotor.rezeroSensor();

    // set slave motors to follow masters
    m_leftSlaveMotor.getBaseController().follow(m_leftMasterMotor.getBaseController());
    m_rightSlaveMotor.getBaseController().follow(m_rightMasterMotor.getBaseController());
    // m_leftSlaveMotor.follow(m_leftMasterMotor);
    // m_rightSlaveMotor.follow(m_rightMasterMotor);

    // ensure left slave follows master inversion
    m_leftSlaveMotor.getBaseController().setInverted(InvertType.FollowMaster);

    var drivetrainCharacteristics = new OscarDTCharacteristics(
      POWER_TRAIN, TRACKWIDTH_METERS, 
      LEFT_FEEDFORWARD, RIGHT_FEEDFORWARD, 
      LEFT_KP, RIGHT_KP, 
      MASS_KG, MOI_KGM2
    );

    /**
    // m_imu = new Gyro() {
    //   @Override
    //   public void close() throws Exception {
    //     // TODO Auto-generated method stub
        
    //   }

    //   @Override
    //   public void calibrate() {
    //     // TODO Auto-generated method stub
        
    //   }

    //   @Override
    //   public void reset() {
    //     // TODO Auto-generated method stub
        
    //   }

    //   @Override
    //   public double getAngle() {
    //     // TODO Auto-generated method stub
    //     return 0;
    //   }

    //   @Override
    //   public double getRate() {
    //     // TODO Auto-generated method stub
    //     return 0;
    //   }
    // };
    **/

    // initialize drivetrain object
    m_drivetrain = new OscarDrivetrain(
      m_leftMasterMotor, m_rightMasterMotor,
      m_imu, drivetrainCharacteristics
    );

    this.gloworm = gloworm;

    DashboardManager.addTab(this);
    SmartDashboard.putNumber("vision kp", m_aimKp);
    SmartDashboard.putNumber("vision kd", m_aimKd);
    

    // m_drivetrain.addPoseToField(FieldConstants.LeftOuterTarmacCorner, "LeftOuterTarmacCorner");
    // m_drivetrain.addPoseToField(FieldConstants.RightOuterTarmacCorner, "RightOuterTarmacCorner");
    // m_drivetrain.addPoseToField(FieldConstants.LeftInnerTarmacCorner, "LeftInnerTarmacCorner");
    // m_drivetrain.addPoseToField(FieldConstants.RightInnerTarmacCorner, "RightInnerTarmacCorner");
    // m_drivetrain.addPoseToField(FieldConstants.LeftTarmacTrueCorner, "LeftTarmacTrueCorner");
    // m_drivetrain.addPoseToField(FieldConstants.RightTarmacTrueCorner, "RightTarmacTrueCorner");
  }

  public void teleopCurvatureDrive(double xSpeed, double zRotation, boolean turnInPlace, double inputScalingPow) {
    m_drivetrain.getDiffDrive().curvatureDrive(xSpeed, zRotation, turnInPlace, inputScalingPow);
  }

  public void teleopArcadeDrive(double xSpeed, double zRotation, double inputScalingPow) {
    m_drivetrain.getDiffDrive().arcadeDrive(xSpeed, zRotation, inputScalingPow);
  }

  public void teleopTankDrive(double leftSpeed, double rightSpeed, double inputScalingPow) {
    m_drivetrain.getDiffDrive().tankDrive(leftSpeed, rightSpeed, inputScalingPow);
  }

  @Override
  public void simulationPeriodic() {
    m_drivetrain.simulationPeriodic();
  }

  @Override
  public void periodic() {
    m_drivetrain.periodic();

    double new_m_aimKp = SmartDashboard.getNumber("vision kp", 0);
    double new_m_aimKd = SmartDashboard.getNumber("vision kd", 0);

    if (m_aimKp != new_m_aimKp) {
      m_aimKp = new_m_aimKp;
      targetingPID.setP(m_aimKp);
    }

    if (m_aimKd != new_m_aimKd) {
      m_aimKd = new_m_aimKd;
      targetingPID.setD(m_aimKd);
    }


    double pitch = getPitch();
    double delta = pitch - imu_prevPitch;
    if(delta < .075) delta = 0;
    imu_prevPitch = pitch;

    SmartDashboard.putNumber("gyro pitch", pitch);
    SmartDashboard.putBoolean("safe to extend", pitch >= ClimbConstants.SAFE_TO_EXTEND);
    SmartDashboard.putNumber("gyro pitch delta", delta);
    SmartDashboard.putBoolean("is swing positive", delta > 0);

    updateVision();
  }

  public Pose2d getPose() {
    return m_drivetrain.getPose();
  }

  public double getLeftMeters() {
    return POWER_TRAIN.calcWheelDistanceMeters(m_leftMasterMotor.getSensorPosition());
  }

  public double getRightMeters() {
    return POWER_TRAIN.calcWheelDistanceMeters(m_rightMasterMotor.getSensorPosition());
  }

  public double getYaw() {
    return m_imu.getYaw();
  }

  public void idleDrivetrain() {
    m_leftMasterMotor.set(0);
    m_rightMasterMotor.set(0);
  }
  
  public void setWheelPower(Double leftVolts, Double rightVolts) {
    m_leftMasterMotor.set(leftVolts);
    m_rightMasterMotor.set(rightVolts);
  }

  public void resetPose(Pose2d newPose) {
    m_drivetrain.resetPose(newPose);
    m_imu.setYaw(newPose.getRotation().getDegrees());
  }

  public void resetPose() {
    m_drivetrain.resetPose();
  }

  public void zeroEncoders() {
    // zero encodors
    m_leftMasterMotor.rezeroSensor();
    m_rightMasterMotor.rezeroSensor();
    // m_leftSlaveMotor.rezeroSensor();
    // m_rightSlaveMotor.rezeroSensor();
  }

  public void setNeutralMode(NeutralMode mode) {
    m_leftMasterMotor.setNeutralMode(mode);
    m_leftSlaveMotor.setNeutralMode(mode);
    m_rightMasterMotor.setNeutralMode(mode);
    m_rightSlaveMotor.setNeutralMode(mode);
  }

  public CommandBase getTrajectoryCommand(Trajectory path) {
    var setField2dPathCmd = new InstantCommand(() -> {
      m_drivetrain.addTrajectoryToField(path, "RamseteCommandPath");
    });

    var trajCmd = m_drivetrain.generateRamseteCommand(path, this);

    var trajAndStopCmd = new ParallelDeadlineGroup(
      trajCmd, 
      new StartEndCommand(() -> {}, this::stop)
    );

    return setField2dPathCmd.andThen(trajAndStopCmd);
  }

  public void stop() {
    m_drivetrain.stop();
  }

  public void setCurrentField2dTrajectory(Trajectory path) {
    m_drivetrain.addTrajectoryToField(path, "Current Path");
  }

  public Trajectory loadPath(String pathName, double maxVel, double maxAccel) {
    return loadPath(pathName, maxVel, maxAccel, false);
  }

  public Trajectory loadPath(String pathName, double maxVel, double maxAccel, boolean reversed) {
    Trajectory trajectory = PathPlanner.loadPath(pathName, maxVel, maxAccel, reversed);
    return trajectory;
  }

  public void updateVision() {
    PhotonPipelineResult latestResult = gloworm.getLatestResult();

    if (latestResult.hasTargets()) {
			target = latestResult.getBestTarget();
      SmartDashboard.putNumber("target yaw", target.getYaw());
      SmartDashboard.putNumber("adjusted target yaw", target.getYaw() + 5);
      SmartDashboard.putNumber("vision pid effort", getTargetingRotationSpeed());
		}
  }
  //Be aware of yaw when fine-tuning the vision
  
  public double getTargetingRotationSpeed() {
    // double targetYaw = target.getYaw() + 5;
    double errorSign = Math.signum(target.getYaw());
    double errorPercentage = Math.abs(target.getYaw() / 27.0);
    errorPercentage *= errorSign;
    SmartDashboard.putNumber("AimError", errorPercentage);
    double ksEffort = (DrivetrainConstants.ANGULAR_KS / 12.0) * -errorSign;
    double effort = targetingPID.calculate(errorPercentage, 0);

    effort = MathUtil.clamp(effort, -0.3, 0.3);

    var dist = PhotonUtils.calculateDistanceToTargetMeters(
      VisionConstants.CAMERA_HEIGHT_METERS, 
      VisionConstants.TARGET_HEIGHT_METERS, 
      VisionConstants.CAMERA_PITCH_RADIANS, 
      Units.degreesToRadians(target.getPitch())
    );
    SmartDashboard.putNumber("Dist", dist);


    return effort + ksEffort;
  }

  public CommandBase getTargetingCommand(DoubleSupplier xPow) {
    return new RunEndCommand(() -> {
      var cmdVisEff = getTargetingRotationSpeed();
      SmartDashboard.putNumber("CmdVisEff", cmdVisEff);
      teleopArcadeDrive(
        xPow.getAsDouble()*.6,
        // 0,
        -cmdVisEff,
        1
      );
    }, 
    () -> {}, 
    this);
  }

  public double getPitch() {
    return m_imu.getPitch();
  }

  public double getSwingDirection() {
    double delta = m_imu.getPitch() - imu_prevPitch;
    imu_prevPitch = m_imu.getPitch();

    if(Math.abs(delta) < .05) return 0;

    return delta;
  }
}
