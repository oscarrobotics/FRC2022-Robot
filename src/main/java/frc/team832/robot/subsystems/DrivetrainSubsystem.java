// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team832.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.drive.OscarDTCharacteristics;
import frc.team832.lib.drive.OscarDrivetrain;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.robot.Constants.FieldConstants;
import frc.team832.robot.Constants.RobotConstants;

import static frc.team832.robot.Constants.DrivetrainConstants.*;

import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.sensors.WPI_PigeonIMU;
// import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

public class DrivetrainSubsystem extends SubsystemBase {

  /** Physical devices **/
  public final CANTalonFX m_leftMasterMotor = new CANTalonFX(LEFT_MASTER_TALON_ID);
  public final CANTalonFX m_leftSlaveMotor = new CANTalonFX(LEFT_SLAVE_TALON_ID);
  public final CANTalonFX m_rightMasterMotor = new CANTalonFX(RIGHT_MASTER_TALON_ID);
  public final CANTalonFX m_rightSlaveMotor = new CANTalonFX(RIGHT_SLAVE_TALON_ID);
  // private final WPI_PigeonIMU m_imu = new WPI_PigeonIMU(PIGEON_ID);

  Gyro pigeon = new Gyro() {
      
    @Override
    public void close() throws Exception {
      // no-op
    }

    @Override
    public void calibrate() {
      // m_imu.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
    }

    @Override
    public void reset() {
      // m_imu.setFusedHeading(0);
    }

    @Override
    public double getAngle() {
      // return m_imu.getFusedHeading();
      return 0;
    }

    @Override
    public double getRate() {
      // double[] angles = new double[3];
      // m_imu.getAccelerometerAngles(angles);
      // return angles[2];
      return 0;
    }
    
  };

  private final OscarDrivetrain m_drivetrain;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
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

    // m_imu.calibrate();

    // set current limits
    m_leftMasterMotor.limitOutputCurrent(CURRENT_LIMIT);
    m_leftSlaveMotor.limitOutputCurrent(CURRENT_LIMIT);
    m_rightMasterMotor.limitOutputCurrent(CURRENT_LIMIT);
    m_rightSlaveMotor.limitOutputCurrent(CURRENT_LIMIT);

    // invert right side
    m_rightMasterMotor.setInverted(true);

    // set to brake mode
    m_leftMasterMotor.setNeutralMode(NeutralMode.kBrake);
    m_leftSlaveMotor.setNeutralMode(NeutralMode.kBrake);
    m_rightMasterMotor.setNeutralMode(NeutralMode.kBrake);
    m_rightSlaveMotor.setNeutralMode(NeutralMode.kBrake);

    // zero encodors
    m_leftMasterMotor.rezeroSensor();
    m_rightMasterMotor.rezeroSensor();
    m_leftSlaveMotor.rezeroSensor();
    m_rightSlaveMotor.rezeroSensor();

    // set slave motors to follow masters
    m_leftSlaveMotor.follow(m_leftMasterMotor);
    m_rightSlaveMotor.follow(m_rightMasterMotor);

    // ensure right slave follows master inversion
    m_rightSlaveMotor.getBaseController().setInverted(InvertType.FollowMaster);

    var drivetrainCharacteristics = new OscarDTCharacteristics(
      POWER_TRAIN, WHEELBASE_INCHES, 
      LEFT_FEEDFORWARD, RIGHT_FEEDFORWARD, 
      LEFT_KP, RIGHT_KP, 
      MASS_KG, MOI_KGM2
    );

    // initialize drivetrain object
    m_drivetrain = new OscarDrivetrain(
      m_leftMasterMotor, m_rightMasterMotor,
      pigeon, drivetrainCharacteristics
    );

    DashboardManager.addTab(this);

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
  public void periodic() {
    m_drivetrain.periodic();
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
  }

  public void resetPose() {
    m_drivetrain.resetPose();
  }

  public void zeroEncoders() {
    // zero encodors
    m_leftMasterMotor.rezeroSensor();
    m_rightMasterMotor.rezeroSensor();
    m_leftSlaveMotor.rezeroSensor();
    m_rightSlaveMotor.rezeroSensor();
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
    return setField2dPathCmd.andThen(m_drivetrain.generateRamseteCommand(path, this));
  }

  public void stop() {
    m_drivetrain.stop();
  }

  public void setCurrentField2dTrajectory(Trajectory path) {
    m_drivetrain.addTrajectoryToField(path, "Current Path");
  }
}
