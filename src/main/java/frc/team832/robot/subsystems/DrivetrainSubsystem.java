// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team832.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.drive.OscarDTCharacteristics;
import frc.team832.lib.drive.OscarDrivetrain;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardWidget;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;

import static frc.team832.robot.Constants.DrivetrainConstants.*;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

public class DrivetrainSubsystem extends SubsystemBase {

  /** Physical devices **/
  public final CANTalonFX m_leftMasterMotor = new CANTalonFX(LEFT_MASTER_TALON_ID);
  public final CANTalonFX m_leftSlaveMotor = new CANTalonFX(LEFT_SLAVE_TALON_ID);
  public final CANTalonFX m_rightMasterMotor = new CANTalonFX(RIGHT_MASTER_TALON_ID);
  public final CANTalonFX m_rightSlaveMotor = new CANTalonFX(RIGHT_SLAVE_TALON_ID);
  private final WPI_PigeonIMU m_imu = new WPI_PigeonIMU(PIGEON_ID);

  private final OscarDrivetrain m_drivetrain;

  private final NetworkTableEntry m_db_leftMeters, m_db_rightMeters;

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

    m_imu.calibrate();


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

    // set slave motors to follow masters
    m_leftSlaveMotor.follow(m_leftMasterMotor);
    m_rightSlaveMotor.follow(m_rightMasterMotor);

    // ensure right slave follows master inversion
    m_rightSlaveMotor.getBaseController().setInverted(InvertType.FollowMaster);

    var drivetrainCharacteristics = new OscarDTCharacteristics(
      POWER_TRAIN, WHEELBASE_INCHES, 
      LEFT_FEEDFORWARD, RIGHT_FEEDFORWARD, 
      LEFT_KP, RIGHT_KP
    );

    // initialize drivetrain object
    m_drivetrain = new OscarDrivetrain(
      m_leftMasterMotor, m_rightMasterMotor,
      m_imu, drivetrainCharacteristics
    );

    DashboardManager.addTab(this);
    m_db_leftMeters = DashboardManager.addTabItem(this, "Left Meters", 0.0, DashboardWidget.TextView);
    m_db_rightMeters = DashboardManager.addTabItem(this, "Right Meters", 0.0, DashboardWidget.TextView);
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
    
    m_db_leftMeters.setDouble(getLeftMeters());
    m_db_rightMeters.setDouble(getRightMeters());
  }

  public Pose2d getPose() {
    return m_drivetrain.getPose();
  }

  public double getLeftMeters() {
    return POWER_TRAIN.calculateWheelDistanceMeters(m_leftMasterMotor.getSensorPosition());
  }

  public double getRightMeters() {
    return POWER_TRAIN.calculateWheelDistanceMeters(m_rightMasterMotor.getSensorPosition());
  }

  // USED FOR BASIC 2 BALL AUTO
  public boolean isAtBall() {
    return m_drivetrain.getPose() == new Pose2d(0.0, -1.5, new Rotation2d(0));
  }
  // USED FOR BASIC 2 BALL AUTO
  public boolean isAtGoal() {
    return m_drivetrain.getPose() == new Pose2d(0.0, 2, new Rotation2d(0));
  }

  public void idleDrivetrain() {
    m_leftMasterMotor.set(0);
    m_rightMasterMotor.set(0);
  }
  
  public void setWheelPower(Double leftVolts, Double rightVolts) {
    m_leftMasterMotor.set(leftVolts);
    m_rightMasterMotor.set(rightVolts);
  }

  public void setNeutralMode(NeutralMode mode) {
    m_leftMasterMotor.setNeutralMode(mode);
    m_leftSlaveMotor.setNeutralMode(mode);
    m_rightMasterMotor.setNeutralMode(mode);
    m_rightSlaveMotor.setNeutralMode(mode);
  }
}
