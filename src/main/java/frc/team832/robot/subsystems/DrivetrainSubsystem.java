// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.drive.OscarDrivetrain;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import static frc.team832.robot.Constants.DrivetrainConstants.*;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

public class DrivetrainSubsystem extends SubsystemBase {

  /** Physical devices **/
  private final CANTalonFX m_leftMasterMotor = new CANTalonFX(LEFT_MASTER_TALON_ID);
  private final CANTalonFX m_leftSlaveMotor = new CANTalonFX(LEFT_SLAVE_TALON_ID);
  private final CANTalonFX m_rightMasterMotor = new CANTalonFX(RIGHT_MASTER_TALON_ID);
  private final CANTalonFX m_rightSlaveMotor = new CANTalonFX(RIGHT_SLAVE_TALON_ID);
  private final PigeonIMU m_imu = new PigeonIMU(PIGEON_ID);

  private final OscarDrivetrain m_drivetrain;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    // set current limits
    m_leftMasterMotor.limitInputCurrent(CURRENT_LIMIT);
    m_leftSlaveMotor.limitInputCurrent(CURRENT_LIMIT);
    m_rightMasterMotor.limitInputCurrent(CURRENT_LIMIT);
    m_rightSlaveMotor.limitInputCurrent(CURRENT_LIMIT);

    // invert right side
    m_rightMasterMotor.setInverted(true);

    // set slave motors to follow masters
    m_leftSlaveMotor.follow(m_leftMasterMotor);
    m_rightSlaveMotor.follow(m_rightMasterMotor);

    // ensure right slave follows master inversion
    m_rightSlaveMotor.getBaseController().setInverted(InvertType.FollowMaster);

    
    // TODO: Gyro based Pigeon wrapper in GrouchLib
    Gyro pigeon = new Gyro() {
      
      @Override
      public void close() throws Exception {
        // no-op
      }

      @Override
      public void calibrate() {
        m_imu.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
      }

      @Override
      public void reset() {
        m_imu.setFusedHeading(0);
      }

      @Override
      public double getAngle() {
        return m_imu.getFusedHeading();
      }

      @Override
      public double getRate() {
        double[] angles = new double[3];
        m_imu.getAccelerometerAngles(angles);
        return angles[2];
      }
      
    };

    // initialize drivetrain object
    m_drivetrain = new OscarDrivetrain(m_leftMasterMotor, m_rightMasterMotor, pigeon, POWER_TRAIN, WHEEBASE_INCHES);
  }

  public void teleopCurvatureDrive(double xSpeed, double zRotation, boolean turnInPlace, double inputScalingPow) {
    m_drivetrain.getDiffDrive().curvatureDrive(xSpeed, zRotation, turnInPlace, inputScalingPow);
  }

  @Override
  public void periodic() {
    m_drivetrain.periodic();
  }
}