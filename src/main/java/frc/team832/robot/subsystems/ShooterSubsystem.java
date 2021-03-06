package frc.team832.robot.subsystems;

import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.lib.motors.Motor;
import frc.team832.robot.Constants.ShooterConstants;

import static frc.team832.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("FieldCanBeLocal")
public class ShooterSubsystem extends SubsystemBase{
    private final CANTalonFX shooterMotor = new CANTalonFX(SHOOTER_MOTOR_TALON_ID);

    //Instantiate PID Controller and FeedFoward 
    private PIDController shooterPID = new PIDController(ShooterConstants.ShooterkP, 0, 0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 1/Motor.kFalcon500.kv);
    
    //Instantiate flywheel RPM + feedfoward variables
    public double flywheelTargetRPM, flywheelActualRPM, flywheelMotorRPM, flywheelPIDEffort, flywheelFFEffort;

    /** Creates a new ShooterSubsystem **/
    public ShooterSubsystem() {
        shooterMotor.limitInputCurrent(CURRENT_LIMIT);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooterRPM", shooterMotor.getSensorVelocity());
    }

    public void updateControlLoops() {
        runShooterPID();
    }
    
    public void runShooterPID() {
        flywheelActualRPM = shooterMotor.getSensorVelocity();
        
        if (flywheelTargetRPM != 0) {
            flywheelFFEffort = feedforward.calculate(flywheelTargetRPM) / 12.0;
            flywheelPIDEffort = shooterPID.calculate(flywheelActualRPM, flywheelTargetRPM) / 12.0;
        } else {
            flywheelFFEffort = 0;
            flywheelPIDEffort = 0;
        }
      
        shooterMotor.set(flywheelPIDEffort + flywheelFFEffort);
        
    }

    public void setPower(double power) {
        SmartDashboard.putNumber("dumbEffortVolts", power * 12);
        SmartDashboard.putNumber("ffEffortVolts", feedforward.calculate(Math.abs(power) * 6380));
        shooterMotor.set(power);
    }

    public void stop() {
        shooterMotor.set(0);
    }
}
