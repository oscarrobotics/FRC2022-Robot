package frc.team832.robot.subsystems;

import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
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
    private PIDController shooterPID = new PIDController(ShooterConstants.KP, 0, 0);
    private final SimpleMotorFeedforward feedforward = ShooterConstants.FEEDFORWARD;
    
    //Instantiate shooter RPM + feedfoward variables
    public double shooterTargetRPM, shooterActualRPM, shooterPIDEffort, shooterFFEffort;

    /** Creates a new ShooterSubsystem **/
    public ShooterSubsystem() {
        shooterMotor.limitInputCurrent(CURRENT_LIMIT);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooterActualRPM", shooterMotor.getSensorVelocity());
        SmartDashboard.putNumber("shooterTargetRPM", shooterTargetRPM);
        
        updateControlLoops();
    }

    public void updateControlLoops() {
        runShooterPID();
    }
    
    public void runShooterPID() {
        shooterActualRPM = shooterMotor.getSensorVelocity();
        
        if (shooterTargetRPM != 0) {
            shooterFFEffort = feedforward.calculate(shooterTargetRPM) / 12.0;
            shooterPIDEffort = shooterPID.calculate(shooterActualRPM, shooterTargetRPM) / 12.0;
        } else {
            shooterFFEffort = 0;
            shooterPIDEffort = 0;
        }
      
        shooterMotor.set(shooterPIDEffort + shooterFFEffort);
    }

    public void setRPM(double targetRPM) {
        shooterTargetRPM = targetRPM;
    }
    
    public void setPower(double power) {
        shooterMotor.set(power);
    }

    public void idleShooter() {
        shooterMotor.set(0);
    }
}
