package frc.team832.robot.subsystems;

import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import static frc.team832.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem {
    private final CANTalonFX shooterMotor = new CANTalonFX(SHOOTER_MOTOR_TALON_ID);

    /** Creates a new ShooterSubsystem **/
    public ShooterSubsystem() {
        shooterMotor.limitInputCurrent(CURRENT_LIMIT);
    }
    
    public void setPower(double power) {
        shooterMotor.set(power);
    }

    public void stop() {
        shooterMotor.set(0);
    }
}