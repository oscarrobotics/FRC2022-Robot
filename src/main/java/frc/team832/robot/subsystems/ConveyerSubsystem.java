package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import static frc.team832.robot.Constants.ConveyerConstants.*;

public class ConveyerSubsystem extends SubsystemBase{
    private final CANTalonFX conveyerMotor = new CANTalonFX(CONVEYER_MOTOR_TALON_ID);

    /** Creates a new ConveyerSubsytem **/
    public ConveyerSubsystem() {
        conveyerMotor.limitInputCurrent(CURRENT_LIMIT);
    }
    
    public void setPower(double power) {
        conveyerMotor.set(power);
    }

    public void stop() {
        conveyerMotor.set(0);
    }
}
