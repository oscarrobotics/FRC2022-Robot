package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import static frc.team832.robot.Constants.ClimbConstants.*;
import static frc.team832.robot.Constants.PneumaticsValues.*;

public class ClimbSubsystem extends SubsystemBase{
    private final CANTalonFX climbMotorLeft = new CANTalonFX(CLIMB_LEFT_TALON_ID);
    private final CANTalonFX climbMotorRight = new CANTalonFX(CLIMB_RIGHT_TALON_ID);
    private final Solenoid climbPistons = new Solenoid(PneumaticsModuleType.REVPH, CLIMB_SOLENOID_ID);

    /** Creates a new ClimbSubsytem **/
    public ClimbSubsystem() {
        climbMotorLeft.limitInputCurrent(CURRENT_LIMIT);
        climbMotorRight.limitInputCurrent(CURRENT_LIMIT);
    }
    
    public void extendClimb() {
        climbMotorLeft.setTargetPosition(EXTEND_TARGET);;
        climbMotorRight.setTargetPosition(EXTEND_TARGET);;
    }

    public void retractClimb() {
        climbMotorLeft.setTargetPosition(RETRACT_TARGET);
        climbMotorRight.setTargetPosition(RETRACT_TARGET);
    }

    public void pivotClimb() {
        climbPistons.set(true);
    }

    public void straightenClimb() {
        climbPistons.set(false);
    }
}
