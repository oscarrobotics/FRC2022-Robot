package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import static frc.team832.robot.Constants.IntakeConstants.*;
import static frc.team832.robot.Constants.PneumaticsValues.*;

public class IntakeSubsystem extends SubsystemBase{
    private final CANTalonFX intakeMotor = new CANTalonFX(INTAKE_MOTOR_TALON_ID);
    private final Solenoid intakePistons = new Solenoid(PneumaticsModuleType.REVPH, INTAKE_SOLENOID_ID);

    /** Creates a new IntakeSubsytem **/
    public IntakeSubsystem() {
        intakeMotor.limitInputCurrent(CURRENT_LIMIT);
    }
    
    public void setPower(double power) {
        intakeMotor.set(power);
    }

    public void stop() {
        intakeMotor.set(0);
    }

    public void extendIntake() {
        intakePistons.set(true);
    }

    public void retractIntake() {
        intakePistons.set(false);
    }
}
