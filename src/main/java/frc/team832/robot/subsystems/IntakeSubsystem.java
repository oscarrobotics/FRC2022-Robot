package frc.team832.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.robot.Constants.IntakeConstants;

import static frc.team832.robot.Constants.IntakeConstants.*;
import static frc.team832.robot.Constants.PneumaticsValues.*;

public class IntakeSubsystem extends SubsystemBase{
    private final CANTalonFX intakeMotor = new CANTalonFX(INTAKE_MOTOR_TALON_ID);
    private final Solenoid intakePistons = new Solenoid(PneumaticsModuleType.REVPH, INTAKE_SOLENOID_ID);

    //Instantiate PID Controller and FeedFoward 
    private PIDController intakePID = new PIDController(IntakeConstants.KP, 0, 0);
    private final SimpleMotorFeedforward feedforward = IntakeConstants.FEEDFORWARD;
    
    //Instantiate intake RPM + feedfoward variables
    public double intakeTargetRPM, intakeActualRPM, intakePIDEffort, intakeFFEffort;
    /** Creates a new IntakeSubsytem **/
    public IntakeSubsystem() {
        intakeMotor.limitInputCurrent(CURRENT_LIMIT);
        intakeMotor.getBaseController().configOpenloopRamp(0.125);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intakeActualRPM", intakeMotor.getSensorVelocity());
        SmartDashboard.putNumber("intakeTargetRPM", intakeTargetRPM);
        
        updateControlLoops();
    }

    public void updateControlLoops() {
        // runIntakePID();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intakeActualRPM", intakeMotor.getSensorVelocity());
        SmartDashboard.putNumber("intakeTargetRPM", intakeTargetRPM);
        
        updateControlLoops();
    }

    public void updateControlLoops() {
        runIntakePID();
    }
    
    public void runIntakePID(){
        intakeActualRPM = intakeMotor.getSensorVelocity();
        
        if (intakeTargetRPM != 0) {
            intakeFFEffort = feedforward.calculate(intakeTargetRPM) / 12.0;
            intakePIDEffort = intakePID.calculate(intakeActualRPM, intakeTargetRPM) / 12.0;
        } else {
            intakeFFEffort = 0;
            intakePIDEffort = 0;
        }
      
        intakeMotor.set(intakePIDEffort + intakeFFEffort);
    }
    
    public void setRPM(double targetRPM) {
        intakeTargetRPM = targetRPM;
    }

    public void setPower(double power) {
        intakeMotor.set(power);
    }

    public void extendIntake() {
        intakePistons.set(true);
    }

    public void retractIntake() {
        intakePistons.set(false);
    }

    public void idleIntake() {
        intakeMotor.set(0);
    }
}
