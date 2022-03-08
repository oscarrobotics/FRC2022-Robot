package frc.team832.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.robot.Constants.IntakeConstants;

import static frc.team832.robot.Constants.IntakeConstants.*;
import static frc.team832.robot.Constants.PneumaticsValues.*;

public class IntakeSubsystem extends SubsystemBase{
    private final CANTalonFX intakeMotor = new CANTalonFX(INTAKE_MOTOR_TALON_ID);
    private final Solenoid intakePistons = new Solenoid(PneumaticsModuleType.REVPH, INTAKE_SOLENOID_ID);

    private PIDController intakePID = new PIDController(IntakeConstants.KP, 0, 0);
    private final SimpleMotorFeedforward feedforward = IntakeConstants.FEEDFORWARD;
    
    public double intakeTargetRPM, intakeActualRPM, intakePIDEffort, intakeFFEffort;

    private final NetworkTableEntry dash_intakeTargetRPM, dash_intakeActualRPM, dash_intakeMotorRPM, dash_intakeFFEffort, dash_intakePIDEffort;

    /** Creates a new IntakeSubsytem **/
    public IntakeSubsystem() {
        DashboardManager.addTab(this);
        SmartDashboard.putNumber("Set Intake RPM", 0.0);

        intakeMotor.limitInputCurrent(CURRENT_LIMIT);
        intakeMotor.getBaseController().configOpenloopRamp(0.125);

        dash_intakeTargetRPM = DashboardManager.addTabItem(this, "Intake Target RPM", 0.0);
        dash_intakeActualRPM = DashboardManager.addTabItem(this, "Intake Actual RPM", 0.0);
        dash_intakeMotorRPM = DashboardManager.addTabItem(this, "Intake Motor RPM", 0.0);
        dash_intakePIDEffort = DashboardManager.addTabItem(this, "Intake PID Effort", 0.0);
        dash_intakeFFEffort = DashboardManager.addTabItem(this,  "Intake FF Effort", 0.0);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("intakeActualRPM", intakeMotor.getSensorVelocity());
        // SmartDashboard.putNumber("intakeTargetRPM", intakeTargetRPM);
        
        updateControlLoops();
        updateDashboardData();
    }

    public void updateControlLoops() {
        // runIntakePID();
    }

    private void updateDashboardData() {
        dash_intakeTargetRPM.setDouble(intakeTargetRPM);
        dash_intakeActualRPM.setDouble(intakeMotor.getSensorVelocity());
        dash_intakePIDEffort.setDouble(intakePIDEffort);
        dash_intakeFFEffort.setDouble(intakeFFEffort);

        intakeTargetRPM = SmartDashboard.getNumber("Set Intake RPM", 0.0);
    }
    
    private void runIntakePID(){
        //uses motor velocity to determine amnt of outpt needed to reach target
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

    //determines methods for intake cmds
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
