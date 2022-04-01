package frc.team832.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.lib.power.monitoring.StallDetector;
import frc.team832.robot.Constants.ClimbConstants;

import static frc.team832.robot.Constants.ClimbConstants.*;
import static frc.team832.robot.Constants.PneumaticsValues.*;

public class ClimbSubsystem extends SubsystemBase{
    /**physical devices */
    private final CANTalonFX climbMotorLeft = new CANTalonFX(CLIMB_LEFT_TALON_ID);
    private final CANTalonFX climbMotorRight = new CANTalonFX(CLIMB_RIGHT_TALON_ID);
    private final Solenoid leftClimbPiston = new Solenoid(PneumaticsModuleType.REVPH, LEFT_CLIMB_SOLENOID_ID);
    private final Solenoid rightClimbPiston = new Solenoid(PneumaticsModuleType.REVPH, RIGHT_CLIMB_SOLENOID_ID);

    /*assigns PID constants + Feed Forward to the climb*/
    private PIDController climbPID = new PIDController(ClimbConstants.KP, 0, ClimbConstants.KD);
    private final ElevatorFeedforward feedforward = ClimbConstants.FEEDFORWARD;
    public double climbTargetPos, climbActualPos, climbPIDEffort, climbFFEffort;

    //visualizes values in the Network Table
    private final NetworkTableEntry dash_climbActualPosRight, dash_climbActualPosLeft;// dash_climbTargetPos, dash_climbFFEffortRight, dash_climbPIDEffortRight, dash_climbFFEffortLeft, dash_climbPIDEffortLeft;
    
    private final StallDetector m_rightStallDetector = new StallDetector(climbMotorRight::getOutputCurrent);
    private final StallDetector m_leftStallDetector = new StallDetector(climbMotorLeft::getOutputCurrent);

    /** Creates a new ClimbSubsytem **/
    public ClimbSubsystem() {
        DashboardManager.addTab(this);
        SmartDashboard.putNumber("Set Climb Target", 0.0);
    
        climbMotorLeft.limitInputCurrent(CURRENT_LIMIT);
        climbMotorRight.limitInputCurrent(CURRENT_LIMIT);

        climbMotorLeft.setNeutralMode(NeutralMode.kBrake);
        climbMotorRight.setNeutralMode(NeutralMode.kBrake);

        // climbMotorLeft.setInverted(true);
        // climbMotorRight.setInverted(true);

        // m_rightStallDetector.setStallCurrent(7);
        // m_leftStallDetector.setStallCurrent(7);

        rezeroClimb();

        // dash_climbTargetPos = DashboardManager.addTabItem(this, "Climb Target Pos", 0.0);
        dash_climbActualPosRight = DashboardManager.addTabItem(this, "Climb Actual Pos Right", 0.0);
        dash_climbActualPosLeft = DashboardManager.addTabItem(this, "Climb Actual Pos Left", 0.0);
        // dash_climbPIDEffortRight = DashboardManager.addTabItem(this, "Climb PID Effort", 0.0);
        // dash_climbFFEffortRight = DashboardManager.addTabItem(this,  "Climb FF Effort", 0.0);
        // dash_climbPIDEffortLeft = DashboardManager.addTabItem(this, "Climb PID Effort", 0.0);
        // dash_climbFFEffortLeft = DashboardManager.addTabItem(this,  "Climb FF Effort", 0.0);
    }

    @Override
    public void periodic() {
        updateControlLoops();
        updateDashboardData();
    }

    public void updateControlLoops() {
        // runClimbPID();
        // setTargetPosition();
    }

    private void updateDashboardData() {
        dash_climbActualPosRight.setDouble(climbMotorRight.getSensorPosition());
        dash_climbActualPosLeft.setDouble(climbMotorLeft.getSensorPosition());
        // dash_climbTargetPos.setDouble(climbTargetPos);
        // dash_climbPIDEffortRight.setDouble(climbPIDEffort);
        // dash_climbFFEffortRight.setDouble(climbFFEffort);

        climbTargetPos = SmartDashboard.getNumber("Set Climb Target", 0.0);
    }

    /*  FFE = motor's velocity / 12 volts
        PID = motor's current position compared to its desired position
          cmnd Adds the FFE and PID effort together for accurate motor effort 
    */
    public void runClimbPID() {
        // ADD PID FOR LEFT SIDE
        climbFFEffort = feedforward.calculate(climbMotorRight.getSensorVelocity()) / 12.0;
        climbPIDEffort = climbPID.calculate(climbMotorRight.getSensorPosition(), climbTargetPos);
        climbMotorRight.set(climbPIDEffort + climbFFEffort);
    }
    
    public void setPower(double leftPow, double rightPow) { 
        climbMotorLeft.set(leftPow);  
        climbMotorRight.set(rightPow);
    }

    public void setLeftPow(double leftPow) { 
        climbMotorLeft.set(leftPow);  
    }

    public void setRightPow(double rightPow) { 
        climbMotorRight.set(rightPow);  
    }

    public void setTargetPosition() {
        if (climbTargetPos > ClimbConstants.MAX_EXTEND_POS) {
            climbTargetPos = ClimbConstants.MAX_EXTEND_POS;
        } else if (climbTargetPos < ClimbConstants.MIN_EXTEND_POS) {
            climbTargetPos = ClimbConstants.MIN_EXTEND_POS;
        }

        climbMotorLeft.setTargetPosition(climbTargetPos);
        climbMotorRight.setTargetPosition(climbTargetPos);
    }

    /**all methods pertaining to Climb cmnds**/
    public void extendClimb(double extendTarget) {
        climbMotorLeft.setTargetPosition(extendTarget);;
        climbMotorRight.setTargetPosition(extendTarget);;
    }

    public void retractClimb(double retractTarget) {
        climbMotorLeft.setTargetPosition(retractTarget);
        climbMotorRight.setTargetPosition(retractTarget);
    }

    public void pivotClimb() {
        leftClimbPiston.set(true);
        rightClimbPiston.set(true);
    }

    public void straightenClimb() {
        leftClimbPiston.set(false);
        rightClimbPiston.set(false);
    }

    public void idle() {
        climbMotorLeft.set(0);
        climbMotorRight.set(0);
    }

    public void rezeroClimb() {
        climbMotorLeft.rezeroSensor();
        climbMotorRight.rezeroSensor();
    }

    public boolean isRightStalling() {
        return m_rightStallDetector.getStallStatus().isStalled;
    }

    public boolean isLeftStalling() {
        return m_leftStallDetector.getStallStatus().isStalled;
    }

    public boolean isStalling() {
        return isLeftStalling() && isRightStalling();
    }
}

