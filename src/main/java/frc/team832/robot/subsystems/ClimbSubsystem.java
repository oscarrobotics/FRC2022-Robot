package frc.team832.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.lib.power.monitoring.StallDetector;

import static frc.team832.robot.Constants.ClimbConstants.*;
import static frc.team832.robot.Constants.PneumaticsValues.*;

public class ClimbSubsystem extends SubsystemBase{
    /**physical devices */
    private final CANTalonFX leftMotor = new CANTalonFX(CLIMB_LEFT_TALON_ID);
    private final CANTalonFX rightMotor = new CANTalonFX(CLIMB_RIGHT_TALON_ID);
    // private final Solenoid leftClimbPiston = new Solenoid(PneumaticsModuleType.REVPH, LEFT_CLIMB_SOLENOID_ID);
    // private final Solenoid rightClimbPiston = new Solenoid(PneumaticsModuleType.REVPH, RIGHT_CLIMB_SOLENOID_ID);
    private final Solenoid climbPiston = new Solenoid(PneumaticsModuleType.REVPH, CLIMB_SOLENOID_ID);

    /*assigns PID constants + Feed Forward to the climb*/
    private ProfiledPIDController leftPID = new ProfiledPIDController(LEFT_KP, 0, LEFT_KD, new TrapezoidProfile.Constraints(MAX_LEFT_ENCODER_VELOCITY, MAX_LEFT_ENCODER_VELOCITY));
    private ProfiledPIDController rightPID = new ProfiledPIDController(RIGHT_KP, 0, RIGHT_KD, new TrapezoidProfile.Constraints(MAX_RIGHT_ENCODER_VELOCITY, MAX_RIGHT_ENCODER_VELOCITY));
    private final ElevatorFeedforward leftFF = LEFT_FEEDFORWARD;
    private final ElevatorFeedforward rightFF = RIGHT_FEEDFORWARD;
    public double leftTargetPos, rightTargetPos, leftActualPos, rightActualPos, rightPIDEffort, rightFFEffort, leftPIDEffort, leftFFEffort;

    //visualizes values in the Network Table
    private final NetworkTableEntry dash_rightActualPos, dash_leftActualPos, dash_leftTargetPos, dash_rightTargetPos, dash_leftKP, dash_leftKD, dash_rightKP, dash_rightKD, dash_leftFFEffort, dash_leftPIDEffort, dash_rightFFEffort, dash_rightPIDEffort;
    
    private final StallDetector m_rightStallDetector = new StallDetector(rightMotor::getOutputCurrent);
    private final StallDetector m_leftStallDetector = new StallDetector(leftMotor::getOutputCurrent);

    /** Creates a new ClimbSubsytem **/
    public ClimbSubsystem() {
        DashboardManager.addTab(this);
    
        leftMotor.limitInputCurrent(CURRENT_LIMIT);
        rightMotor.limitInputCurrent(CURRENT_LIMIT);

        leftMotor.setNeutralMode(NeutralMode.kBrake);
        rightMotor.setNeutralMode(NeutralMode.kBrake);

        leftMotor.setInverted(true);
        //rightMotor.setInverted(true);

        // m_rightStallDetector.setStallCurrent(7);
        // m_leftStallDetector.setStallCurrent(7);

        zeroClimb();

        dash_leftActualPos = DashboardManager.addTabItem(this, "Left Actual Pos", 0.0);
        dash_rightActualPos = DashboardManager.addTabItem(this, "Right Actual Pos", 0.0);
        dash_leftTargetPos = DashboardManager.addTabItem(this, "Left Target Pos", 0.0);
        dash_rightTargetPos = DashboardManager.addTabItem(this, "Right Target Pos", 0.0);
        dash_leftPIDEffort = DashboardManager.addTabItem(this, "Left PID Effort", 0.0);
        dash_rightPIDEffort = DashboardManager.addTabItem(this, "Right PID Effort", 0.0);
        dash_leftFFEffort = DashboardManager.addTabItem(this,  "Left FF Effort", 0.0);
        dash_rightFFEffort = DashboardManager.addTabItem(this,  "Right FF Effort", 0.0);
        dash_leftKP = DashboardManager.addTabItem(this, "Left KP", 0.0);
        dash_leftKD = DashboardManager.addTabItem(this, "Left KD", 0.0);
        dash_rightKP = DashboardManager.addTabItem(this, "Right KP", 0.0);
        dash_rightKD = DashboardManager.addTabItem(this, "Right KD", 0.0);
    }

    @Override
    public void periodic() {
        // updateControlLoops();
        updateDashboardData();
    }

    public void updateControlLoops() {
        setPID();
        runClimbPID();
        // setTargetPosition();
    }

    private void updateDashboardData() {
        dash_rightActualPos.setDouble(rightMotor.getSensorPosition());
        dash_leftActualPos.setDouble(leftMotor.getSensorPosition());
        dash_leftTargetPos.setDouble(leftTargetPos);
        dash_rightTargetPos.setDouble(rightTargetPos);
        dash_leftPIDEffort.setDouble(leftPIDEffort);
        dash_rightPIDEffort.setDouble(rightPIDEffort);
        dash_leftFFEffort.setDouble(leftFFEffort);
        dash_rightFFEffort.setDouble(rightFFEffort);
    }

    /*  FFE = motor's velocity / 12 volts
        PID = motor's current position compared to its desired position
          cmnd Adds the FFE and PID effort together for accurate motor effort 
    */
    public void runClimbPID() {
        leftFFEffort = leftFF.calculate(leftMotor.getSensorVelocity()) / 12.0;
        leftPIDEffort = leftPID.calculate(leftMotor.getSensorPosition(), leftTargetPos);
        leftMotor.set(leftFFEffort + leftPIDEffort);
        
        rightFFEffort = rightFF.calculate(rightMotor.getSensorVelocity()) / 12.0;
        rightPIDEffort = rightPID.calculate(rightMotor.getSensorPosition(), rightTargetPos);
        rightMotor.set(rightFFEffort + rightPIDEffort);
    }
    
    public void setPID() {
        leftPID.setP(dash_leftKP.getDouble(0.0));
        leftPID.setD(dash_leftKD.getDouble(0.0));

        rightPID.setP(dash_rightKP.getDouble(0.0));
        rightPID.setD(dash_rightKD.getDouble(0.0));
    }

    public void setPower(double leftPow, double rightPow) { 
        leftMotor.set(leftPow);  
        rightMotor.set(rightPow);
    }

    public void setPower(double pow) {   
        if (pow <= 0) {
            leftMotor.set(pow);
            rightMotor.set(pow);
            System.out.println("pow <= 0");
        } else {
            System.out.println("pow > 0");

            if (leftMotor.getSensorPosition() <= LEFT_MAX_EXTEND_POS) {
                leftMotor.set(pow); 
                System.out.println("left < max"); 
            } else {
                System.out.println("left @ max"); 
                leftMotor.set(0);
            }
            
            if (rightMotor.getSensorPosition() <= RIGHT_MAX_EXTEND_POS) {
                rightMotor.set(pow);
                System.out.println("right < max"); 
            } else {
                System.out.println("right @ max"); 
                rightMotor.set(0);
            }       
        }
    }

    public void setLeftPow(double leftPow) { 
        leftMotor.set(leftPow);  
    }

    public void setRightPow(double rightPow) { 
        rightMotor.set(rightPow);  
    }

    public void setTargetPosition(double leftPos, double rightPos) {
        // if (climbTargetPos > ClimbConstants.MAX_EXTEND_POS) {
        //     climbTargetPos = ClimbConstants.MAX_EXTEND_POS;
        // } else if (climbTargetPos < ClimbConstants.MIN_EXTEND_POS) {
        //     climbTargetPos = ClimbConstants.MIN_EXTEND_POS;
        // }

        // leftMotor.setTargetPosition(climbTargetPos);
        // rightMotor.setTargetPosition(climbTargetPos);

        leftTargetPos = leftPos;
        rightTargetPos = rightPos;
        
        leftMotor.setTargetPosition(leftTargetPos);
        rightMotor.setTargetPosition(rightTargetPos);
    }

    /**all methods pertaining to Climb cmnds**/
    public void extendClimb(double extendTarget) {
        leftMotor.setTargetPosition(extendTarget);;
        rightMotor.setTargetPosition(extendTarget);;
    }

    public void retractClimb(double retractTarget) {
        leftMotor.setTargetPosition(retractTarget);
        rightMotor.setTargetPosition(retractTarget);
    }

    public void pivotClimb() {
        // leftClimbPiston.set(true);
        // rightClimbPiston.set(true);
        climbPiston.set(true);
    }

    public void straightenClimb() {
        // leftClimbPiston.set(false);
        // rightClimbPiston.set(false);
        climbPiston.set(false);
    }

    public void idle() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public void zeroClimb() {
        leftMotor.rezeroSensor();
        rightMotor.rezeroSensor();
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

    public double getRightVelocity() {
        return rightMotor.getSensorVelocity();
    }

    public double getLeftVelocity() {
        return leftMotor.getSensorVelocity();
    }

    public double getRightPosition() {
        return rightMotor.getSensorPosition();
    }

    public double getLeftPosition() {
        return leftMotor.getSensorPosition();
    }
}

