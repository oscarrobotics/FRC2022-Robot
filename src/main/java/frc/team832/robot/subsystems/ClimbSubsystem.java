package frc.team832.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.lib.power.monitoring.StallDetector;
import frc.team832.lib.util.OscarMath;

import static frc.team832.robot.Constants.ClimbConstants.*;
import static frc.team832.robot.Constants.PneumaticsValues.*;

public class ClimbSubsystem extends SubsystemBase{
    /**physical devices */
    private final CANTalonFX m_leftMotor = new CANTalonFX(CLIMB_LEFT_TALON_ID);
    private final CANTalonFX m_rightMotor = new CANTalonFX(CLIMB_RIGHT_TALON_ID);
    // private final Solenoid leftClimbPiston = new Solenoid(PneumaticsModuleType.REVPH, LEFT_CLIMB_SOLENOID_ID);
    // private final Solenoid rightClimbPiston = new Solenoid(PneumaticsModuleType.REVPH, RIGHT_CLIMB_SOLENOID_ID);
    private final Solenoid climbPiston = new Solenoid(PneumaticsModuleType.REVPH, CLIMB_SOLENOID_ID);

    /*assigns PID constants + Feed Forward to the climb*/
    private ProfiledPIDController m_leftPID = new ProfiledPIDController(LEFT_KP, 0, LEFT_KD, new TrapezoidProfile.Constraints(MAX_LEFT_ENCODER_VELOCITY, 250));
    private ProfiledPIDController m_rightPID = new ProfiledPIDController(RIGHT_KP, 0, RIGHT_KD, new TrapezoidProfile.Constraints(MAX_RIGHT_ENCODER_VELOCITY, 250));
    private final ElevatorFeedforward leftFF = LEFT_FEEDFORWARD;
    private final ElevatorFeedforward rightFF = RIGHT_FEEDFORWARD;
    public double m_leftTargetPos, m_rightTargetPos, leftActualPos, rightActualPos, m_rightPIDEffort, m_rightFFEffort, m_leftPIDEffort, m_leftFFEffort;

    public double m_leftRawEffort, m_rightRawEffort;

    private boolean m_usePid = false;


    //visualizes values in the Network Table
    private final NetworkTableEntry dash_rightActualPos, dash_leftActualPos, dash_leftTargetPos, dash_rightTargetPos, dash_leftFFEffort, dash_leftPIDEffort, dash_rightFFEffort, dash_rightPIDEffort;
    private final NetworkTableEntry dash_leftRawEffort, dash_rightRawEffort;
    private final NetworkTableEntry dash_usePid;
    private final NetworkTableEntry dash_rightOutputCurrentDraw, dash_leftOutputCurrentDraw, dash_rightInputCurrentDraw, dash_leftInputCurrentDraw;
    private final NetworkTableEntry dash_leftHomingStalled, dash_rightHomingStalled;
    
    private final StallDetector m_rightHomingStallDetector = new StallDetector(m_rightMotor::getInputCurrent);
    private final StallDetector m_leftHomingStallDetector = new StallDetector(m_leftMotor::getInputCurrent);

    /** Creates a new ClimbSubsytem **/
    public ClimbSubsystem() {
        DashboardManager.addTab(this);
    
        m_leftMotor.limitInputCurrent(CURRENT_LIMIT);
        m_rightMotor.limitInputCurrent(CURRENT_LIMIT);

        m_leftMotor.setNeutralMode(NeutralMode.kBrake);
        m_rightMotor.setNeutralMode(NeutralMode.kBrake);

        m_leftMotor.setInverted(true);
        //rightMotor.setInverted(true);

        m_leftHomingStallDetector.setStallCurrent(15);
        m_rightHomingStallDetector.setStallCurrent(15);
        m_leftHomingStallDetector.setMinStallMillis(250);
        m_rightHomingStallDetector.setMinStallMillis(250);

        zeroClimb();
        
        m_leftPID.setP(.2);
        m_leftPID.setD(0);

        m_rightPID.setP(.2);
        m_rightPID.setD(0);

        m_leftPID.setTolerance(.75);
        m_rightPID.setTolerance(.75);

        dash_leftActualPos = DashboardManager.addTabItem(this, "Left Actual Pos", 0.0);
        dash_rightActualPos = DashboardManager.addTabItem(this, "Right Actual Pos", 0.0);
        dash_leftTargetPos = DashboardManager.addTabItem(this, "Left Target Pos", 0.0);
        dash_rightTargetPos = DashboardManager.addTabItem(this, "Right Target Pos", 0.0);
        dash_leftPIDEffort = DashboardManager.addTabItem(this, "Left PID Effort", 0.0);
        dash_rightPIDEffort = DashboardManager.addTabItem(this, "Right PID Effort", 0.0);
        dash_leftFFEffort = DashboardManager.addTabItem(this,  "Left FF Effort", 0.0);
        dash_rightFFEffort = DashboardManager.addTabItem(this,  "Right FF Effort", 0.0);
        dash_leftRawEffort = DashboardManager.addTabNumberBar(this, "leftRawEffort", -12, 12);
        dash_rightRawEffort = DashboardManager.addTabNumberBar(this, "rightRawEffort", -12, 12);

        dash_leftOutputCurrentDraw = DashboardManager.addTabItem(this, "left output current draw", 0.0);
        dash_rightOutputCurrentDraw = DashboardManager.addTabItem(this, "right output current draw", 0.0);
        dash_leftInputCurrentDraw = DashboardManager.addTabItem(this, "left input current draw", 0.0);
        dash_rightInputCurrentDraw = DashboardManager.addTabItem(this, "right input current draw", 0.0);

        dash_leftHomingStalled = DashboardManager.addTabBooleanBox(this, "leftHomingStalled");
        dash_rightHomingStalled = DashboardManager.addTabBooleanBox(this, "rightHomingStalled");

        dash_usePid = DashboardManager.addTabBooleanBox(this, "UsePID");

        setDefaultCommand(new StartEndCommand(this::idle, () -> {}, this).withName("default idle cmd"));
    }

    @Override
    public void periodic() {
        m_leftHomingStallDetector.updateStallStatus();
        m_rightHomingStallDetector.updateStallStatus();

        updateControlLoops();
        updateDashboardData();
        setMotorOutputs();
    }

    private void setMotorOutputs() {
        if (m_usePid) {
            m_leftRawEffort = m_leftPIDEffort + m_leftFFEffort;
            m_rightRawEffort = m_rightPIDEffort + m_rightFFEffort;
        }

        m_leftMotor.set(m_leftRawEffort);
        m_rightMotor.set(m_rightRawEffort);
    }

    public void updateControlLoops() {
        dash_usePid.setBoolean(m_usePid);
        runClimbPID();
    }

    public void reset() {
        idle();
        zeroClimb();
        m_leftTargetPos = 0;
        m_rightRawEffort = 0;
    }

    private void updateDashboardData() {
        dash_rightActualPos.setDouble(m_rightMotor.getSensorPosition());
        dash_leftActualPos.setDouble(m_leftMotor.getSensorPosition());
        dash_leftTargetPos.setDouble(m_leftTargetPos);
        dash_rightTargetPos.setDouble(m_rightTargetPos);
        dash_leftPIDEffort.setDouble(m_leftPIDEffort);
        dash_rightPIDEffort.setDouble(m_rightPIDEffort);
        dash_leftFFEffort.setDouble(m_leftFFEffort);
        dash_rightFFEffort.setDouble(m_rightFFEffort);
        dash_leftRawEffort.setDouble(m_leftRawEffort);
        dash_rightRawEffort.setDouble(m_rightRawEffort);
        
        dash_leftOutputCurrentDraw.setDouble(m_leftMotor.getOutputCurrent());
        dash_rightOutputCurrentDraw.setDouble(m_rightMotor.getOutputCurrent());
        dash_leftInputCurrentDraw.setDouble(m_leftMotor.getInputCurrent());
        dash_rightInputCurrentDraw.setDouble(m_rightMotor.getInputCurrent());

        dash_leftHomingStalled.setBoolean(isLeftStalling());
        dash_rightHomingStalled.setBoolean(isRightStalling());
    }

    /*  FFE = motor's velocity / 12 volts
        PID = motor's current position compared to its desired position
          cmnd Adds the FFE and PID effort together for accurate motor effort 
    */
    public void runClimbPID() {
        if (m_leftPID.atGoal()) {
            m_leftPIDEffort = 0;
            m_leftFFEffort = 0;
        } else {
            m_leftPIDEffort = m_leftPID.calculate(m_leftMotor.getSensorPosition(), m_leftTargetPos);
            m_leftFFEffort = leftFF.calculate(m_leftPID.getGoal().velocity) / 12.0;
        }

        if (m_rightPID.atGoal()) {
            m_rightPIDEffort = 0;
            m_rightFFEffort = 0;
        } else {
            m_rightPIDEffort = m_rightPID.calculate(m_rightMotor.getSensorPosition(), m_rightTargetPos);
            m_rightFFEffort = rightFF.calculate(m_rightPID.getGoal().velocity) / 12.0;
        }
    }

    public boolean atTarget() {
        m_leftPID.setGoal(m_leftTargetPos);
        m_rightPID.setGoal(m_rightTargetPos);
        return m_leftPID.atGoal() && m_rightPID.atGoal();
    }

    public void setPower(double leftPow, double rightPow) {
        m_usePid = false;

        boolean leftUnderMax = m_leftMotor.getSensorPosition() <= LEFT_MAX_EXTEND_POS;

        if (leftPow <= 0 || leftUnderMax) {
            m_leftRawEffort = leftPow;
        }

        boolean rightUnderMax = m_rightMotor.getSensorPosition() <= RIGHT_MAX_EXTEND_POS;

        if (rightPow <= 0 || rightUnderMax) {
            m_rightRawEffort = rightPow;
        } 
    }

    public void setTargetPosition(double leftPos, double rightPos) {
        m_usePid = true;

        leftPos = OscarMath.clip(leftPos, 0, LEFT_MAX_EXTEND_POS);
        rightPos = OscarMath.clip(rightPos, 0, RIGHT_MAX_EXTEND_POS);

        m_leftTargetPos = leftPos;
        m_leftPID.setGoal(m_leftTargetPos);

        m_rightTargetPos = rightPos;
        m_rightPID.setGoal(m_rightTargetPos);
    }

    public void pivotClimb() {
        climbPiston.set(true);
    }

    public void straightenClimb() {
        climbPiston.set(false);
    }

    public void idle(boolean wasClimbing) {
        m_usePid = false;
        m_leftRawEffort = 0;
        m_rightRawEffort = 0;

        // if (!wasClimbing) {
            // straightenClimb();
        // }
    }

    public void idle() {
        idle(false);
    }

    public void zeroClimb() {
        m_leftMotor.rezeroSensor();
        m_rightMotor.rezeroSensor();
    }

    public void zeroLeft() {
        m_leftMotor.rezeroSensor();
    }

    public void zeroRight() {
        m_rightMotor.rezeroSensor();
    }

    public boolean isRightStalling() {
        return m_rightHomingStallDetector.getStallStatus().isStalled;
    }

    public boolean isLeftStalling() {
        return m_leftHomingStallDetector.getStallStatus().isStalled;
    }

    public boolean isStalling() {
        return isLeftStalling() && isRightStalling();
    }

    public double getRightVelocity() {
        return m_rightMotor.getSensorVelocity();
    }

    public double getLeftVelocity() {
        return m_leftMotor.getSensorVelocity();
    }

    public double getRightPosition() {
        return m_rightMotor.getSensorPosition();
    }

    public double getLeftPosition() {
        return m_leftMotor.getSensorPosition();
    }

    public void clearTarget() {
        m_usePid = false;
        m_leftTargetPos = 0;
        m_rightTargetPos = 0;
    }
}

