package frc.team832.robot.subsystems;

import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.robot.Constants.ShooterConstants;

import static frc.team832.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("FieldCanBeLocal")
public class ShooterSubsystem extends SubsystemBase{
    /**physical object */
    private final CANTalonFX shooterMotor = new CANTalonFX(SHOOTER_MOTOR_TALON_ID);

    /**adds PID + FF constants */
    private PIDController shooterPID = new PIDController(ShooterConstants.KP, 0, 0);
    private final SimpleMotorFeedforward feedforward = ShooterConstants.FEEDFORWARD;
    
    public double shooterTargetRPM, shooterActualRPM, shooterPIDEffort, shooterFFEffort;

    private final NetworkTableEntry dash_shooterTargetRPM, dash_shooterActualRPM, dash_shooterMotorRPM, dash_shooterFFEffort, dash_shooterPIDEffort;

    /** Creates a new ShooterSubsystem **/
    public ShooterSubsystem() {
        DashboardManager.addTab(this);
        SmartDashboard.putNumber("Set Shooter RPM", 0.0);

        shooterMotor.limitInputCurrent(CURRENT_LIMIT);

        dash_shooterTargetRPM = DashboardManager.addTabItem(this, "Shooter Target RPM", 0.0);
        dash_shooterActualRPM = DashboardManager.addTabItem(this, "Shooter Actual RPM", 0.0);
        dash_shooterMotorRPM = DashboardManager.addTabItem(this, "Shooter Motor RPM", 0.0);
        dash_shooterPIDEffort = DashboardManager.addTabItem(this, "Shooter PID Effort", 0.0);
        dash_shooterFFEffort = DashboardManager.addTabItem(this,  "Shooter FF Effort", 0.0);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("shooterActualRPM", shooterMotor.getSensorVelocity());
        // SmartDashboard.putNumber("shooterTargetRPM", shooterTargetRPM);
        
        updateControlLoops();
        updateDashboardData();
    }

    public void updateControlLoops() {
        runShooterPID();
    }

    private void updateDashboardData() {
        dash_shooterTargetRPM.setDouble(shooterTargetRPM);
        dash_shooterActualRPM.setDouble(shooterMotor.getSensorVelocity());
        dash_shooterPIDEffort.setDouble(shooterPIDEffort);
        dash_shooterFFEffort.setDouble(shooterFFEffort);

        shooterTargetRPM = SmartDashboard.getNumber("Set Shooter RPM", 0.0);
    } 
    
    private void runShooterPID() {
        /**Determines how much motor output must be given to reach RPM target
         * If target RPM is not equal to 0, calculate the deviation and add output
         * voltage until it reaches the target
        */
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
