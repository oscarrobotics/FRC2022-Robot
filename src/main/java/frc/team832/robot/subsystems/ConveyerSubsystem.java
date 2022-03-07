package frc.team832.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.robot.Constants.ConveyerConstants;

public class ConveyerSubsystem extends SubsystemBase{
    private final CANTalonFX conveyerMotor = new CANTalonFX(ConveyerConstants.CONVEYER_MOTOR_TALON_ID);

    private PIDController conveyerPID = new PIDController(ConveyerConstants.KP, 0, 0);
    private final SimpleMotorFeedforward feedforward = ConveyerConstants.FEEDFORWARD;
    
    public double conveyerTargetRPM, conveyerActualRPM, conveyerPIDEffort, conveyerFFEffort;

    private final NetworkTableEntry dash_conveyerTargetRPM, dash_conveyerActualRPM, dash_conveyerMotorRPM, dash_conveyerFFEffort, dash_conveyerPIDEffort;

    /** Creates a new ConveyerSubsytem **/
    public ConveyerSubsystem() {
        DashboardManager.addTab(this);
        SmartDashboard.putNumber("Set Conveyer RPM", 0.0);

        conveyerMotor.limitInputCurrent(ConveyerConstants.CURRENT_LIMIT);

        dash_conveyerTargetRPM = DashboardManager.addTabItem(this, "Conveyer Target RPM", 0.0);
        dash_conveyerActualRPM = DashboardManager.addTabItem(this, "Conveyer Actual RPM", 0.0);
        dash_conveyerMotorRPM = DashboardManager.addTabItem(this, "Conveyer Motor RPM", 0.0);
        dash_conveyerPIDEffort = DashboardManager.addTabItem(this, "Conveyer PID Effort", 0.0);
        dash_conveyerFFEffort = DashboardManager.addTabItem(this,  "Conveyer FF Effort", 0.0);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("conveyerActualRPM", conveyerMotor.getSensorVelocity());
        // SmartDashboard.putNumber("conveyerTargetRPM", conveyerTargetRPM);
        
        updateControlLoops();
        updateDashboardData();
    }

    public void updateControlLoops() {
        // runConveyerPID();
    }

    private void updateDashboardData() {
        dash_conveyerTargetRPM.setDouble(conveyerTargetRPM);
        dash_conveyerActualRPM.setDouble(conveyerMotor.getSensorVelocity());
        dash_conveyerPIDEffort.setDouble(conveyerPIDEffort);
        dash_conveyerFFEffort.setDouble(conveyerFFEffort);

        conveyerTargetRPM = SmartDashboard.getNumber("Set Conveyer RPM", 0.0);
    }
    
    private void runConveyerPID() {
        conveyerActualRPM = conveyerMotor.getSensorVelocity();
        
        if (conveyerTargetRPM != 0) {
            conveyerFFEffort = feedforward.calculate(conveyerTargetRPM) / 12.0;
            conveyerPIDEffort = conveyerPID.calculate(conveyerActualRPM, conveyerTargetRPM) / 12.0;
        } else {
            conveyerFFEffort = 0;
            conveyerPIDEffort = 0;
        }
      
        conveyerMotor.set(conveyerPIDEffort + conveyerFFEffort);
    }

    public void setRPM(double targetRPM) {
        conveyerTargetRPM = targetRPM;
    }

    public void setPower(double power) {
        conveyerMotor.set(power);
    }

    public void idleConveyer() {
        conveyerMotor.set(0);
    }
}
