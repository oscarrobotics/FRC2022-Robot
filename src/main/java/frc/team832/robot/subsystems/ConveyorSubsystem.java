package frc.team832.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase{
    /**physical devices */
    private final CANTalonFX conveyorMotor = new CANTalonFX(ConveyorConstants.CONVEYOR_MOTOR_TALON_ID);
    private final DigitalInput topCargoSensor = new DigitalInput(0);

    //assigns P value of PID + feed forward to conveyor
    private PIDController conveyorPID = new PIDController(ConveyorConstants.KP, 0, 0);
    private final SimpleMotorFeedforward feedforward = ConveyorConstants.FEEDFORWARD;
    
    public double conveyorTargetRPM, conveyorActualRPM, conveyorPIDEffort, conveyorFFEffort;

    private final NetworkTableEntry dash_conveyorTargetRPM, dash_conveyorActualRPM, dash_conveyorMotorRPM, dash_conveyorFFEffort, dash_conveyorPIDEffort;
    private final NetworkTableEntry dash_bottomSensorR, dash_bottomSensorG, dash_bottomSensorB, dash_bottomSensorProximity;

    /** Creates a new ConveyorSubsytem **/
    public ConveyorSubsystem() {
        DashboardManager.addTab(this);
        SmartDashboard.putNumber("Set Conveyor RPM", 0.0);

        conveyorMotor.setNeutralMode(NeutralMode.kBrake);

        conveyorMotor.limitInputCurrent(ConveyorConstants.CURRENT_LIMIT);

        dash_conveyorTargetRPM = DashboardManager.addTabItem(this, "Conveyor Target RPM", 0.0);
        dash_conveyorActualRPM = DashboardManager.addTabItem(this, "Conveyor Actual RPM", 0.0);
        dash_conveyorMotorRPM = DashboardManager.addTabItem(this, "Conveyor Motor RPM", 0.0);
        dash_conveyorPIDEffort = DashboardManager.addTabItem(this, "Conveyor PID Effort", 0.0);
        dash_conveyorFFEffort = DashboardManager.addTabItem(this,  "Conveyor FF Effort", 0.0);
        dash_bottomSensorR = DashboardManager.addTabItem(this, "Bottom Cargo Sensor R", 0.0);
        dash_bottomSensorG = DashboardManager.addTabItem(this, "Bottom Cargo Sensor G", 0.0);
        dash_bottomSensorB = DashboardManager.addTabItem(this, "Bottom Cargo Sensor B", 0.0);
        dash_bottomSensorProximity = DashboardManager.addTabItem(this, "Bottom Cargo Sensor Proximity", 0.0);
    }

    @Override
    public void periodic() {     
        updateControlLoops();
        updateDashboardData();
    }

    public void updateControlLoops() {
        // runConveyorPID();
    }

    private void updateDashboardData() {
        dash_conveyorTargetRPM.setDouble(conveyorTargetRPM);
        dash_conveyorActualRPM.setDouble(conveyorMotor.getSensorVelocity());
        dash_conveyorPIDEffort.setDouble(conveyorPIDEffort);
        dash_conveyorFFEffort.setDouble(conveyorFFEffort);

        // dash_bottomSensorR.setDouble();
        // dash_bottomSensorG.setDouble();
        // dash_bottomSensorB.setDouble();
        // dash_bottomSensorProximity.setDouble();

        conveyorTargetRPM = SmartDashboard.getNumber("Set Conveyor RPM", 0.0);
        SmartDashboard.putBoolean("TopCargoSensor", topIsCargo());
    }
    
    private void runConveyorPID() {
        //Determines how much output must be given to reach RPM target
        conveyorActualRPM = conveyorMotor.getSensorVelocity();
        
        if (conveyorTargetRPM != 0) {
            conveyorFFEffort = feedforward.calculate(conveyorTargetRPM) / 12.0;
            conveyorPIDEffort = conveyorPID.calculate(conveyorActualRPM, conveyorTargetRPM) / 12.0;
        } else {
            conveyorFFEffort = 0;
            conveyorPIDEffort = 0;
        }
      
        //adds FF and PID together for accurate energy output
        conveyorMotor.set(conveyorPIDEffort + conveyorFFEffort);
    }

    public void setRPM(double targetRPM) {
        conveyorTargetRPM = targetRPM;
    }

    public void setPower(double power) {
        conveyorMotor.set(power);
    }

    public void idle() {
        conveyorMotor.set(0);
    }

    public boolean topIsCargo() {
        return topCargoSensor.get();
    }

    public boolean bottomIsCargo() {
        return false;
    }
}
