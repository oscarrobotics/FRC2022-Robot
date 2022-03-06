package frc.team832.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.motorcontrol.vendor.CANTalonFX;
import frc.team832.robot.Constants.ConveyerConstants;

import static frc.team832.robot.Constants.ConveyerConstants.*;

public class ConveyerSubsystem extends SubsystemBase{
    private final CANTalonFX conveyerMotor = new CANTalonFX(CONVEYER_MOTOR_TALON_ID);

    //Instantiate PID Controller and FeedFoward 
    private PIDController conveyerPID = new PIDController(ConveyerConstants.KP, 0, 0);
    private final SimpleMotorFeedforward feedforward = ConveyerConstants.FEEDFORWARD;
    
    //Instantiate conveyer RPM + feedfoward variables
    public double conveyerTargetRPM, conveyerActualRPM, conveyerPIDEffort, conveyerFFEffort;


    /** Creates a new ConveyerSubsytem **/
    public ConveyerSubsystem() {
        conveyerMotor.limitInputCurrent(CURRENT_LIMIT);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("conveyerActualRPM", conveyerMotor.getSensorVelocity());
        SmartDashboard.putNumber("conveyerTargetRPM", conveyerTargetRPM);
        
        updateControlLoops();
    }

    public void updateControlLoops() {
        runConveyerPID();
    }
    
    public void runConveyerPID() {
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
