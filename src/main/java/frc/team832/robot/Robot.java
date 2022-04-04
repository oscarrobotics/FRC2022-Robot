package frc.team832.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.robot.subsystems.ClimbSubsystem;
import frc.team832.robot.subsystems.ConveyorSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer = new RobotContainer();

  private final Compressor compressor = m_robotContainer.compressor;
  private final DrivetrainSubsystem drivetrain = m_robotContainer.drivetrain;
  private final IntakeSubsystem intake = m_robotContainer.intake;
  private final ConveyorSubsystem conveyor = m_robotContainer.conveyor;
  private final ShooterSubsystem shooter = m_robotContainer.shooter;
  private final ClimbSubsystem climb = m_robotContainer.climb;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    if (RobotBase.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    CameraServer.startAutomaticCapture();

    climb.setIsPID(false);
    climb.zeroClimb();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    intake.periodic();
    conveyor.periodic();
    shooter.periodic();

    if (m_robotContainer.userButton.get()) {
      m_robotContainer.setAutoPose();
    }

    SmartDashboard.putNumber("Storage PSI", compressor.getPressure());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // drivetrain.stop();
    CommandScheduler.getInstance().cancelAll();
    drivetrain.setNeutralMode(NeutralMode.kCoast);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    CommandScheduler.getInstance().cancelAll();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    drivetrain.setNeutralMode(NeutralMode.kBrake);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    CommandScheduler.getInstance().cancelAll();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    drivetrain.setNeutralMode(NeutralMode.kBrake);

    climb.setIsPID(false);
    climb.zeroClimb();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    drivetrain.setNeutralMode(NeutralMode.kCoast);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    drivetrain.teleopArcadeDrive(0.7, 0, 1);
  }
}
