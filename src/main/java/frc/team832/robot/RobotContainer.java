package frc.team832.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team832.robot.subsystems.ClimbSubsystem;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.DrivetrainSubsystem;
import frc.team832.robot.subsystems.IntakeSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /** Control system objects **/
  public final PowerDistribution powerDist = new PowerDistribution(Constants.RPD_CAN_ID, ModuleType.kRev);
  public final Compressor compressor = new Compressor(Constants.RPH_CAN_ID, PneumaticsModuleType.REVPH);

  /** Subsystems **/
  public final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  // public final IntakeSubsystem intake = new IntakeSubsystem();
  // public final ConveyerSubsystem conveyer = new ConveyerSubsystem();
  // public final ShooterSubsystem shooter = new ShooterSubsystem();
  // public final ClimbSubsystem climber = new ClimbSubsystem();
  
  /** HID Controllers **/
  private final CommandXboxController m_xboxCtrl = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    LiveWindow.disableAllTelemetry();
    drivetrainSubsystem.setDefaultCommand(new RunCommand(() -> {
      var shouldTurnInPlace = m_xboxCtrl.rightStick().getAsBoolean();
      drivetrainSubsystem.teleopCurvatureDrive(
        m_xboxCtrl.getLeftX(),
        m_xboxCtrl.getRightY(),
        shouldTurnInPlace,
        1.2);
    }, drivetrainSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PrintCommand("Autonomous!");
  }
}
