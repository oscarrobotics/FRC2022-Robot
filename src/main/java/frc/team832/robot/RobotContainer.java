package frc.team832.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandXboxController;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team832.lib.driverinput.controllers.StratComInterface;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.commands.AcceptBallCommand;
import frc.team832.robot.commands.RejectBallCommand;
import frc.team832.robot.commands.ShootBallCommand;
import frc.team832.robot.commands.AutonomousCommands.BasicAutoCommand;
import frc.team832.robot.commands.AutonomousCommands.OneCargoAutoCommand;
import frc.team832.robot.commands.AutonomousCommands.ThreeCargoAutoCommand;
import frc.team832.robot.commands.AutonomousCommands.TwoCargoAutoCommand;
import frc.team832.robot.commands.Climb.ExtendClimbCommand;
import frc.team832.robot.commands.Climb.PivotClimbCommand;
import frc.team832.robot.commands.Climb.RetractClimbCommand;
import frc.team832.robot.commands.Climb.StraightenClimbCommand;
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
  // public final PowerDistribution powerDist = new PowerDistribution(Constants.RPD_CAN_ID, ModuleType.kRev);
  // public final Compressor compressor = new Compressor(Constants.RPH_CAN_ID, PneumaticsModuleType.REVPH);

  /** Subsystems **/
  public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final ConveyerSubsystem conveyer = new ConveyerSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  // public final ClimbSubsystem climb = new ClimbSubsystem();
  
  /** HID Controllers **/
  private final CommandXboxController m_xboxCtrl = new CommandXboxController(0);
  // public final StratComInterface stratComInterface = new StratComInterface(1);

  /** Sendable Chooser object **/
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    LiveWindow.disableAllTelemetry();

    autoChooser.setDefaultOption("0 Cargo Auto", new BasicAutoCommand(drivetrain));
    autoChooser.addOption("1 Cargo Auto", new OneCargoAutoCommand(drivetrain, intake, conveyer, shooter));
    autoChooser.addOption("2 Cargo Auto", new TwoCargoAutoCommand(drivetrain, intake, conveyer, shooter));
    // autoChooser.addOption("3 Cargo Auto", new ThreeCargoAutoCommand(drivetrain, intake, conveyer, shooter));
    SmartDashboard.putData(autoChooser);

    // drivetrain.setDefaultCommand(new RunCommand(() -> {
    //   // var shouldTurnInPlace = m_xboxCtrl.rightStick().getAsBoolean();
    //   drivetrain.teleopArcadeDrive(
    //     m_xboxCtrl.getLeftY()*.5,
    //     -m_xboxCtrl.getRightX()*.5,
    //     1.2);
    // }, drivetrain));

    configOperatorCommands();
  }

  public void configOperatorCommands() {
    // Testing commands
    m_xboxCtrl.a()
      .whileHeld(new AcceptBallCommand(intake, shooter, conveyer))
      .whileHeld(new StartEndCommand(
        () -> {
          conveyer.setPower(0.5);
        }
      , () -> {
        conveyer.setPower(0);
      }, conveyer));

    m_xboxCtrl.b().whileHeld(new RejectBallCommand(intake, conveyer));

    // stratComInterface.arcadeBlackRight().whileHeld(new AcceptBallCommand(intake));
    // stratComInterface.arcadeWhiteRight().whileHeld(new RejectBallCommand(intake, conveyer));

    // stratComInterface.arcadeBlackLeft().whileHeld(new ShootBallCommand(conveyer, shooter));

    // stratComInterface.sc1().whenHeld(new ExtendClimbCommand(climb));
    // stratComInterface.sc4().whenHeld(new RetractClimbCommand(climb));
    // stratComInterface.sc2().whenHeld(new PivotClimbCommand(climb));
    // stratComInterface.sc5().whenHeld(new StraightenClimbCommand(climb));
  }

  public void configTestingCommands() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Returns command selected by SendableChooser autoCommand to run in autonomous
    return autoChooser.getSelected();
  }
}
