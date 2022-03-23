package frc.team832.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team832.AutonomousSelector;
import frc.team832.AutonomousSelector.AutonomousMode;
import frc.team832.lib.driverinput.controllers.StratComInterface;
// import frc.team832.lib.util.OscarMath;
import frc.team832.lib.motion.PathHelper;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants.*;
import frc.team832.robot.commands.ShootBallCommand;
// import frc.team832.robot.commands.*;
// import frc.team832.robot.commands.Climb.*;
import frc.team832.robot.commands.AutonomousCommands.*;
import frc.team832.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /** Control system objects **/
  // public final PowerDistribution powerDist = new PowerDistribution(Constants.RPD_CAN_ID, ModuleType.kRev);
  public final Compressor compressor = new Compressor(Constants.RPH_CAN_ID, PneumaticsModuleType.REVPH);

  /** Vision Camera**/
  public static final PhotonCamera camera = new PhotonCamera("camera");

  /** Subsystems **/
  public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(camera);
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final ConveyerSubsystem conveyer = new ConveyerSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem(camera);
  public final ClimbSubsystem climb = new ClimbSubsystem();
   
  /** HID Controllers **/
  private final CommandXboxController m_xboxCtrl = new CommandXboxController(0);
  private final StratComInterface stratComInterface = new StratComInterface(1);
  public final Trigger userButton = new Trigger(RobotController::getUserButton);

  // /** Sendable Chooser object **/
  // private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public final AutonomousSelector autoSelector = new AutonomousSelector();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    LiveWindow.disableAllTelemetry();

    autoSelector.addAutonomous("0 Cargo Auto", new BasicAutoCommand(drivetrain));
    autoSelector.addAutonomous("1 Cargo Auto", new OneCargoAutoCommand(drivetrain, intake, conveyer, shooter));
    autoSelector.addAutonomous("2 Cargo Auto", new TwoCargoAutoCommand(drivetrain, intake, conveyer, shooter));
    autoSelector.addAutonomous("3 Cargo Auto", new ThreeCargoAutoCommand(drivetrain, intake, conveyer, shooter));

    var tarmacTestPath = PathHelper.generatePath(FieldConstants.RightOuterTarmacCorner, new Pose2d(1.5, 1.5, Rotation2d.fromDegrees(180 + 45)), DrivetrainConstants.CALM_TRAJCONFIG);
    var tarmacTestCmd = drivetrain.getTrajectoryCommand(tarmacTestPath);
    autoSelector.addDefaultAutonomous("PathTest", FieldConstants.RightOuterTarmacCorner, tarmacTestCmd);

    // drivetrain.setDefaultCommand(new RunCommand(() -> {
    //   // var shouldTurnInPlace = m_xboxCtrl.rightStick().getAsBoolean();
    //   drivetrain.teleopTankDrive(
    //     -m_xboxCtrl.getRightY() * 1,
    //     -m_xboxCtrl.getLeftY() * 1,
    //     2);
    // }, drivetrain));

    var arcadeDriveCommand = new RunEndCommand(
      () -> {
        drivetrain.teleopArcadeDrive(
          -m_xboxCtrl.getLeftY(),
          m_xboxCtrl.getRightX(), 
          2
        );
      },
      drivetrain::stop,
      drivetrain)
    .withName("ArcadeDriveCommand");

    var tankDriveCommand = new RunEndCommand(
      () -> {
        drivetrain.teleopTankDrive(
          -m_xboxCtrl.getRightY(),
          -m_xboxCtrl.getRightX(), 
          2
        );
      },
      drivetrain::stop,
      drivetrain)
    .withName("ArcadeDriveCommand");

    drivetrain.setDefaultCommand(arcadeDriveCommand);
    configTestingCommands();
    // configSimTestingCommands();
  }

  public void configOperatorCommands() {
    // BUTTON BINDINGS
    // m_xboxCtrl.rightBumper().whileHeld(new AcceptBallCommand(intake, shooter, conveyer)).whenReleased(new QueueBallCommand(conveyer, shooter));
    
    // stratComInterface.arcadeBlackRight().whileHeld(new AcceptBallCommand(intake, shooter, conveyer)).whenReleased(new QueueBallCommand(conveyer, shooter));
    // stratComInterface.arcadeWhiteRight().whileHeld(new RejectBallCommand(intake, conveyer));

    // stratComInterface.arcadeBlackLeft().whenPressed(new ShootBallCommand(conveyer, shooter));

    // stratComInterface.sc1().whileHeld(new RunEndCommand(() -> {
    //     climb.setLeftPow(-.35);
    //     climb.setRightPow(-.35);
    //   }, 
    //   () -> {
    //     climb.setLeftPow(0);
    //     climb.setRightPow(0);
    //   }, climb));

    //   stratComInterface.sc4().whileHeld(new RunEndCommand(() -> {
    //     climb.setLeftPow(.65);
    //     climb.setRightPow(.65);
    //   }, 
    //   () -> {
    //     climb.setLeftPow(0);
    //     climb.setRightPow(0);
    //   }, climb));

    //   stratComInterface.sc2().whenPressed(new PivotClimbCommand(climb));
    //   stratComInterface.sc5().whenReleased(new StraightenClimbCommand(climb));

    // auto climb
    // stratComInterface.sc1().whenHeld(new ExtendClimbCommand(climb));
    // stratComInterface.sc4().whenHeld(new RetractClimbCommand(climb));
    // stratComInterface.sc2().whenHeld(new PivotClimbCommand(climb));
    // stratComInterface.sc5().whenHeld(new StraightenClimbCommand(climb));

    // TEST CMDS
    // m_xboxCtrl.rightBumper().whileHeld(new AcceptBallCommand(intake, shooter, conveyer)).whenReleased(new QueueBallCommand(conveyer, shooter));
    // m_xboxCtrl.leftBumper().whenPressed(new ShootBallCommand(conveyer, shooter));
    
    // m_xboxCtrl.x().whileHeld(new RunEndCommand(() -> {climb.setLeftPow(.35);}, () -> {climb.setLeftPow(0);}, climb)); // E 
    // m_xboxCtrl.b().whileHeld(new RunEndCommand(() -> {climb.setLeftPow(-.35);}, () -> {climb.setLeftPow(0);}, climb));  // R
    // m_xboxCtrl.y().whileHeld(new RunEndCommand(() -> {climb.setRightPow(.35);}, () -> {climb.setRightPow(0);}, climb)); // E
    // m_xboxCtrl.a().whileHeld(new RunEndCommand(() -> {climb.setRightPow(-.35);}, () -> {climb.setRightPow(0);}, climb)); // R 
    
    // m_xboxCtrl.rightBumper().whenPressed(new PivotClimbCommand(climb)).whenReleased(new StraightenClimbCommand(climb));
  }
 
  public void configSimTestingCommands() {
    // m_xboxCtrl.a()
    //   .whenPressed(() -> shooter.setRPM(2600, 2600), shooter)
    //   .whenReleased(shooter::idleShooter, shooter);

    var ramseteTestCommand = drivetrain.getTrajectoryCommand(DrivetrainConstants.test3MeterForwardTraj).withName("RamseteTestCommand")
    .andThen(drivetrain::stop);

    m_xboxCtrl.b()
      .whenPressed(ramseteTestCommand);
  }

  public void configTestingCommands() {
    // map sliders to each flywheel
    stratComInterface.singleToggle().whileHeld(
      new RunEndCommand(
        () -> {
          double topRpm = OscarMath.map(stratComInterface.getLeftSlider(), -1, 1, 0, 6380);
          double botRpm = OscarMath.map(stratComInterface.getRightSlider(), -1, 1, 0, 6380);
          SmartDashboard.putNumber("Top Flywheel Target RPM", topRpm);
          SmartDashboard.putNumber("Bottom Flywheel Target RPM", botRpm);
          shooter.setRPM(botRpm, topRpm);
        },
        () -> {
          shooter.idleShooter();
        }, 
        shooter
      )
    );
    // DoubleSupplier leftSliderRpm = () -> OscarMath.map(stratComInterface.getLeftSlider(), -1, 1, 0, 6380);
    // DoubleSupplier rightSliderRpm = () -> OscarMath.map(stratComInterface.getRightSlider(), -1, 1, 0, 6380);
    // stratComInterface.singleToggle().whileHeld(new ShootBallCommand(conveyer, shooter, leftSliderRpm, rightSliderRpm));

    // // intake
    stratComInterface.arcadeBlackRight().whileHeld(
      new RunEndCommand(
        () -> {
          intake.extendIntake();
          intake.setPower(IntakeConstants.INTAKE_POWER);
          conveyer.setPower(ConveyerConstants.FEEDING_POWER);
          shooter.setRPM(0, -2500);
        }, 
        () -> {
          intake.idleIntake();
          conveyer.idleConveyer();
        }, 
        intake
      )
    );

    // // feed shooter
    stratComInterface.arcadeWhiteRight().whileHeld(
      new RunEndCommand(
        () -> {
          // intake.extendIntake();
          // intake.setPower(IntakeConstants.INTAKE_POWER);
          conveyer.setPower(ConveyerConstants.FEEDING_POWER);
        }, 
        () -> {
          // intake.idleIntake();
          conveyer.idleConveyer();
        }, 
        conveyer
      )
    );
    // // current sensing feed
    // // stratComInterface.arcadeWhiteRight().whenHeld(new FeedBallCommand(conveyer, shooter));


    // // reverse conveyer
    stratComInterface.arcadeWhiteLeft().whileHeld(
      new RunEndCommand(
        () -> {
          // intake.extendIntake();
          // intake.setPower(IntakeConstants.INTAKE_POWER);
          conveyer.setPower(-ConveyerConstants.FEEDING_POWER);
          shooter.setRPM(-1000, -1000);
        }, 
        () -> {
          // intake.idleIntake();
          conveyer.idleConveyer();
          shooter.idleShooter();
        }, 
        conveyer
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Returns command selected by SendableChooser autoCommand to run in autonomous
    return autoSelector.getSelectedAutonomous().autoCommand;
  }

  public void setAutoPose() {
    var selectedAuto = autoSelector.getSelectedAutonomous();
    System.out.println("USR BTN | Resetting Robot Pose to " + selectedAuto.startPose.toString());
    drivetrain.resetPose(selectedAuto.startPose);
    if (selectedAuto.path != null) {
      drivetrain.setCurrentField2dTrajectory(selectedAuto.path);
    }
  }
}
