package frc.team832.robot;

import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
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
import frc.team832.robot.commands.AcceptBallCommand;
import frc.team832.robot.commands.QueueBallCommand;
import frc.team832.robot.commands.RejectBallCommand;
import frc.team832.robot.commands.ShootBallCmd;
import frc.team832.robot.commands.ShootBallVisionCmd;
// import frc.team832.robot.commands.*;
// import frc.team832.robot.commands.Climb.*;
import frc.team832.robot.commands.AutonomousCommands.*;
import frc.team832.robot.commands.Climb.ExtendClimbCommand;
import frc.team832.robot.commands.Climb.PivotClimbCommand;
import frc.team832.robot.commands.Climb.RetractClimbCommand;
import frc.team832.robot.commands.Climb.StraightenClimbCommand;
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
  public static final PhotonCamera gloworm = new PhotonCamera("gloworm");

  /** Subsystems **/
  public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(gloworm);
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final ConveyerSubsystem conveyer = new ConveyerSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem(gloworm);
  public final ClimbSubsystem climb = new ClimbSubsystem();
   
  /** HID Controllers **/
  private final CommandXboxController m_xboxCtrl = new CommandXboxController(0);
  private final StratComInterface stratComInterface = new StratComInterface(1);
  public final Trigger userButton = new Trigger(RobotController::getUserButton);
  private final GenericHID keyboard = new GenericHID(2);

  /** Autonomous Selector **/
  public final AutonomousSelector autoSelector = new AutonomousSelector();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    LiveWindow.disableAllTelemetry();
    PortForwarder.add(5800, "gloworm.local", 5800);
    PortForwarder.add(5800, "gloworm.local", 1181);
    PortForwarder.add(5800, "gloworm.local", 1182);

    var zeroHeadingRot = Rotation2d.fromDegrees(0);

    // var tarmacTestPath = PathHelper.generatePath(FieldConstants.RightOuterTarmacCorner, new Pose2d(1.5, 1.5, Rotation2d.fromDegrees(180 + 45)), DrivetrainConstants.CALM_TRAJCONFIG);
    var trajPoses = List.of(new Pose2d(0, 0, zeroHeadingRot), new Pose2d(0.75, 0, zeroHeadingRot), new Pose2d(3, 0, zeroHeadingRot));
    var twoBallPath = TrajectoryGenerator.generateTrajectory(trajPoses, DrivetrainConstants.CALM_TRAJCONFIG);
    var twoBallTestCmd = drivetrain.getTrajectoryCommand(twoBallPath);
    // var tarmacTestCmd = drivetrain.getTrajectoryCommand(tarmacTestPath);
    // autoSelector.addDefaultAutonomous("PathTest", FieldConstants.RightOuterTarmacCorner, tarmacTestCmd);
    autoSelector.addAutonomous("0 Cargo Auto", new BasicAutoCmd(drivetrain));
    autoSelector.addAutonomous("1 Cargo Auto", new OneCargoHighAutoCmd(drivetrain, intake, conveyer, shooter));
    autoSelector.addDefaultAutonomous("2 Cargo Auto", new TwoCargoAutoCmd(drivetrain, intake, conveyer, shooter));
    autoSelector.addAutonomous("2 Cargo Path Test", twoBallPath, twoBallTestCmd);
    var threeCargoAutoCmd = new ThreeCargoAutoCmd(drivetrain, intake, conveyer, shooter);
    autoSelector.addAutonomous("3 Cargo Auto", threeCargoAutoCmd.initialPath.getInitialPose(), threeCargoAutoCmd);

    var arcadeDriveCommand = new RunEndCommand(() -> {
        drivetrain.teleopArcadeDrive(
          -m_xboxCtrl.getLeftY(),
          -m_xboxCtrl.getRightX(), 
          1
        );
      },
      drivetrain::stop, drivetrain).withName("ArcadeDriveCommand");

    // var tankDriveCommand = new RunEndCommand(() -> {
    //     drivetrain.teleopTankDrive(
    //       -m_xboxCtrl.getRightY(),
    //       -m_xboxCtrl.getRightX(), 
    //       2
    //     );
    //   },
    //   drivetrain::stop, drivetrain).withName("ArcadeDriveCommand");

    drivetrain.setDefaultCommand(arcadeDriveCommand);

    // configTestingCommands();
    // configSimTestingCommands();
    configOperatorCommands();
  }

  public void configOperatorCommands() {
    m_xboxCtrl.b().whileHeld(drivetrain.getTargetingCommand(() -> -m_xboxCtrl.getLeftY()));
    
    stratComInterface.arcadeBlackRight().whileHeld(new AcceptBallCommand(intake, shooter, conveyer)).whenReleased(new QueueBallCommand(conveyer, shooter));
    stratComInterface.arcadeWhiteRight().whileHeld(new RejectBallCommand(intake, conveyer));
    stratComInterface.arcadeBlackLeft().whileHeld(new ShootBallVisionCmd(conveyer, shooter));
    stratComInterface.arcadeWhiteLeft().whileHeld(new ShootBallCmd(conveyer, shooter, () -> ShooterConstants.FRONT_RPM_LOW_FENDER, () -> ShooterConstants.REAR_RPM_LOW_FENDER, true));

    stratComInterface.scSideTop().whileHeld(new ShootBallCmd(conveyer, shooter, () -> ShooterConstants.FRONT_RPM_TARMAC, () -> ShooterConstants.REAR_RPM_TARMAC, false));
    stratComInterface.scSideMid().whileHeld(new ShootBallCmd(conveyer, shooter, () -> ShooterConstants.FRONT_RPM_HIGH_FENDER, () -> ShooterConstants.REAR_RPM_HIGH_FENDER, false));


    stratComInterface.sc1().whileHeld(new RunEndCommand(() -> {
        climb.setLeftPow(-.45);
        climb.setRightPow(-.45);
      }, 
      () -> {
        climb.setLeftPow(0);
        climb.setRightPow(0);
      }, climb));

      stratComInterface.sc4().whileHeld(new RunEndCommand(() -> {
        climb.setLeftPow(.65);
        climb.setRightPow(.65);
      }, 
      () -> {
        climb.setLeftPow(0);
        climb.setRightPow(0);
      }, climb));

      stratComInterface.sc2().whenPressed(new PivotClimbCommand(climb));
      stratComInterface.sc5().whenReleased(new StraightenClimbCommand(climb));

    // auto climb
    // stratComInterface.sc1().whenHeld(new ExtendClimbCommand(climb));
    // stratComInterface.sc4().whenHeld(new RetractClimbCommand(climb));
    // stratComInterface.sc2().whenHeld(new PivotClimbCommand(climb));
    // stratComInterface.sc5().whenHeld(new StraightenClimbCommand(climb));
  }

  public void configSimTestingCommands() {
    // var ramseteTestCommand = drivetrain.getTrajectoryCommand(DrivetrainConstants.test3MeterForwardTraj).withName("RamseteTestCommand")
    // .andThen(drivetrain::stop);

    // m_xboxCtrl.b()
    //   .whenPressed(ramseteTestCommand);

    // m_xboxCtrl.a().whileHeld(drivetrain.getTargetingCommand(() -> -m_xboxCtrl.getLeftY()));

    // m_xboxCtrl.a().whileHeld(drivetrain.getTrajectoryCommand(drivetrain.initializePaths(DrivetrainConstants.THREE_BALL_AUTO_PATH)));
  }

  public void configTestingCommands() {
    // track target command
    m_xboxCtrl.b().whileHeld(drivetrain.getTargetingCommand(() -> -m_xboxCtrl.getLeftY()));

    // map sliders to each flywheel and turn on shooter with single toggle
    stratComInterface.singleToggle().whileHeld(new RunEndCommand(
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

    // stratComInterface.singleToggle().whileHeld(new RunEndCommand(
    //     () -> {
    //       shooter.setVisionRpms();
    //     },
    //     () -> {
    //       shooter.idleShooter();
    //     }, 
    //     shooter
    //   )
    // );

    // shooting with vision
    stratComInterface.arcadeBlackLeft().whileHeld(new ShootBallVisionCmd(conveyer, shooter));
      
    // stratComInterface.scSideTop().whileHeld(ne);
      
    // intake
    stratComInterface.arcadeBlackRight().whileHeld(new RunEndCommand(
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

    // feed shooter (spin conveyer forward)
    stratComInterface.arcadeWhiteRight().whileHeld(new RunEndCommand(
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
    
    // reverse conveyer (to feed from shooter)
    stratComInterface.arcadeWhiteLeft().whileHeld(new RunEndCommand(
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
