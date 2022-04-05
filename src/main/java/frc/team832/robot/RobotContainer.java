package frc.team832.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonVersion;

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
import frc.team832.lib.motion.PathHelper;
import frc.team832.lib.util.OscarMath;

import frc.team832.robot.Constants.*;
import frc.team832.robot.commands.*;
import frc.team832.robot.commands.AutonomousCommands.*;
import frc.team832.robot.commands.Climb.*;
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
  public final ConveyorSubsystem conveyor = new ConveyorSubsystem();
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
    PhotonCamera.setVersionCheckEnabled(false);
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
    autoSelector.addAutonomous("1 Cargo Auto", new OneCargoHighAutoCmd(drivetrain, intake, conveyor, shooter));
    autoSelector.addDefaultAutonomous("2 Cargo Auto", new TwoCargoAutoCmd(drivetrain, intake, conveyor, shooter));
    autoSelector.addAutonomous("2 Cargo Path Test", twoBallPath, twoBallTestCmd);
    var threeCargoAutoCmd = new ThreeCargoAutoCmd(drivetrain, intake, conveyor, shooter);
    autoSelector.addAutonomous("3 Cargo Auto", threeCargoAutoCmd.initialPath.getInitialPose(), threeCargoAutoCmd);

    var arcadeDriveCommand = new RunEndCommand(() -> {
        drivetrain.teleopArcadeDrive(
          -m_xboxCtrl.getLeftY(),
          -m_xboxCtrl.getRightX(), 
          2
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

    // configOperatorCommands();
    configTestingCommands();
    }

  public void configOperatorCommands() {
    /**
     * climb same
     * sc3 = aut climb
     * intake reject same
     * sc side bot = low fender
     * distance low = wihte left
     * 
     */

    m_xboxCtrl.b().whileHeld(drivetrain.getTargetingCommand(() -> -m_xboxCtrl.getLeftY()));
    
    stratComInterface.arcadeBlackRight().whileHeld(new AcceptBallCommand(intake, shooter, conveyor)).whenReleased(new QueueBallCommand(conveyor, shooter));
    stratComInterface.arcadeWhiteRight().whileHeld(new RejectBallCommand(intake, conveyor));
    stratComInterface.arcadeBlackLeft().whileHeld(new ShootBallVisionCmd(conveyor, shooter));

    stratComInterface.scSideBot().whenPressed(new ShootBallCmd(
      conveyor, shooter, ShooterConstants.FRONT_RPM_LOW_FENDER, ShooterConstants.REAR_RPM_LOW_FENDER, true));
    stratComInterface.scSideTop().whileHeld(new ShootBallCmd(
      conveyor, shooter, ShooterConstants.FRONT_RPM_TARMAC, ShooterConstants.REAR_RPM_TARMAC));
    stratComInterface.scSideMid().whileHeld(new ShootBallCmd(
      conveyor, shooter, ShooterConstants.FRONT_RPM_HIGH_FENDER, ShooterConstants.REAR_RPM_HIGH_FENDER));


    // stratComInterface.sc1().whileHeld(new RunEndCommand(
    //   () -> {
    //     // climb.setLeftPow(1);
    //     // climb.setRightPow(1);
    //     climb.setPower(1);
    //   }, 
    //   () -> {
    //     // climb.setLeftPow(0);
    //     // climb.setRightPow(0);
    //     climb.setPower(0);
    //   }, climb));

    //   stratComInterface.sc4().whileHeld(new RunEndCommand(() -> {
    //     // climb.setLeftPow(-.70);
    //     // climb.setRightPow(-.70);
    //     climb.setPower(-.70);
    //   }, 
    //   () -> {
    //     // climb.setLeftPow(0);
    //     // climb.setRightPow(0);
    //     climb.setPower(0);
    //   }, climb)
    // );

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
    configClimbTestCmds();
    // configShootTestCmds();
  }

  public void configClimbTestCmds() {
    // zero climb for testing
    stratComInterface.arcadeBlackLeft().whenPressed(() -> climb.zeroClimb());

    // // climb via power to soft limit
    // stratComInterface.sc3().whenPressed(
    //   () -> {
    //     climb.setIsPID(true); 
    //     climb.setTargetPosition(ClimbConstants.LEFT_MAX_EXTEND_POS, ClimbConstants.RIGHT_MAX_EXTEND_POS);
    //   }, climb);
    
    // // climb via pid to 0
    // stratComInterface.sc6().whenPressed(
    //   () -> { 
    //     climb.setIsPID(true);
    //     climb.setTargetPosition(0, 0);
    //   }, climb);

    // test auto climb cmd
    stratComInterface.arcadeBlackRight().whenPressed(new AutoToNextBarCmd(climb));

    // climb via pid (EXTEND TARGET CMD) to next bar target
    var extendCmd = new PositionClimbCommand(climb, ClimbConstants.LEFT_TO_NEXT_BAR_TARGET, ClimbConstants.RIGHT_TO_NEXT_BAR_TARGET).withName("ExtendCmd");
    
    stratComInterface.sc2().whenPressed(extendCmd);
    // stratComInterface.arcadeWhiteLeft().whenPressed(nextBarCmd);

    // climb via pid (EXTEND TARGET CMD) to free hook target
    stratComInterface.sc3().whenPressed(new PositionClimbCommand(climb, ClimbConstants.LEFT_FREE_HOOK_TARGET, ClimbConstants.RIGHT_FREE_HOOK_TARGET));

    // climb via pid (EXTEND TARGET CMD) to min (enc pos = 10)
    stratComInterface.sc6().whenPressed(new PositionClimbCommand(climb, ClimbConstants.RETRACT_TARGET, ClimbConstants.RETRACT_TARGET));

    // climb up via power - will stop at limit
    stratComInterface.sc1().whileHeld(new RunEndCommand(
      () -> climb.setPower(.5, .5), 
      climb::idle,
      climb)
    );
    // climb down via power - no bottom soft limit
    stratComInterface.sc4().whileHeld(new RunEndCommand(
      () -> climb.setPower(-.5, -.5), 
      climb::idle,
      climb)
    );

    // control climb up down via power - toggle up = climb up, slider pos = power
    stratComInterface.doubleToggleUp().whileHeld(new RunEndCommand(
      () -> {
        double rightPow = OscarMath.map(stratComInterface.getRightSlider(), -1, 1, 0, 1);
        double leftPow = OscarMath.map(stratComInterface.getLeftSlider(), -1, 1, 0, 1);
        SmartDashboard.putNumber("Right Climb Velocity", climb.getRightVelocity());
        SmartDashboard.putNumber("Left Climb Velocity", climb.getLeftVelocity());
        SmartDashboard.putNumber("Right Climb Position", climb.getRightPosition());
        SmartDashboard.putNumber("Left Climb Position", climb.getLeftPosition());
        climb.setPower(leftPow, rightPow);
      },
      () -> {
        climb.idle();
      }, 
      climb
    ).withName("SliderClimbExtendCmd"));
    stratComInterface.doubleToggleDown().whileHeld(new RunEndCommand(
      () -> {
        double rightPow = OscarMath.map(stratComInterface.getRightSlider(), -1, 1, 0, 1);
        double leftPow = OscarMath.map(stratComInterface.getLeftSlider(), -1, 1, 0, 1);
        climb.setPower(-leftPow, -rightPow);
      },
      () -> {
        climb.idle();
      }, 
      climb
    ));
  }

  public void configShootTestCmds() {
    // track target command
    m_xboxCtrl.b().whileHeld(drivetrain.getTargetingCommand(() -> -m_xboxCtrl.getLeftY()));

    // map sliders to each flywheel and turn on shooter with single toggle
    stratComInterface.singleToggle().whileHeld(new RunEndCommand(
        () -> {
          double topRpm = OscarMath.map(stratComInterface.getLeftSlider(), -1, 1, 0, 6380);
          double botRpm = OscarMath.map(stratComInterface.getRightSlider(), -1, 1, 0, 6380);
          SmartDashboard.putNumber("Rear Flywheel Target RPM", topRpm);
          SmartDashboard.putNumber("Front Flywheel Target RPM", botRpm);
          shooter.setRPM(botRpm, topRpm);
        },
        () -> {
          shooter.idle();
        }, 
        shooter
      )
    );

    // hood control
    if (stratComInterface.doubleToggleUp().get()) {
      shooter.extendHood();
    } else {
      shooter.retractHood();
    }

    // shooting with vision
    stratComInterface.arcadeBlackLeft().whileHeld(new ShootBallVisionCmd(conveyor, shooter));
          
    // regular intake
    stratComInterface.arcadeBlackRight().whileHeld(new RunEndCommand(
        () -> {
          intake.extendIntake();
          intake.setPower(IntakeConstants.INTAKE_POWER);
          conveyor.setPower(ConveyorConstants.FEEDING_POWER);
          shooter.setRPM(0, -2500);
        }, 
        () -> {
          intake.idle();
          conveyor.idle();
        }, 
        intake
      )
    );

    // feed shooter (spin conveyor forward)
    stratComInterface.arcadeWhiteRight().whileHeld(new RunEndCommand(
        () -> {
          conveyor.setPower(ConveyorConstants.FEEDING_POWER);
        }, 
        () -> {
          // intake.idle();
          conveyor.idle();
        }, 
        conveyor
      )
    );
    
    // intake from shooter - reverse conveyer and both flywheels
    stratComInterface.arcadeWhiteLeft().whileHeld(new RunEndCommand(
        () -> {
          conveyor.setPower(-ConveyorConstants.FEEDING_POWER);
          shooter.setRPM(-1000, -1000);
        }, 
        () -> {
          // intake.idle();
          conveyor.idle();
          shooter.idle();
        }, 
        conveyor
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
