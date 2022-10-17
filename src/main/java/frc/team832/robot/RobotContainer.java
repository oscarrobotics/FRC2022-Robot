package frc.team832.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.team832.AutonomousSelector;
import frc.team832.lib.driverinput.controllers.StratComInterface;
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

  /** Autonomous Selector **/
  public final AutonomousSelector autoSelector = new AutonomousSelector();

  private final SlewRateLimiter driveLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(3);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    PhotonCamera.setVersionCheckEnabled(false);
    LiveWindow.disableAllTelemetry();

    var halfMeterTraj = Constants.DrivetrainConstants.testHalfMeterForwardTraj;

    // autoSelector.addDefaultAutonomous("HalfMeterTest", halfMeterTraj, drivetrain.getTrajectoryCommand(halfMeterTraj));

    // var threeBallPath = PathPlanner.loadPath("3 Ball Auto", 2, 2);
    // var threeBallTestCmd = drivetrain.getTrajectoryCommand(threeBallPath);
    // autoSelector.addAutonomous("3 Ball Auto PathTest", threeBallPath, threeBallTestCmd);
    var twoBallAutoCmd = new TwoCargoAutoCmd(drivetrain, intake, conveyor, shooter);                           //two ball auto for shuffleboard
    autoSelector.addAutonomous("2 Cargo Auto", twoBallAutoCmd.initialPath, twoBallAutoCmd);
    // autoSelector.addAutonomous("3 Cargo Auto", new ThreeCargoAutoCmd(drivetrain, intake, conveyor, shooter));
    // autoSelector.addAutonomous("4 Cargo Auto", new FourCargoAutoCmd(drivetrain, intake, conveyor, shooter));

    var fiveBallAutoCmd = new FiveCargoAutoCmd(drivetrain, intake, conveyor, shooter);                        //five ball auto for shuffleboard (WIP)
    autoSelector.addDefaultAutonomous("5 Cargo Auto", fiveBallAutoCmd.initialPath, fiveBallAutoCmd);
   
    /* Arcade drive Commands*/
    var arcadeDriveCommand = new RunEndCommand(() -> {
        drivetrain.teleopArcadeDrive(
          // -m_xboxCtrl.getLeftY(),
          driveLimiter.calculate(-m_xboxCtrl.getLeftY()*0.1), 
          // m_xboxCtrl.getRightX(),
          turnLimiter.calculate(m_xboxCtrl.getRightX()*0.55),  //change back to .55 when not testing
          2
        );
    }, drivetrain::stop, drivetrain).withName("ArcadeDriveCommand");

    /* Tank Drive */

    /** var tankDriveCommand = new RunEndCommand(() -> {
          drivetrain.teleopTankDrive(
            -m_xboxCtrl.getRightY(),
            -m_xboxCtrl.getRightX(), 
            2
          );
        }, drivetrain::stop, drivetrain).withName("TankDriveCommand");
    **/

    drivetrain.setDefaultCommand(arcadeDriveCommand);

    configOperatorCommands();
    // configTestingCommands();
  }
  
  /* Operator buttons assigned to commands */
  public void configOperatorCommands() {
    m_xboxCtrl.b().whileHeld(drivetrain.getTargetingCommand(() -> -m_xboxCtrl.getLeftY()));
    
    stratComInterface.arcadeBlackRight().whileHeld(new AcceptBallCommand(intake, shooter, conveyor)).whenReleased(new QueueBallCommand(conveyor, shooter));
    stratComInterface.arcadeWhiteRight().whileHeld(new RejectBallCommand(intake, conveyor));

    // stratComInterface.arcadeBlackLeft().whileHeld(new ShootBallVisionCmd(conveyor, shooter, false));
    // stratComInterface.arcadeWhiteLeft().whileHeld(new ShootBallVisionCmd(conveyor, shooter, true));

    stratComInterface.arcadeBlackLeft().whileHeld(new ShootBallCmd(conveyor, shooter, 2000, 2000));

    stratComInterface.scSideBot().whenPressed(new ShootBallCmd(     //Spins wheels for fender and tarmac buttons
      conveyor, shooter, ShooterConstants.FRONT_RPM_LOW_FENDER, ShooterConstants.REAR_RPM_LOW_FENDER, true));
    stratComInterface.scSideTop().whileHeld(new ShootBallCmd(
      conveyor, shooter, ShooterConstants.FRONT_RPM_HIGH_TARMAC, ShooterConstants.REAR_RPM_HIGH_TARMAC));
    stratComInterface.scSideMid().whileHeld(new ShootBallCmd(
      conveyor, shooter, ShooterConstants.FRONT_RPM_HIGH_FENDER, ShooterConstants.REAR_RPM_HIGH_FENDER));

    stratComInterface.sc1().whileHeld(new RunEndCommand(    //buttons for climb commands
      () -> climb.setPower(1, 1), 
      () -> climb.setPower(0, 0),
      climb
    ));
    stratComInterface.sc4().whileHeld(new RunEndCommand(
      () -> climb.setPower(-7,-.7), 
      () -> climb.setPower(0, 0),
      climb
    ));
    stratComInterface.sc2().whenPressed(new PivotClimbCommand(climb));
    stratComInterface.sc5().whenReleased(new StraightenClimbCommand(climb));


    // auto climb
    stratComInterface.sc3().whenPressed(new PositionClimbCommand(climb, ClimbConstants.LEFT_TO_NEXT_BAR_TARGET, ClimbConstants.RIGHT_TO_NEXT_BAR_TARGET));
    stratComInterface.sc6().whileActiveOnce(new AutoMidToHigh(climb, drivetrain));
  }

  public void configSimTestingCommands() {
    // var ramseteTestCommand = drivetrain.getTrajectoryCommand(DrivetrainConstants.test3MeterForwardTraj).withName("RamseteTestCommand")
    // .andThen(drivetrain::stop);

    // m_xboxCtrl.b()
    //   .whenPressed(ramseteTestCommand);

    // m_xboxCtrl.a().whileHeld(drivetrain.getTargetingCommand(() -> -m_xboxCtrl.getLeftY()));

    // m_xboxCtrl.a().whileHeld(drivetrain.getTrajectoryCommand(drivetrain.loadPath(DrivetrainConstants.THREE_BALL_AUTO_PATH)));
  }

  public void configTestingCommands() {
    // configClimbTestCmds();
    // configShootTestCmds();
    configIntakeTestCmds();
  }

  public void configClimbTestCmds() {
    // // zero climb for testing
    stratComInterface.arcadeBlackLeft().whenPressed(() -> climb.pivotClimb());
    stratComInterface.arcadeWhiteLeft().whenPressed(() -> climb.straightenClimb());

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

    // // auto climb cmd
    stratComInterface.arcadeBlackRight().whenPressed(new AutoClimbCmd(climb, drivetrain));

    // // home climb cmd
    stratComInterface.arcadeWhiteRight().whenPressed(new HomeClimbCmd(climb));

    // // climb via pid (EXTEND TARGET CMD) to next bar wait point target
    // stratComInterface.sc2().whenPressed(new PositionClimbCommand(climb, ClimbConstants.LEFT_TO_NEXT_BAR_WAIT_POINT_TARGET, ClimbConstants.RIGHT_TO_NEXT_BAR_WAIT_POINT_TARGET).withName("ExtendCmd"));
    // stratComInterface.sc5().whenPressed(new PositionClimbCommand(climb, ClimbConstants.LEFT_TO_NEXT_BAR_TARGET, ClimbConstants.RIGHT_TO_NEXT_BAR_TARGET).withName("ExtendCmd2"));


    // // stratComInterface.arcadeWhiteLeft().whenPressed(nextBarCmd);

    stratComInterface.sc4().whenPressed(new PositionClimbCommand(climb, ClimbConstants.MIN_EXTEND_POS, ClimbConstants.MIN_EXTEND_POS));

    // // climb via pid (EXTEND TARGET CMD) to free hook target
    stratComInterface.sc3().whenPressed(new PositionClimbCommand(climb, ClimbConstants.LEFT_FREE_HOOK_TARGET, ClimbConstants.RIGHT_FREE_HOOK_TARGET));

    // // climb via pid (EXTEND TARGET CMD) to min
    // stratComInterface.sc6().whenPressed(new PositionClimbCommand(climb, ClimbConstants.RETRACT_TARGET, ClimbConstants.RETRACT_TARGET));




    stratComInterface.sc1().whenPressed(new PositionClimbCommand(climb, ClimbConstants.LEFT_TO_NEXT_BAR_WAIT_POINT_TARGET, ClimbConstants.RIGHT_TO_NEXT_BAR_WAIT_POINT_TARGET).withName("ExtendCmd"));
    stratComInterface.sc2().whenPressed(new PositionClimbCommand(climb, ClimbConstants.LEFT_TO_NEXT_BAR_TARGET, ClimbConstants.RIGHT_TO_NEXT_BAR_TARGET).withName("ExtendCmd"));


    // // climb up via power - will stop at limit
    // stratComInterface.sc1().whileHeld(new RunEndCommand(
    //   () -> climb.setPower(.5, .5), 
    //   climb::idle,
    //   climb)
    // );
    // // climb down via power - no bottom soft limit
    // stratComInterface.sc4().whileHeld(new RunEndCommand(
    //   () -> climb.setPower(-.5, -.5), 
    //   climb::idle,
    //   climb)
    // );

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
          // hood control
          shooter.setHood(stratComInterface.doubleToggleUp().get());
        },
        () -> {
          shooter.idle();
        }, 
        shooter
      )
    );

    // shooting with vision
    stratComInterface.arcadeBlackLeft().whileHeld(new ShootBallVisionCmd(conveyor, shooter, false));
          
    // regular intake
    stratComInterface.arcadeBlackRight().whileHeld(new AcceptBallCommand(intake, shooter, conveyor)).whenReleased(new QueueBallCommand(conveyor, shooter));

    // feed shooter (spin conveyor forward)
    stratComInterface.arcadeWhiteRight().whileHeld(new RunEndCommand(
        () -> {
          conveyor.setPower(ConveyorConstants.FEEDING_POWER);
        }, 
        () -> {
          conveyor.idle();
        }, 
        conveyor
      )
    );
    
    // intake from shooter - reverse conveyer and both flywheels
    stratComInterface.arcadeWhiteLeft().whileHeld(new RunEndCommand(
        () -> {
          conveyor.setPower(-ConveyorConstants.FEEDING_POWER);
          shooter.setRPM(-2000, -2000);
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

  public void configIntakeTestCmds() {
    // hold intake out
    stratComInterface.singleToggle().whileHeld(new RunEndCommand(
      () -> intake.extendIntake(),
      () -> intake.retractIntake(), 
      intake));

    // intake
    stratComInterface.arcadeBlackRight().whileHeld(new AcceptBallCommand(intake, shooter, conveyor));//.whenReleased(new QueueBallCommand(conveyor, shooter));

    // feed shooter (spin conveyor forward)
    stratComInterface.arcadeWhiteRight().whileHeld(new RunEndCommand(
      () -> {
        conveyor.setPower(ConveyorConstants.FEEDING_POWER);
      }, 
      () -> {
        conveyor.idle();
      }, 
      conveyor
    ));

    // intake from shooter - reverse conveyer and both flywheels
    stratComInterface.arcadeWhiteLeft().whileHeld(new RunEndCommand(
      () -> {
        conveyor.setPower(-ConveyorConstants.FEEDING_POWER);
        shooter.setRPM(-2000, -2000);
      }, 
      () -> {
        // intake.idle();
        conveyor.idle();
        shooter.idle();
      }, 
      conveyor
    ));
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
    // if (selectedAuto.path != null) {
      // drivetrain.setCurrentField2dTrajectory(selectedAuto.path);
    // }
  }
}
