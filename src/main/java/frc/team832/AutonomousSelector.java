package frc.team832;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomousSelector {
	
	public class AutonomousMode {
		public final Pose2d startPose;
		public final Trajectory path;
		public final CommandBase autoCommand;

		public AutonomousMode(Pose2d startPose, Trajectory path, CommandBase autoCommand) {
			this.startPose = startPose;
			this.path = path;
			this.autoCommand = autoCommand;
		}

		public AutonomousMode(Pose2d startPose, CommandBase autoCommand) {
			this.startPose = startPose;
			this.path = null;
			this.autoCommand = autoCommand;
		}
	}
	
	private final SendableChooser<AutonomousMode> m_autonomousChooser = new SendableChooser<>();

	public AutonomousSelector() {
		SmartDashboard.putData("Auto Chooser", m_autonomousChooser);
	}

	public void addDefaultAutonomous(String name, Trajectory path, CommandBase autoCommand) {
		m_autonomousChooser.setDefaultOption(name, new AutonomousMode(path.getInitialPose(), path, autoCommand));
	}

	public void addDefaultAutonomous(String name, Pose2d startPose, CommandBase autoCommand) {
		m_autonomousChooser.setDefaultOption(name, new AutonomousMode(startPose, autoCommand));
	}

	public void addDefaultAutonomous(String name, CommandBase autoCommand) {
		addDefaultAutonomous(name, new Pose2d(), autoCommand);
	}

	public void addAutonomous(String name, Trajectory path, CommandBase autoCommand) {
		m_autonomousChooser.addOption(name, new AutonomousMode(path.getInitialPose(), path, autoCommand));
	}

	public void addAutonomous(String name, Pose2d startPose, CommandBase autoCommand) {
		m_autonomousChooser.addOption(name, new AutonomousMode(startPose, autoCommand));
	}

	public void addAutonomous(String name, CommandBase autoCommand) {
		addAutonomous(name, new Pose2d(), autoCommand);
	}

	public AutonomousMode getSelectedAutonomous() {
		return m_autonomousChooser.getSelected();
	}
}
