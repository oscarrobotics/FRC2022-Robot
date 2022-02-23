package frc.team832.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team832.robot.subsystems.ConveyerSubsystem;
import frc.team832.robot.subsystems.ShooterSubsystem;

public class ShootBallCommand extends SequentialCommandGroup {
    public ShootBallCommand(ConveyerSubsystem conveyer, ShooterSubsystem shooter) {
        addRequirements(conveyer, shooter);
        addCommands(
            // hey nicole :) how are you? how's your japanese competition thingy going?

            /**
             * 99 litle bugs in the code
             * 99 bugs in the code
             * patch one down, compile it down
             * 112 litle bugs in the code
             * 
             * (many sleepless nights later)
             * 
             * 0 little bugs in the code
             * 0 bugs in the code
             * WHY TF IS THIS STILL NOT WORKING
             * 0 little bugs in the code
             */


            // you can use the queueballcommand as an example and heres the link to the wpilib docs for the different prewritten commands: https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html?highlight=instantcommand#instantcommand 
            
            // Step 1: reverse conveyer for 1 sec
            // Step 2: set power of shooter equal to the shooting speed constant 
                // (one thing to do later is either rename "shooting speed" to "shooting power" OR make a method to set speed directly, and not set power)
            // Step 3: put converyer forward for 2 sec
                // also the time (1 sec reverse and 2 sec forward) will probably need to be changed with testing, these are just placeholders for now
            // Step 4: stop both conveyer and shooter
        );
    }
}
