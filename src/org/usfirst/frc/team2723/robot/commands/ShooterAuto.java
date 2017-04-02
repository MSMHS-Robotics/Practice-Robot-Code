package org.usfirst.frc.team2723.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ShooterAuto extends CommandGroup {

    public ShooterAuto() {
    	addParallel(new Shooter());
    	addSequential(new Wait());
    	addSequential            (new MixerandWow());
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both th==99e chassis and the
        // arm.
    }
}
    