// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.intake.feederAndIntakePercentage;
import frc.robot.commands.robotState.stopIfCanShoot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SmartShoot extends SequentialCommandGroup
{
    /** Creates a new SmartShoot. */
    public SmartShoot()
    {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(new ParallelRaceGroup(new stopIfCanShoot(), new WaitCommand(Constants.INTAKE_FEEDER.SMART_SHOOT_TIMEOUT)),
                new feederAndIntakePercentage(Constants.INTAKE_FEEDER.FORCE_FEED_OVERRIDE_PERCENT));
    }
}
