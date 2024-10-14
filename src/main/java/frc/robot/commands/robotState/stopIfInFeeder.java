// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotStatus;

/* returns true if robot has note in feeder */
public class stopIfInFeeder extends Command
{
    private final RobotStatus state = RobotContainer.ROBOT_STATUS;

    public stopIfInFeeder()
    {
        // addRequirements(state);
    }

    @Override
    public boolean isFinished()
    {
        return state.inFeeder();
    }
}
