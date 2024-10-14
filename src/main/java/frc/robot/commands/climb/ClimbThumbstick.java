// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimbThumbstick extends Command
{
    private final Climber climb = RobotContainer.CLIMBER;
    

    /** Creates a new ArmThumbstick. */
    public ClimbThumbstick()
    {
        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        double speedL;
        double speedR;

        if (RobotContainer.CONTROLS_OPERATOR.isLeftTriggerPressed())
        {
            speedL = RobotContainer.CONTROLS_OPERATOR.getClimberJoystick() * Constants.CLIMBER.CLIMB_SPEED;
        } else {
            speedL = 0;
        }

        if (RobotContainer.CONTROLS_OPERATOR.isRightTriggerPressed())
        {
            speedR = RobotContainer.CONTROLS_OPERATOR.getClimberJoystick() * Constants.CLIMBER.CLIMB_SPEED;
        } else {
            speedR = 0;
        }
        climb.setOutput(speedL, speedR);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
