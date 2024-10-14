// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberTelescope;
import frc.robot.subsystems.ClimberTelescope.ClimbPosition;

public class TelescopeClimbUp extends Command
{
    private final ClimberTelescope climber = RobotContainer.CLIMBER_TELESCOPE;
    private double setPointL, setPointR;
    private ClimbPosition position;
    public TelescopeClimbUp(ClimbPosition positionIn)
    {
        position = positionIn;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        if (position == ClimbPosition.LEFT)
        {
            setPointL = Constants.CLIMBER_TELESCOPE.SIDE_CLIMB_LOW_HEIGHT;
            setPointR = Constants.CLIMBER_TELESCOPE.SIDE_CLIMB_HIGH_HEIGHT;
        }
        if (position == ClimbPosition.CENTER)
        {
            setPointL = Constants.CLIMBER_TELESCOPE.CENTER_CLIMB_HEIGHT;
            setPointR = Constants.CLIMBER_TELESCOPE.CENTER_CLIMB_HEIGHT;
        }
        if (position == ClimbPosition.RIGHT)
        {
            setPointL = Constants.CLIMBER_TELESCOPE.SIDE_CLIMB_HIGH_HEIGHT;
            setPointR = Constants.CLIMBER_TELESCOPE.SIDE_CLIMB_LOW_HEIGHT;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
            climber.setTargetPosition(setPointL, setPointR);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return climber.checkPositionL(Constants.CLIMBER_TELESCOPE.POSITION_TOLERANCE) && climber.checkPositionR(Constants.CLIMBER_TELESCOPE.POSITION_TOLERANCE);
    }
}
