// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TelescopeClimbUntilTorque extends Command
{

    private boolean isLeftTorqueSaturated, isRightTorqueSaturated;
    private double startTime;

    public TelescopeClimbUntilTorque()
    {
        addRequirements(RobotContainer.CLIMBER_TELESCOPE);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        isLeftTorqueSaturated = false;
        isRightTorqueSaturated = false;
        startTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        double torqueL = RobotContainer.CLIMBER_TELESCOPE.getTorqueCurrentL();
        double torqueR = RobotContainer.CLIMBER_TELESCOPE.getTorqueCurrentR();
        if (RobotContainer.CONTROLS_OPERATOR.isRightThumbstickDown())
        {
            if (!isLeftTorqueSaturated && Math.abs(torqueL) >= Constants.CLIMBER_TELESCOPE.MAX_LEFT_TORQUE_CURRENT
                    && Timer.getFPGATimestamp() - startTime > Constants.CLIMBER_TELESCOPE.TORQUE_WAIT_TIME)
            {
                isLeftTorqueSaturated = true;
            }

            if (!isRightTorqueSaturated && Math.abs(torqueR) >= Constants.CLIMBER_TELESCOPE.MAX_RIGHT_TORQUE_CURRENT
                    && Timer.getFPGATimestamp() - startTime > Constants.CLIMBER_TELESCOPE.TORQUE_WAIT_TIME)
            {
                isRightTorqueSaturated = true;
            }

            if (isLeftTorqueSaturated)
            {
                RobotContainer.CLIMBER_TELESCOPE.setTargetVoltageL(0);
            }
            else
            {
                RobotContainer.CLIMBER_TELESCOPE.setTargetVoltageL(-Constants.CLIMBER_TELESCOPE.CLIMB_SPEED);
            }

            if (isRightTorqueSaturated)
            {
                RobotContainer.CLIMBER_TELESCOPE.setTargetVoltageR(0);
            }
            else
            {
                RobotContainer.CLIMBER_TELESCOPE.setTargetVoltageR(-Constants.CLIMBER_TELESCOPE.CLIMB_SPEED);
            }
        }
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
        return isLeftTorqueSaturated && isRightTorqueSaturated;
    }
}
