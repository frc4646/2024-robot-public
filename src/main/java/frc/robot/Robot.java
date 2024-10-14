// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot
{
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    @Override
    public void robotInit()
    {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic()
    {
        robotContainer.cacheSensors();
        CommandScheduler.getInstance().run(); // Must be called from robotPeriodic(). Runs these steps: Polls buttons, adds newly-scheduled commands, runs
                                              // already-scheduled commands, removes finished or interrupted commands, calls subsystem periodic() methods.
        robotContainer.updateHardware();
    }

    @Override
    public void disabledInit()
    {
        robotContainer.onDisable();
    }

    @Override
    public void autonomousInit()
    {
        robotContainer.onEnable(true);
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
        robotContainer.onEnable(false);
    }

    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic()
    {
        robotContainer.whileDisabled();

        autonomousCommand = robotContainer.getAutonomousCommand();
    }

    @Override
    public void autonomousPeriodic()
    {
    }

    @Override
    public void teleopPeriodic()
    {
    }

    @Override
    public void testPeriodic()
    {
    }

    @Override
    public void simulationPeriodic()
    {
    }
}
