// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.team4646.SmartSubsystem;
import frc.team4646.drivers.TalonFXFactory;

public class Intake extends SmartSubsystem
{
    private final TalonFX IntakeMotor;
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(true);

    /** Creates a new intake. */
    public Intake()
    {
        IntakeMotor = TalonFXFactory.createDefaultTalon(Constants.CAN.INTAKE, Constants.CAN.BUS_NAME);
        IntakeMotor.getConfigurator().apply(
                new OpenLoopRampsConfigs()
                        .withDutyCycleOpenLoopRampPeriod(.25));
        IntakeMotor.getConfigurator().apply(Constants.INTAKE_FEEDER.CURRENT_LIMIT_CONFIG);
        IntakeMotor.setInverted(true);
    }

    public void setOutput(double percentage)
    {
        dutyCycleOut.withOutput(percentage);
        IntakeMotor.setControl(dutyCycleOut);
        // sends information to the motor
        // is called outside of this file (spin the motor)e
    }

    @Override
    public void periodic()
    {
        // SmartDashboard.putData("intake subsystem", this);
    }

}
