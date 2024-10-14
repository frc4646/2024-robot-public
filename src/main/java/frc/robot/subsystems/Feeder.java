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

public class Feeder extends SmartSubsystem
{
    private final TalonFX FeederMotor;
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(true);

    /** Creates a new intake. */
    public Feeder()
    {
        FeederMotor = TalonFXFactory.createDefaultTalon(Constants.CAN.FEEDER, Constants.CAN.BUS_NAME);
        FeederMotor.getConfigurator().apply(
                new OpenLoopRampsConfigs()
                        .withDutyCycleOpenLoopRampPeriod(.25));
        FeederMotor.getConfigurator().apply(Constants.INTAKE_FEEDER.CURRENT_LIMIT_CONFIG);
        FeederMotor.setInverted(true);
    }

    public void setOutput(double percentage)
    {
        dutyCycleOut.withOutput(percentage);
        FeederMotor.setControl(dutyCycleOut);
        // sends information to the motor
        // is called outside of this file (spin the motor)e
    }
}
