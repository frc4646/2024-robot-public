// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team4646.SmartSubsystem;

public class Climber extends SmartSubsystem
{
    private static final String DASH_NAME = "Climber";
    /** Creates a new ClimberSub. */
    private final TalonFX left, right;

    public Climber()
    {
        left = new TalonFX(Constants.CAN.CLIMBER_LEFT);
        right = new TalonFX(Constants.CAN.CLIMBER_RIGHT);
        right.setInverted(true);
        // left.getConfigurator().apply(
        // new OpenLoopRampsConfigs()
        // .withDutyCycleOpenLoopRampPeriod(.25));
        left.getConfigurator().apply(Constants.CLIMBER.LIMITS_HARDWARE);
        right.getConfigurator().apply(Constants.CLIMBER.LIMITS_HARDWARE);

        // right.setControl(new StrictFollower(Constants.CAN.CLIMBER_LEFT));

        // right.setControl(ServoMode.OPEN_LOOP, 0);

        createDashboard();
    }

    public void setOutput (double percentL, double percentR)
    {
        left.set(percentL);
        right.set(percentR);
    }
    public void setOutputL(double percent)
    {
        left.set(percent);
    }

    public void setOutputR(double percent)
    {
        right.set(percent);
    }

    public double getTorqueCurrentL()
    {
        return left.getTorqueCurrent().getValueAsDouble();
    }

    public double getTorqueCurrentR()
    {
        return right.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void updateHardware()
    {
        // if tuning sys id this gets set in the setMotor call
        if (!Constants.TUNING.SYSID)
        {
            // left.updateHardware();
            // right.updateHardware();
        }
    }

    @Override
    public void cacheSensors()
    {
        // left.cacheSensors();
        // right.cacheSensors();
        SmartDashboard.putBoolean("ClimberR", right.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);
        SmartDashboard.putBoolean("ClimberL", left.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);
    }

    public void createDashboard()
    {
        if (Constants.TUNING.CLIMBER)
        {
            Shuffleboard.getTab(DASH_NAME).add(this).withPosition(0, 0).withSize(2, 1);
            Shuffleboard.getTab(DASH_NAME).addNumber("Left Torque Current", () -> left.getTorqueCurrent().getValueAsDouble()).withPosition(2, 0);
            Shuffleboard.getTab(DASH_NAME).addNumber("Right Torque Current", () -> right.getTorqueCurrent().getValueAsDouble()).withPosition(3, 0).withSize(2,
                    1);

            Shuffleboard.getTab(DASH_NAME).addNumber("Left Supply Current", () -> left.getSupplyCurrent().getValueAsDouble()).withPosition(0, 1).withSize(2, 1);
            Shuffleboard.getTab(DASH_NAME).addNumber("Right Supply Current", () -> left.getSupplyCurrent().getValueAsDouble()).withPosition(2, 1).withSize(2,
                    1);

            // Shuffleboard.getTab(DASH_NAME).addBoolean("Saturated Min", () ->
            // left.isSaturatedMin(left.getAngle())).withPosition(0, 2);
            // Shuffleboard.getTab(DASH_NAME).addBoolean("Saturated Max", () ->
            // left.isSaturatedMax(left.getAngle())).withPosition(1, 2);
        }
    }
}