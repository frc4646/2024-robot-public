// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.team4646.SmartSubsystem;
import frc.team4646.drivers.TalonFXFactory;
import frc.team4646.util.ModifiedSignalLogger;
import frc.team4646.util.StabilityCounter;

public class Shooter extends SmartSubsystem
{
    private TalonFX bottomLeft, bottomRight, topLeft, topRight;

    private double maxRPM = 5800;
    private double commandedRPML, commandedRPMR;
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0)
            .withEnableFOC(true);
    private final MotionMagicVelocityVoltage m_mmVoltageVelocity = new MotionMagicVelocityVoltage(0)
            .withEnableFOC(true);

    private final StabilityCounter stabilityCounter;

    /** Creates a new shooter. */
    public Shooter()
    {
        bottomLeft = new TalonFX(Constants.CAN.SHOOTER_BOTTOM_LEFT);
        bottomRight = new TalonFX(Constants.CAN.SHOOTER_BOTTOM_RIGHT);
        topLeft = new TalonFX(Constants.CAN.SHOOTER_TOP_LEFT);
        topRight = new TalonFX(Constants.CAN.SHOOTER_TOP_RIGHT);
        bottomRight.setInverted(true);
        topLeft.setInverted(true);
        topLeft.setControl(new StrictFollower(Constants.CAN.SHOOTER_BOTTOM_LEFT));
        topRight.setControl(new StrictFollower(Constants.CAN.SHOOTER_BOTTOM_RIGHT));

        TalonFXFactory.ApplySlotConfig(bottomLeft, Constants.SHOOTER.SLOT_CONFIGS);
        TalonFXFactory.ApplySlotConfig(bottomRight, Constants.SHOOTER.SLOT_CONFIGS);

        bottomLeft.getConfigurator().apply(Constants.SHOOTER.CURRENT_LIMIT_CONFIG);
        bottomRight.getConfigurator().apply(Constants.SHOOTER.CURRENT_LIMIT_CONFIG);

        var motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(350) // Target acceleration of 400 rps/s (0.25 seconds to max)
                .withMotionMagicJerk(3000); // Target jerk of 4000 rps/s/s (0.1 seconds)

        bottomLeft.getConfigurator().apply(motionMagicConfigs);
        bottomRight.getConfigurator().apply(motionMagicConfigs);

        stabilityCounter = new StabilityCounter(5);

        BaseStatusSignal.setUpdateFrequencyForAll(100, bottomLeft.getVelocity(), bottomRight.getVelocity());
        setupSysId();
        createDashboard();
    }

    public void setPercentOutput(double speedL, double speedR)
    {
        bottomLeft.set(speedL);
        bottomRight.set(speedR);
    }

    public void setRPM(double rpmL, double rpmR)
    {
        if (rpmL > maxRPM)
        {
            rpmL = maxRPM;
        }
        if (rpmL < -maxRPM)
        {
            rpmL = -maxRPM;
        }

        if (rpmR > maxRPM)
        {
            rpmR = maxRPM;
        }
        if (rpmR < -maxRPM)
        {
            rpmR = -maxRPM;
        }
        commandedRPML = rpmL;
        commandedRPMR = rpmR;

        bottomLeft.setControl(m_voltageVelocity.withVelocity(rpmL / 60));
        bottomRight.setControl(m_voltageVelocity.withVelocity(rpmR / 60));

        // Checks if speed of motors is close to wanted speed within 200 rpm
        if (Constants.TUNING.SHOOTER)
        {
            SmartDashboard.putNumber("RPM L", rpmL);
            SmartDashboard.putNumber("RPM R", rpmR);
            SmartDashboard.putNumber("Actual RPM BL", bottomLeft.getVelocity().getValue() * 60);
            SmartDashboard.putNumber("Actual RPM BR", bottomRight.getVelocity().getValue() * 60);
            SmartDashboard.putNumber("Actual RPM TL", topLeft.getVelocity().getValue() * 60);
            SmartDashboard.putNumber("Actual RPM TR", topRight.getVelocity().getValue() * 60);
            // SmartDashboard.putBoolean("Up to Speed", upToSpeed());
        }
    }

    private boolean checkRPM(double RPM, TalonFX motor)
    {
        if (Math.abs(RPM - motor.getVelocity().getValue() * 60) < 200)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    ///////////////////////////////////////////////////
    /////////////////// SysID stuff ///////////////////
    ///////////////////////////////////////////////////
    private final VoltageOut m_sysidControl = new VoltageOut(0);

    private SysIdRoutine m_SysIdRoutine;

    private void setupSysId()
    {
        if (Constants.TUNING.SYSID)
        {
            m_SysIdRoutine = new SysIdRoutine(
                    new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState("Shooter State")),
                    new SysIdRoutine.Mechanism(
                            (Measure<Voltage> volts) ->
                            {
                                bottomLeft.setControl(m_sysidControl.withOutput(volts.in(edu.wpi.first.units.Units.Volts)));
                                bottomRight.setControl(m_sysidControl.withOutput(volts.in(edu.wpi.first.units.Units.Volts)));
                            },
                            null,
                            this));
        }
    }

    public Command runQuasiTest(Direction direction)
    {
        return m_SysIdRoutine.quasistatic(direction);
    }

    public Command runDynamTest(Direction direction)
    {
        return m_SysIdRoutine.dynamic(direction);
    }

    @Override
    public void cacheSensors()
    {
        stabilityCounter.calculate(checkRPM(commandedRPML, bottomLeft) && checkRPM(commandedRPMR, bottomRight));
    }

    public boolean isOnTarget()
    {
        return stabilityCounter.isStable();
    }

    private void createDashboard()
    {
        if (Constants.TUNING.SHOOTER)
        {
            SmartDashboard.putNumber("Commanded RPM L", commandedRPML);
            SmartDashboard.putNumber("Commanded RPM R", commandedRPMR);
            SmartDashboard.putNumber("Actual RPM BL", bottomLeft.getVelocity().getValue() * 60);
            SmartDashboard.putNumber("Actual RPM BR", bottomRight.getVelocity().getValue() * 60);
            SmartDashboard.putNumber("Actual RPM TL", topLeft.getVelocity().getValue() * 60);
            SmartDashboard.putNumber("Actual RPM TR", topRight.getVelocity().getValue() * 60);
            SmartDashboard.putBoolean("Up to Speed", isOnTarget());
        }
    }
}
