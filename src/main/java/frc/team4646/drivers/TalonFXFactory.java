/*
 * Initially from https://github.com/frc1678/C2022 and https://github.com/HuskieRobotics/3061-lib
 */

package frc.team4646.drivers;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.wpilibj.DriverStation;

@java.lang.SuppressWarnings(
{
        "java:S1104", "java:S116"
})

/**
 * Creates CANTalon objects and configures all the parameters we care about to
 * factory defaults.
 * Closed-loop and sensor parameters are not set, as these are expected to be
 * set by the
 * application.
 */
public class TalonFXFactory
{
    private static final double TIMEOUT_S = .1;

    // TODO Adjust message frame rates?
    private static final TalonFXConfiguration kDefaultConfiguration = configureDefault(new TalonFXConfiguration());
    private static final TalonFXConfiguration kFollowerConfiguration = configureDefault(new TalonFXConfiguration());

    /**
     * Set closed-loop constants for motor controller. Is more concise than applying
     * each constant individually.
     */
    public static void setPID(TalonFX motor, SlotConfigs slot)
    {
        TalonFXConfigurator configs = motor.getConfigurator();
        configs.apply(slot, TIMEOUT_S);
    }

    /**
     * Get our Default configuration for further modification
     * 
     * @return
     */
    public static TalonFXConfiguration getDefaultConfiguration()
    {
        return kDefaultConfiguration;
    }

    /**
     * Create a Talon with our Default configuration
     */
    public static TalonFX createDefaultTalon(int id, String canBusName)
    {
        return createTalon(id, canBusName, kDefaultConfiguration);
    }

    /**
     * Create a Talon to follow another Talon
     */
    public static TalonFX createPermanentFollowerTalon(int id, String canBusName, TalonFX leader, boolean invert)
    {
        final TalonFX talon = createTalon(id, canBusName, kFollowerConfiguration);
        talon.setInverted(invert);
        talon.setControl(new StrictFollower(leader.getDeviceID()));
        return talon;
    }

    /**
     * Create a Talon with a custom configuration
     */
    public static TalonFX createTalon(int id, String canBusName, TalonFXConfiguration talonFXConfig)
    {
        TalonFX talon = new TalonFX(id, canBusName);

        ApplyConfig(talon, talonFXConfig);

        talon.clearStickyFaults(TIMEOUT_S);

        return talon;
    }

    /**
     * Retry config apply up to 5 times, report if failure. Returns true if successful
     */
    public static boolean ApplyConfig(TalonFX motor, TalonFXConfiguration config)
    {
        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i)
        {
            status = motor.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }

        if (!status.isOK())
        {
            // System.out.println("Could not apply configs, error code: " + status.toString());
            DriverStation.reportError("Could not apply configs, error code: " + status.toString(), false);
        }

        return status.isOK();
    }

    /**
     * Retry PID slot config apply up to 5 times, report if failure. Returns true if successful
     */
    public static boolean ApplySlotConfig(TalonFX motor, SlotConfigs slotConfig)
    {
        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i)
        {
            status = motor.getConfigurator().apply(slotConfig);
            if (status.isOK())
                break;
        }

        if (!status.isOK())
        {
            // System.out.println("Could not apply configs, error code: " + status.toString());
            DriverStation.reportError("Could not apply configs, error code: " + status.toString(), false);
        }

        return status.isOK();
    }

    /**
     * Configures a Talon for our Default configuration
     */
    private static TalonFXConfiguration configureDefault(TalonFXConfiguration talon)
    {
        talon.MotorOutput.DutyCycleNeutralDeadband = 0.04;

        talon.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talon.MotorOutput.PeakForwardDutyCycle = 1.0;
        talon.MotorOutput.PeakReverseDutyCycle = -1.0;

        talon.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        talon.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        talon.HardwareLimitSwitch.ForwardLimitEnable = false;
        talon.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        talon.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        talon.HardwareLimitSwitch.ReverseLimitEnable = false;

        talon.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
        talon.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;

        talon.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        talon.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        talon.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        talon.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        talon.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
        talon.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;
        talon.OpenLoopRamps.TorqueOpenLoopRampPeriod = 0;

        talon.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        talon.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
        talon.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0;

        // talon.voltageCompSaturation = 0.0; // In a set request, add .withOutput(12.0)

        talon.CurrentLimits.StatorCurrentLimit = 300;
        talon.CurrentLimits.StatorCurrentLimitEnable = false;

        talon.CurrentLimits.SupplyCurrentLimit = 30;
        talon.CurrentLimits.SupplyCurrentLimitEnable = false;
        talon.CurrentLimits.SupplyCurrentThreshold = 100;
        talon.CurrentLimits.SupplyTimeThreshold = 1;

        return talon;
    }

}