package frc.team4646.drivers.servo;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

/**
 * Hold configuration of a TalonFX and CANcoder for use as a postition control Servo
 */
public class ServoConfig
{
    public final TalonFXConfiguration TALON_FX_CONFIG;
    public final CANcoderConfiguration CANCODER_CONFIG;

    public final String CAN_BUS_NAME;
    public final int TALON_CAN_ID;
    public final int TALON_FOLLOWER_CAN_ID;
    public final boolean TALON_FOLLOWER_INVERT;
    public final int CANCODER_CAN_ID;

    public final double ON_TARGET;
    public final double MIN;
    public final double MAX;

    public final int AT_TARGET_STABILITY_COUNT = 5;

    public ServoConfig(String canBusName,
            int talonCanId,
            double onTarget,
            double min,
            double max,
            TalonFXConfiguration talonFxConfig)
    {
        this(canBusName,
                talonCanId,
                -1,
                false,
                -1,
                onTarget,
                min,
                max,
                talonFxConfig,
                null);
    }

    /**
     * Create a Servo Config using all separate config objects
     */
    public ServoConfig(String canBusName,
            int talonCanId,
            int talonFollowerCanId,
            boolean talonFollowerInvert,
            int cancoderCanId,
            double onTarget,
            double min,
            double max,
            FeedbackConfigs feedbackConfigs,
            MotionMagicConfigs motionMagicConfigs,
            SlotConfigs slotConfigs,
            HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs,
            SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs,
            MagnetSensorConfigs magnetSensorConfigs)
    {
        this(canBusName,
                talonCanId,
                talonFollowerCanId,
                talonFollowerInvert,
                cancoderCanId,
                onTarget,
                min,
                max,
                new TalonFXConfiguration()
                        .withFeedback(feedbackConfigs)
                        .withMotionMagic(motionMagicConfigs)
                        .withSlot0(Slot0Configs.from(slotConfigs))
                        .withHardwareLimitSwitch(hardwareLimitSwitchConfigs)
                        .withSoftwareLimitSwitch(softwareLimitSwitchConfigs),
                new CANcoderConfiguration()
                        .withMagnetSensor(magnetSensorConfigs));
    }

    /**
     * Create a Servo Config with pre-made TalonFX and CANcoder configurations
     */
    public ServoConfig(String canBusName,
            int talonCanId,
            int talonFollowerCanId,
            boolean talonFollowerInvert,
            int cancoderCanId,
            double onTarget,
            double min,
            double max,
            TalonFXConfiguration talonFxConfig,
            CANcoderConfiguration cancoderConfig)
    {
        CAN_BUS_NAME = canBusName;
        TALON_CAN_ID = talonCanId;
        TALON_FOLLOWER_CAN_ID = talonFollowerCanId;
        TALON_FOLLOWER_INVERT = talonFollowerInvert;
        CANCODER_CAN_ID = cancoderCanId;

        ON_TARGET = onTarget;
        MIN = min;
        MAX = max;

        TALON_FX_CONFIG = talonFxConfig;
        CANCODER_CONFIG = cancoderConfig;
    }
}