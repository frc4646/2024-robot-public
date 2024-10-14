package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ShooterMap;
import frc.team4646.DiagnosticState;
import frc.team4646.LEDColor;
import frc.team4646.drivers.servo.AngleServo;
import frc.team4646.drivers.servo.ServoConfig;

public final class Constants
{
    public static int CAN_TIMEOUT = 100;

    public static final double LOOP_TIME = 0.02;

    public static final String COMP_DASH_NAME = "Main";

    public static final boolean DATA_LOGGING_ENABLED = false;

    public static final class CAN
    {
        public static final String BUS_NAME = "rio"; // "rio" is built-in

        public static final int POWER_DISTRIBUTION_PANEL = 1;
        public static final int CANDLE = 15;
        public static final int CANIFIER = 16;
        public static final int SHOOTER_BOTTOM_LEFT = 26;
        public static final int SHOOTER_BOTTOM_RIGHT = 27;
        public static final int SHOOTER_TOP_LEFT = 28;
        public static final int SHOOTER_TOP_RIGHT = 29;
        public static final int INTAKE = 25;
        public static final int ARM_LEADER = 35;
        public static final int CLIMBER_LEFT = 7; // TODO
        public static final int CLIMBER_RIGHT = 8; // TODO
        public static final int CLIMBER_LEFT_CANCODER = 9; // TODO
        public static final int CLIMBER_RIGHT_CANCODER = 10; // TODO
        public static final int ARM_FOLLOWER = 31; // TODO
        public static final int ARM_CANCODER = 32; // TODO
        public static final int FEEDER = 33; // TODO
    }

    public static final class DIGIN
    {
        public static final int INTAKE_RIGHT = 0;
        public static final int FEEDER = 1;
        public static final int INTAKE_MID = 2;
    }

    public static final class TUNING
    {
        public static final boolean INFRASTRUCTURE = false;
        public static final boolean SWERVE = false;
        public static final boolean VISION = true;
        public static final boolean SHOOTER = true;
        public static final boolean SHOOTER_ARM = false;
        public static final boolean SYSID = false;
        public static final boolean CLIMBER = false;
    }

    public static final class DIAGNOSTICS
    {
        public static final DiagnosticState NO_PIECE = new DiagnosticState(new LEDColor(255, 0, 0), false);
        public static final DiagnosticState INTAKE_NOTE = new DiagnosticState(new LEDColor(255, 155, 0), false);
        public static final DiagnosticState ALLIGNED = new DiagnosticState(new LEDColor(0, 255, 0), true);
    }

    public static final class SHOOTER_ARM
    {
        /** order of slot configs must match the {@link AngleServo.Slot} order, where 0 is precise and 1 is loose */
        public static final ServoConfig SERVO_CONFIG = new ServoConfig(
                CAN.BUS_NAME,
                CAN.ARM_LEADER,
                CAN.ARM_FOLLOWER,
                SERVO.FOLLOWER_INVERT,
                CAN.ARM_CANCODER,
                SERVO.ON_TARGET_DEGREES,
                SERVO.DEGREES_MIN,
                SERVO.DEGREES_MAX,
                new TalonFXConfiguration()
                        .withFeedback(SERVO.FEEDBACK_CONFIGS)
                        .withMotionMagic(SERVO.MOTION_MAGIC)
                        .withSlot0(Slot0Configs.from(SERVO.SLOT_CONFIG_PRECISE))
                        .withSlot1(Slot1Configs.from(SERVO.SLOT_CONFIG_OVER_THE_TOP))
                        .withHardwareLimitSwitch(SERVO.LIMITS_HARDWARE)
                        .withSoftwareLimitSwitch(SERVO.LIMITS_SOFTWARE)
                        .withCurrentLimits(SERVO.CURRENT_LIMIT_CONFIG)
                        .withMotorOutput(SERVO.MOTOR_OUTPUTS),
                new CANcoderConfiguration().withMagnetSensor(SERVO.MAGNET_SENSOR));

        public static final double Y_PIVOT_TO_SPEAKER_M = Units.inchesToMeters(54); // Vertical distance from pivot to bottom of speaker goal
        public static final double X_PIVOT_OFFSET_M = Units.inchesToMeters(-5.875); // Fore/aft distance from swerve center to pivot

        public static final class SERVO
        {
            public static final double ON_TARGET_DEGREES = .3;
            public static final double ON_TARGET_DEGREES_PER_SECOND = 2.0;

            public static final double GEAR_RATIO = (84.0 / 12.0) * (84.0 / 16.0) * (72.0 / 50.0) * (76.0 / 12.0);
            public static final double DEGREES_MIN = 15.0; // 0 is horizontal, 90 is up
            public static final double DEGREES_MAX = 122.0;
            public static final double DEGREES_HOME = 9.5;
            public static final double LENGTH = 20.0; // Inches
            public static final double ANGLE_OFFSET_ROTATIONS = .094; // value from Tuner X, units in rotations; calibration angle to 0 deg horizontal
            public static final double AMP_DEGREES = 118.5;

            public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
                    .withRotorToSensorRatio(GEAR_RATIO)
                    .withSensorToMechanismRatio(1.0)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withFeedbackRemoteSensorID(CAN.ARM_CANCODER);

            public static final MotionMagicConfigs MOTION_MAGIC = new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(90) // 90 deg per second at cruise
                    .withMotionMagicAcceleration(90 * 2) // .5 seconds to max velocity
                    .withMotionMagicJerk(90 * 20); // .2s to reach max accel

            /** Tune values via Tuner X, then set in here */
            public static final SlotConfigs SLOT_CONFIG_PRECISE = new SlotConfigs()
                    .withKP(800) // if you bump it to 1600, it gets more accurate but chatters a lot
                    .withKI(800)
                    .withKD(.028345)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKG(0.020766)
                    .withKS(.089948)
                    .withKV(3.0606)
                    .withKA(.0062749);
            // these gains are good when going over the top (~90deg)
            public static final SlotConfigs SLOT_CONFIG_OVER_THE_TOP = new SlotConfigs()
                    .withKP(55)
                    .withKI(5)
                    .withKD(.028345)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKG(0.020766)
                    .withKS(.089948)
                    .withKV(3.0606)
                    .withKA(.0062749);

            public static final HardwareLimitSwitchConfigs LIMITS_HARDWARE = new HardwareLimitSwitchConfigs()
                    .withForwardLimitEnable(false)
                    .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)
                    .withReverseLimitEnable(true)
                    .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen);
            public static final SoftwareLimitSwitchConfigs LIMITS_SOFTWARE = new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(DEGREES_MAX / 360)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(DEGREES_MIN / 360);

            public static CurrentLimitsConfigs CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(60)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentThreshold(90)
                    .withSupplyTimeThreshold(1);

            public static final MagnetSensorConfigs MAGNET_SENSOR = new MagnetSensorConfigs()
                    .withMagnetOffset(ANGLE_OFFSET_ROTATIONS);

            public static final MotorOutputConfigs MOTOR_OUTPUTS = new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
            public static final boolean FOLLOWER_INVERT = false;
        }

        public static final double SLOT_MIN_DEGREES = 55;
        public static final double SLOT_MAX_DEGREES = 125;
        public static final double AMP_STOCKPILE_ANGLE = 45;

    }

    public static final class SWERVE_DRIVE
    {
        public static final double ALLIGNED_DEGREES = 3;

        public static final double TARGET_ROTATION_DAMPENING = -0.065;
        public static final double TIMEOUT_DISABLED_COAST = 5.0;

        public static final double MaxSpeed = 4.25; // meters per second desired top speed
        public static final double MaxAngularRate = Math.PI * 1.5; // 6/8 a rotation per second max angular velocity
        public static final double MaxSpeedAccel = MaxSpeed * 2; // from 0 to this in one second
        public static final double MaxAngularRateAccel = MaxAngularRate * 3.5; // from 0 to this in one second

        public static final double MaxSpeedInAuto = 4; // 4 meters per second for Auto

        public static final boolean AUTO_OVERRIDE = true;

        public static final double NUDGE_X = -0.3;
        public static final double NUDGE_Y = -0.3;

        public static final double PID_ROTATION_KP = 23;
        public static final double PID_ROTATION_KI = 0.005;
        public static final double PID_ROTATION_KD = 2;

        public static final double STAGE_PID_KP = 13;
        public static final double PID_TRANSLATION_KP = 10;
        public static final double PID_TRANSLATION_KI = 0.0025;
        public static final double PID_TRANSLATION_KD = 0;

        public static CurrentLimitsConfigs CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentThreshold(80)
                .withSupplyTimeThreshold(1);
        public static CurrentLimitsConfigs TURN_CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentThreshold(40)
                .withSupplyTimeThreshold(1);
        public static OpenLoopRampsConfigs RAMP_RATES = new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(.2)
                .withVoltageOpenLoopRampPeriod(.2);

        public static final double AMP_DEGREES = 270;

    }

    public static final class SHOOTER
    {
        public static final double MINIMUM_DISTANCE = 1.18;
        public static final double MAXIMUM_DISTANCE = 6.5;

        public static ShooterMap MAP = new ShooterMap();
        static
        {
            MAP.add(1.18, 2000, 1700, 62, true);
            MAP.add(2.0, 2600, 1900, 47.0, true);
            MAP.add(2.5, 2900, 2200, 40.5, true);
            MAP.add(3.0, 3500, 2400, 36.5, true);
            MAP.add(3.5, 3800, 2600, 32.7, true);
            MAP.add(4.0, 4100, 2800, 29.5, true);
            MAP.add(4.5, 4500, 3000, 28.75, true);
            MAP.add(5.0, 4800, 3300, 27.75, true);
            MAP.add(5.5, 5000, 3500, 27.0, true);
            MAP.add(6.0, 5300, 3700, 25.25, true);
            MAP.add(6.5, 5400, 4000, 24.75, true);
        }

        public static final double SPIN_PERCENT = 0.6; // todo: make sure this works with close shots, where the spin percent was lower

        public static SlotConfigs SLOT_CONFIGS = new SlotConfigs()
                .withKP(.19593)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(.2318)
                .withKV(.12601)
                .withKA(.029396);

        public static CurrentLimitsConfigs CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentThreshold(100)
                .withSupplyTimeThreshold(1);

        public static final double AMP_SCORE_RPM = 450;
        public static final double AMP_LAUNCH_RPM = 3200;
        public static final double AMP_SLIDE_RPM = 2000;
        public static final double PASSIVE_RPM = 500;
        public static final double DEFAULT_PERCENT_SMARTDASH = 0.7;

        public static final double DEFAULT_RPM_SMARTDASH = 4000;

    }

    public static final class INTAKE_FEEDER
    {
        // TODO: reconcile FORCE_PERCENT, FEED_PERCENT, and FORCE_FEED_OVERRIDE_PERCENT, as they probably are
        // all basically the same thing but aren't the same values.
        public static final double FORCE_PERCENT = 0.4;
        public static final double INTAKE_LOAD_PERCENT = 0.85;
        public static final double FEED_PERCENT = 0.25;
        public static final double REJECT_TIME = 0.3;
        public static final double REJECT_PERCENT = -0.2;
        public static final double FORCE_FEED_OVERRIDE_PERCENT = 0.6;
        public static final double SMART_SHOOT_TIMEOUT = 2;
        public static final double AUTO_FEED_WAIT = 0.2;
        public static final double JIGGLE_TIME = 0.3;

        public static CurrentLimitsConfigs CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentThreshold(40)
                .withSupplyTimeThreshold(1);
    }

    public static final class CLIMBER
    {
        public static final boolean ENABLED = false;

        public static final double CLIMB_SPEED = 1;

        public static final double MAX_LEFT_TORQUE_CURRENT = 9; // TODO
        public static final double MAX_RIGHT_TORQUE_CURRENT = 7;

        public static final HardwareLimitSwitchConfigs LIMITS_HARDWARE = new HardwareLimitSwitchConfigs()
                .withForwardLimitEnable(false)
                .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)
                .withReverseLimitEnable(false)
                .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen);
    }

    public static final class CLIMBER_TELESCOPE
    {
        public static final boolean ENABLED = true;

        public static final double CLIMB_SPEED = 1.0;

        public static final double MAX_LEFT_TORQUE_CURRENT = 13; // TODO
        public static final double MAX_RIGHT_TORQUE_CURRENT = 13;
        public static final double MAX_HEIGHT = 23.5;
        public static final double MIN_HEIGHT = 0;
        public static final double TORQUE_WAIT_TIME = 0.5;
        public static final double SIDE_CLIMB_HIGH_HEIGHT = 23.5;
        public static final double SIDE_CLIMB_LOW_HEIGHT = 18;
        public static final double CENTER_CLIMB_HEIGHT = 16;
        public static final double POSITION_TOLERANCE = 0.5;
        public static final double PULL_UP_INCHES = 6;

        public static final ServoConfig SERVO_CONFIG_L = new ServoConfig(
                CAN.BUS_NAME,
                CAN.CLIMBER_LEFT,
                SERVO.ON_TARGET_POSITION,
                SERVO.POSITION_MIN,
                SERVO.POSITION_MAX,
                new TalonFXConfiguration()
                        .withFeedback(SERVO.FEEDBACK_CONFIGS)
                        .withMotionMagic(SERVO.MOTION_MAGIC)
                        .withSlot0(Slot0Configs.from(SERVO.SLOT_CONFIG_PRECISE))
                        .withSlot1(Slot1Configs.from(SERVO.SLOT_CONFIG_PRECISE))
                        .withHardwareLimitSwitch(SERVO.LIMITS_HARDWARE)
                        .withSoftwareLimitSwitch(SERVO.LIMITS_SOFTWARE)
                        .withCurrentLimits(SERVO.CURRENT_LIMIT_CONFIG)
                        .withMotorOutput(SERVO.MOTOR_OUTPUTS_L));

        public static final ServoConfig SERVO_CONFIG_R = new ServoConfig(
                CAN.BUS_NAME,
                CAN.CLIMBER_RIGHT,
                SERVO.ON_TARGET_POSITION,
                SERVO.POSITION_MIN,
                SERVO.POSITION_MAX,
                new TalonFXConfiguration()
                        .withFeedback(SERVO.FEEDBACK_CONFIGS)
                        .withMotionMagic(SERVO.MOTION_MAGIC)
                        .withSlot0(Slot0Configs.from(SERVO.SLOT_CONFIG_PRECISE))
                        .withSlot1(Slot1Configs.from(SERVO.SLOT_CONFIG_PRECISE))
                        .withHardwareLimitSwitch(SERVO.LIMITS_HARDWARE)
                        .withSoftwareLimitSwitch(SERVO.LIMITS_SOFTWARE)
                        .withCurrentLimits(SERVO.CURRENT_LIMIT_CONFIG)
                        .withMotorOutput(SERVO.MOTOR_OUTPUTS_R));

        public static final class SERVO
        {
            public static final double ON_TARGET_POSITION = .3;
            public static final double ON_TARGET_POSITION_PER_SECOND = 2.0;

            public static final double SPOOL_DIAMETER = 1.0;
            public static final double GEAR_RATIO = (72.0 / 9.0) * (84.0 / 16.0) * SPOOL_DIAMETER;
            public static final double POSITION_MIN = 0.0; // TODO
            public static final double POSITION_MAX = 12.0; // TODO

            public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
                    .withRotorToSensorRatio(1.0)
                    .withSensorToMechanismRatio(GEAR_RATIO / Math.PI); // using the motor's sensor, so mechanism ratio is the gear ratio

            public static final MotionMagicConfigs MOTION_MAGIC = new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(7) // 90 deg per second at cruise
                    .withMotionMagicAcceleration(7 * 2) // .5 seconds to max velocity
                    .withMotionMagicJerk(7 * 20); // .2s to reach max accel

            public static final SlotConfigs SLOT_CONFIG_PRECISE = new SlotConfigs()
                    .withKP(15)
                    .withKI(0)
                    .withKD(0)
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withKG(0)
                    .withKS(.2)
                    .withKV(4)
                    .withKA(0);

            public static final HardwareLimitSwitchConfigs LIMITS_HARDWARE = new HardwareLimitSwitchConfigs()
                    .withForwardLimitEnable(true)
                    .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen)
                    .withReverseLimitEnable(true)
                    .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
                    .withReverseLimitAutosetPositionValue(POSITION_MIN).withReverseLimitAutosetPositionEnable(true);
            public static final SoftwareLimitSwitchConfigs LIMITS_SOFTWARE = new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(MAX_HEIGHT)
                    .withReverseSoftLimitEnable(false)
                    .withReverseSoftLimitThreshold(MIN_HEIGHT);

            public static CurrentLimitsConfigs CURRENT_LIMIT_CONFIG = new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(20)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentThreshold(60)
                    .withSupplyTimeThreshold(1);

            public static final MotorOutputConfigs MOTOR_OUTPUTS_L = new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
            public static final MotorOutputConfigs MOTOR_OUTPUTS_R = new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
        }

    }

    public static final class FIELD
    {
        // public static final AprilTag SPEAKER_RED = new AprilTag(4, new Pose3d(652.73, 218.42, 57.13, new Rotation3d(0.0,
        // 180.0, 0.0)));
        // public static final AprilTag SPEAKER_BLUE = new AprilTag(7, new Pose3d(-1.50, 218.42, 57.13, new Rotation3d(0.0, 0.0,
        // 0.0)));
        public static final Pose2d STAGE_CENTER = new Pose2d(3.22, 4.07, Rotation2d.fromDegrees(0));
        public static final Pose2d STAGE_LEFT = new Pose2d(5.72, 5.57, Rotation2d.fromDegrees(0));
        public static final Pose2d STAGE_RIGHT = new Pose2d(5.71, 2.73, Rotation2d.fromDegrees(0));
        public static final Pose2d SPEAKER = new Pose2d(0, 5.45, Rotation2d.fromDegrees(0));
        public static final Pose2d AMP = new Pose2d(1.92, 8.09, Rotation2d.fromDegrees(270));
        public static final Pose2d SOURCE_CLOSE = new Pose2d(1.71, 0, Rotation2d.fromDegrees(0));
        public static final Pose2d SOURCE_FAR = new Pose2d(0, 1.06, Rotation2d.fromDegrees(0));
        public static final Pose2d AMP_CLOSE = new Pose2d(3.70, 7, Rotation2d.fromDegrees(150)); // was 3.45, 7.4
        public static final double FIELD_LENGTH_M = 16.542;
        public static final double FIELD_WIDTH_M = 8.014;
        public static final int STAGE_RIGHT_ANGLE = 240;
        public static final int STAGE_LEFT_ANGLE = 120;
    }
}