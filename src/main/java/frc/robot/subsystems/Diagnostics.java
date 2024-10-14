package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.configuration.OperatorControls;
import frc.team4646.DiagnosticState;
import frc.team4646.LEDColor;
import frc.team4646.SmartSubsystem;
import frc.team4646.drivers.Candle;

public class Diagnostics extends SmartSubsystem
{
    public final LEDColor OFF = new LEDColor(0, 0, 0),
            RED = new LEDColor(255, 0, 0),
            BLUE = new LEDColor(0, 0, 255);

    private Candle candle;
    private final OperatorControls operator = RobotContainer.CONTROLS_OPERATOR;
    private LEDColor modeDefault = OFF, robotState = OFF;
    private boolean flashing;

    public Diagnostics()
    {
        if (!Utils.isSimulation())
            candle = RobotContainer.CANDLE;
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void setState(DiagnosticState state)
    {
        robotState = state.color;
        flashing = state.flash;
    }

    public void setStateOkay()
    {
        robotState = modeDefault;
    }

    private void setLEDs(LEDColor color, boolean flashing)
    {
        if (!Utils.isSimulation())
        {
            if (flashing)
            {
                candle.set(new StrobeAnimation(color.red, color.green, color.blue, 0, .05, Candle.LED_COUNT));
            }
            else
            {
                candle.set(new SingleFadeAnimation(color.red, color.green, color.blue, 0, 0, Candle.LED_COUNT));
            }
        }
    }

    private void setLEDs(Animation animation)
    {
        if (!Utils.isSimulation())
        {
            candle.set(animation);
        }
    }

    private void setRumble(double percent)
    {
        operator.setRumble(true, percent);
        operator.setRumble(false, percent);
    }

    private LEDColor allianceColor()
    {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? RED : BLUE;
    }

    @Override
    public void updateHardware()
    {
        setLEDs(robotState, flashing);
        // setRumble((DriverStation.isDisabled() && isCriticalIssuePresent) ? Constants.DIAGNOSTICS.RUMBLE_PERCENT : 0.0);
    }

    @Override
    public void onEnable(boolean isAutonomous)
    {
        if (!isAutonomous)
        {
            modeDefault = OFF;
        }
        else
        {
            modeDefault = allianceColor();
        }
    }

    @Override
    public void onDisable()
    {
        modeDefault = allianceColor();
    }
}
