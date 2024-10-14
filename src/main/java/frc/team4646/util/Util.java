package frc.team4646.util;

import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util
{
    public static final double kEpsilon = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util()
    {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude)
    {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max)
    {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean inRange(double v, double maxMagnitude)
    {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    /**
     * Checks if the given input is within the range (min, max), both exclusive.
     */
    public static boolean inRange(double v, double min, double max)
    {
        return v > min && v < max;
    }

    public static double interpolate(double a, double b, double x)
    {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    /**
     * Remap an input (min to max) to a new min and max
     */
    public static double map(double input, double in_min, double in_max, double out_min, double out_max)
    {
        return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static String joinStrings(final String delim, final List<?> strings)
    {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i)
        {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1)
            {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon)
    {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b)
    {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon)
    {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon)
    {
        boolean result = true;
        for (Double value_in : list)
        {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    /***
     * Correct for a joystick's deadband. If *value* is below *deadband*, treat as 0
     * 
     * @param value
     *            input, -1-1
     * @param deadband
     *            ignore below this, scale from 0-1 above this
     */
    public static double handleDeadband(double value, double deadband)
    {
        deadband = Math.abs(deadband);
        if (deadband == 1)
        {
            return 0;
        }
        double scaledValue = (value + (value < 0 ? deadband : -deadband)) / (1 - deadband);
        return (Math.abs(value) > Math.abs(deadband)) ? scaledValue : 0;
    }

    /**
     * Correct for a joystick's deadband and also apply a x^power factor. If *value*
     * is below *deadband*, treat as 0
     * 
     * @param value
     *            input, -1-1
     * @param deadband
     *            ignore below this, scale from 0-1 above this
     * @param power
     *            value^power scaling
     * @return
     */
    public static double handleJoystick(double value, double deadband, int power)
    {
        double banded = handleDeadband(value, deadband);
        if (power % 2 == 0)
        {
            // odd, so have to account for flipping the sign when applying the power
            return Math.abs(Math.pow(banded, power)) * Math.signum(banded);
        }
        else
        {
            // even
            return Math.pow(banded, power);
        }
    }

    public static double round(double value, int decimals)
    {
        double shift = Math.pow(10.0, decimals);
        return Math.round(value * shift) / shift;
    }
}
