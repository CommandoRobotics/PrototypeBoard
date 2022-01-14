package frc.robot.Utils;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

public class AccelerationLimiter {

    private final double rateLimit;
    private double previousTime;
    private double previousValue;
    
    /**
     * Create a new AccelerationLimiter with the given rate limit
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public AccelerationLimiter(double rateLimit) {
        this.rateLimit = rateLimit;
        previousTime = WPIUtilJNI.now() * 1e-6;
    }

    /**
     * Filters the input to determine the speed at which to drive based on the rate limit
     * @param input The input value whose acceleration is to be limited
     * @return The filtered value, which will not change faster than the acceleration rate
     */
    public double calculate(double input) {
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime-previousTime;

        // Check if the robot needs to switch direction
        boolean isInputSwitchingDirection = input*previousValue > 0;

        if(!isInputSwitchingDirection && Math.abs(input) > previousValue) {
            // The robot is accelerating without changing direction
            double amountToChange = MathUtil.clamp(input-previousValue, elapsedTime * -rateLimit, elapsedTime * rateLimit);
            previousValue += amountToChange;
            previousTime = currentTime;
            return previousValue;
        } else if(Math.abs(input) <= Math.abs(previousValue)) {
            // The robot is deaccelerating without changing direction
            previousValue = input;
            previousTime = currentTime;
            return previousValue;
        } else {
            previousValue = input;
            previousTime = currentTime;
            return previousValue;
        }
    }

}
