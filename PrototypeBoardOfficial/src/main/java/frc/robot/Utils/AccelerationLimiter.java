package frc.robot.Utils;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

public class AccelerationLimiter {


    private double rateLimit;
    private double previousTime;
    private double previousValue = 0;


    
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

        double outputAdjustment = MathUtil.clamp(input-previousValue, -rateLimit*elapsedTime, rateLimit*elapsedTime);

        boolean areInputAndPreviousSameSign = (input*previousValue) >= 0;

        if(areInputAndPreviousSameSign && Math.abs(input) > Math.abs(previousValue)) {
            previousTime = currentTime;
            previousValue += outputAdjustment;
            return previousValue;
        } else if(!areInputAndPreviousSameSign) {
            previousTime = currentTime;
            previousValue = 0;
            return previousValue;
        } else if(input == 0) {
            previousValue = 0;
            return 0;
        }
        return 0;
    }

}