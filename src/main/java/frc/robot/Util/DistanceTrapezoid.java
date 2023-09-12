package frc.robot.Util;

import edu.wpi.first.math.MathSharedStore;
import frc.robot.Constants;

/*
 * Calculate required velocity based on remaining distance current velocity and target velocity
 * velocities are positive
 * if backward required - calculate for forward and use -velocity
 */
public class DistanceTrapezoid {

    double maxVelocity;
    double maxAcceleration;
    double minVelocity;
    private double accelDist; // 1/2 * a * t * t
    private double deltaV; // max velocity change in 1 cycle at max acceleration
    private double lastTime = 0;
    private double lastVelocity = 0;
    private double lastAccel = 0;
    
    public DistanceTrapezoid(double maxVelocity, double maxAcceleration, double minVelocity) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        this.minVelocity = minVelocity;
        deltaV = maxAcceleration * Constants.CycleTime;
        accelDist = maxAcceleration * Constants.CycleTime * Constants.CycleTime / 2;

    }

    public double time() {
        return MathSharedStore.getTimestamp();
    }

    public double calculate(double remainingDistance, double curentVelocity, double tgtVelocity) {
        if(remainingDistance < 0) {
            return  -calculate(-remainingDistance, -curentVelocity, -tgtVelocity);
        }
        double time = time();
        // use last/max velocity if we didn't reach target required velocity
        if(time - lastTime < 2 * Constants.CycleTime) { // last data is valid
            if(curentVelocity < lastVelocity && lastAccel > 0) { // we are too slow while accelerating - use last velocity that will increase accel
                curentVelocity = lastVelocity;
            } else if(curentVelocity > lastVelocity && lastAccel < 0) { // we are to fast while deaccelerating - use last velocity to increase deacceleration
                curentVelocity = lastVelocity;
            }
        }
        // calculate accel for next cycle
        if(curentVelocity < maxVelocity && distanceToVel(curentVelocity+deltaV, tgtVelocity, maxAcceleration) < remainingDistance - cycleDistanceWithAccel(curentVelocity)) {
            // can accelerate - velocity not at max and remaining distance allow deacceleration
            lastAccel = Math.min(deltaV,maxVelocity - curentVelocity);
        } else if(distanceToVel(curentVelocity, tgtVelocity, maxAcceleration) < remainingDistance - cycleDistanceNoAccel(curentVelocity)) {
            // mainintining velocity
            lastAccel = 0;
        } else {
            lastAccel = Math.max(-deltaV, tgtVelocity - curentVelocity);
        }
        // set the velocity 
        lastVelocity = curentVelocity + lastAccel;
        if(lastVelocity > maxVelocity) {
            lastVelocity = maxVelocity;
        }
        if(lastVelocity < minVelocity && lastVelocity > 0  && lastAccel >= 0) {
            lastVelocity = minVelocity;
        }
        lastTime = time;
        return lastVelocity;
    }

    private double cycleDistanceWithAccel(double currentVelocity) {
        return currentVelocity * Constants.CycleTime + accelDist;
    }
    private double cycleDistanceNoAccel(double currentVelocity) {
        return currentVelocity * Constants.CycleTime;
    }

    private double distanceToVel(double currentVel, double tgtVel, double accel) {
        double deltaVel = currentVel - tgtVel;
        return (currentVel - deltaVel/2)*deltaVel/accel;
    }

}
