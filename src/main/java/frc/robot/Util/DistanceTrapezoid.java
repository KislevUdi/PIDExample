package frc.robot.Util;

import edu.wpi.first.math.MathSharedStore;
import static frc.robot.Constants.CycleTime;;

/*
 * Calculate required velocity based on remaining distance current velocity and target velocity
 * velocities are positive
 * if backward required - calculate for forward and use -velocity
 */
public class DistanceTrapezoid {

    double maxVelocity;
    double maxAcceleration;
    double minVelocity;
    double minDistance; // if remaining distance < minDistance - end condition
    private double accelDist; // 1/2 * a * t * t
    private double deltaV; // max velocity change in 1 cycle at max acceleration
    private double lastTime = 0;
    private double lastVelocity = 0;
    private double lastAccel = 0;
    
    public DistanceTrapezoid(double maxVelocity, double maxAcceleration, double minVelocity) {
        this(maxVelocity, maxAcceleration, minVelocity, 0.05);
    }
    public DistanceTrapezoid(double maxVelocity, double maxAcceleration, double minVelocity, double minDistance) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        this.minVelocity = minVelocity;
        this.minDistance = minDistance;
        deltaV = maxAcceleration * CycleTime;
        accelDist = maxAcceleration * CycleTime * CycleTime / 2;
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
        curentVelocity = setCurrentVelocity(curentVelocity, time);

        // calculate accel for next cycle
        if(canAccelerate(curentVelocity, tgtVelocity, remainingDistance)) {
            lastAccel = deltaV;
        } else if(canMaintain(curentVelocity, tgtVelocity, remainingDistance)) {
            lastAccel = 0;
        } else {
            lastAccel = deaccel(curentVelocity, tgtVelocity, remainingDistance);
        }

        // set the velocity 
        lastVelocity = nextVelocity(curentVelocity, lastAccel);
        lastTime = time;
        return lastVelocity;
    }

    // set current velocity based on expected velocity
    private double setCurrentVelocity(double curentVelocity, double time) {
        if(isLastValid(time)) { // last data is valid
            if(curentVelocity < lastVelocity && lastAccel > 0) { // we are too slow while accelerating - use last velocity that will increase accel
                return lastVelocity;
            } else if(curentVelocity > lastVelocity && lastAccel < 0) { // we are to fast while deaccelerating - use last velocity to increase deacceleration
                return lastVelocity;
            }
        }
        return curentVelocity;
    }

    // set the next velocity not to exceed max and not belowe min while accelerating
    private double nextVelocity(double curentVelocity, double accel) {
        double v = curentVelocity + accel;
        if(v > maxVelocity) {
            return maxVelocity;
        }
        if(v < minVelocity && v > 0  && accel >= 0) {
            return minVelocity;
        }
        return v;

    }

    private boolean isLastValid(double time) {
        return time - lastTime < CycleTime*2;

    }

    private boolean canAccelerate(double curentVelocity, double tgtVelocity, double remainingDistance) {
        return curentVelocity < maxVelocity && 
               distanceToVel(curentVelocity+deltaV, tgtVelocity, maxAcceleration) < remainingDistance - cycleDistanceWithAccel(curentVelocity);
    } 
    private boolean canMaintain(double curentVelocity, double tgtVelocity, double remainingDistance) {
        return lastAccel > 0 || 
               distanceToVel(curentVelocity, tgtVelocity, maxAcceleration) < remainingDistance - cycleDistanceNoAccel(curentVelocity);
    }

    private double cycleDistanceWithAccel(double currentVelocity) {
        // calculate distance with 1 cycle acceleration and 1 cycle maintain
        return currentVelocity * CycleTime * 2 + accelDist + deltaV*CycleTime;
    }
    private double cycleDistanceNoAccel(double currentVelocity) {
        return currentVelocity * CycleTime;
    }

    private double distanceToVel(double currentVel, double tgtVel, double accel) {
        double deltaVel = currentVel - tgtVel;
        return (currentVel - deltaVel/2)*deltaVel/accel;
    }

    // calculate deacceleration required to reach tgt distance at target velocity
    private double deaccel(double currentV, double tgtV, double distance) {
        if(currentV == tgtV) {
            return 0;
        }
        double t = 2 * distance / (currentV + tgtV);
        return (tgtV - currentV)/t;
    }

}
