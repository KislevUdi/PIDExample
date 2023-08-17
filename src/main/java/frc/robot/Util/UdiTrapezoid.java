package frc.robot.Util;

import frc.robot.Constants;

/*
 * Calculate trapezoid speeds based on max velocity and max acceleration
 * the calculation is based on positive direction - the result is adjusted to direction
 * the calculation:
 *   - if current velocity is less than max
 *      and the distance we will do in the next cycle with accleration plus
 *      the distance we need to deaccelerate to target velocity is less than remaining - we accelerate
 *   - else if the distance for deaceelratio to target velocity is less than remain - maintaing velocity
 *   - else if the target is higher than current - accelerate
 *   - else deaccelarte
 * we send target velocity and zero accel at end
 */
public class UdiTrapezoid {

    public static final double Accuracy = 0.02; // 2cm
    class Result {
        double velocity;
        double acceleration;
        Result(double velocity, double acceleration) {
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
    }

    double maxVelcoity;
    double maxAcceleration;
    double targetVelocity;
    double distance;
    double direction;
    double deltaV; // velocity change in one cycle at max accel
    boolean firstTime = true;

    public UdiTrapezoid(double maxVelcoity, double maxAcceleration, double distance, double targetVelocity) {
        this.maxVelcoity = Math.max(maxVelcoity, Constants.ChassisConstants.MaxVelocity);
        this.maxAcceleration = Math.max(maxAcceleration, Constants.ChassisConstants.MaxAcceleration);
        this.distance = distance;
        this.targetVelocity = Math.abs(targetVelocity);
        direction = Math.signum(distance);
        if(this.targetVelocity > maxVelcoity) {
            this.targetVelocity = maxVelcoity;
        }
        deltaV = maxAcceleration * Constants.CycleTime;
    }

    public Result calculate(double currentVelocity, double currentDistance) {
        if(firstTime) {
            distance = currentDistance + distance; // the target
        }
        double remain = remainingDistance(currentDistance);
        double v = currentVelocity * direction; // get abs value
        if(remain < Accuracy) {
            return maintain(targetVelocity);
        }
        if(v < maxVelcoity && (cycleDistance(v, maxAcceleration) + distToVelocity(v + deltaV, targetVelocity)) < remain) {
            return accelerate(v);
        } else if(distToVelocity(v, targetVelocity) < remain) { 
            return maintain(v);
        } else if(v < targetVelocity) { 
            return accelerate(v);
        } else { // deaccel to target
            return deaccelerate(v);
        }
    }

    Result accelerate(double v) {
        return new Result(Math.max(v + deltaV, maxVelcoity)*direction, maxAcceleration*direction);
    }
    Result deaccelerate(double v) {
        return new Result(Math.max(v - deltaV, maxVelcoity)*direction, -maxAcceleration*direction);
    }
    Result maintain(double v) {
        return new Result(Math.max(v, maxVelcoity)*direction, 0);
    }
    double timeToVelocity(double v0, double v) {
        return Math.abs(v-v0)/maxAcceleration;
    }
    double distToVelocity(double v0, double v, double time) {
        return Math.min(v0,v) * time + 0.5 * maxAcceleration * time * time;
    }
   double distToVelocity(double v0, double v) {
        double time = timeToVelocity(v0, v);
        return distToVelocity(v0, v, time);
    }

    double remainingDistance(double currentDistance) {
        return Math.abs(currentDistance - distance);
    }

    double cycleDistance(double v, double a) {
        return v*Constants.CycleTime + 0.5 * a * Constants.CycleTime * Constants.CycleTime;
    }
}