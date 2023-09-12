package frc.robot.Util;

import java.util.Arrays;
import java.util.Comparator;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;

public class VisionFilter {

    class VisionData {
        Pose2d pose;
        double timeStamp;
        double offset;
        VisionData(Pose2d p, double t) {
            pose = p;
            timeStamp = t;
            if(t > 0) {
                setOffset();
            } else {
                clear();
            }
        }
        void recalc(double time) {
            // for newer data - recalc the twsit
            if(timeStamp > time) {
                setOffset();
            } else {
                clear();
            }
        }

        void setOffset() {
            Pose2d pp = poseEstimator.getSample(timeStamp);
            if(pp != null) {
                offset = pp.getTranslation().getDistance(pose.getTranslation());
            } else {
                clear();
            }
        }

        void clear() {
            offset = -1;
            pose = null;
            timeStamp = 0;
        }
    }

    PoseEstimator poseEstimator;
    VisionData[] buf = new VisionData[3];
    int lastData = -1;
    double lastUpdateTime;

    int next() {
        return (lastData+1)%buf.length;
    }

    public VisionFilter(PoseEstimator estimator) {
        poseEstimator = estimator;
        for(int i = 0; i < buf.length; i++) {
            buf[i] = new VisionData(null, 0);
        }
    }

    public double getTime() {
        return MathSharedStore.getTimestamp();
    }

    public void update(Pose2d pose, double latency) {
        double time = getTime();
        lastData = next();
        buf[lastData] = new VisionData(pose, time-latency);
        if(validBuf(time)) {
            VisionData v = median();
            if(v != null && v.pose != null) {
                poseEstimator.addVisionMeasurement(v.pose, getTime()-v.timeStamp);
                lastUpdateTime = time;
                time = v.timeStamp;
                for(VisionData vd : buf) {
                    vd.recalc(time);
                }
            }
        }
    }

    Comparator<VisionData> comperator = new Comparator<VisionData>() {
        @Override
        public int compare(VisionData arg0, VisionData arg1) {
            return Double.compare(arg0.offset, arg1.offset);
        }
    };

    private boolean validBuf(double time) {
        double minTime = time - 1.2;
        for(VisionData v : buf) {
            if(v.timeStamp < minTime) {
                return false;
            }
        }
        return true;
    }

    private VisionData median() {
        // create array of Visions
        VisionData[] v = new VisionData[buf.length];
        v = buf.clone();
        Arrays.sort(v,comperator);
        return v[buf.length/2];
    }

    public double lastUpdateLatency() {
        return getTime() - lastUpdateTime;
    }

    public boolean validVisionPosition() {
        return lastUpdateLatency() < 1;
    }
}
