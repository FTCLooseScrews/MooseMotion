package com.loosescrews.follower;


public class MotionProfile {
    double maxAcceleration;
    double maxVelocity;
    double dist;
    double t1;
    public MotionProfile (double dist, double maxAcceleration, double maxVelocity) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        this.dist = dist;
        this.t1 = maxAcceleration*1.0/3 <= maxVelocity ? 1.0/3 : maxVelocity / maxAcceleration;
    }

    private double getVelocityParametric(double time) {
        double t = Math.max(Math.min(time, 1), 0);
        if (t <= t1) {
            return maxAcceleration * t;
        }

        if (t <= (1-t1) && (t >= t1)) {
            return maxAcceleration * t1;
        }

        if (t <= 1 && t >= (1-t1)) {
            return -maxAcceleration * t + maxAcceleration;
        }

        return 0;
    }

    public double getVelocity(double distanceDriven) {
        return getVelocityParametric(distanceDriven / dist);
    }


}

