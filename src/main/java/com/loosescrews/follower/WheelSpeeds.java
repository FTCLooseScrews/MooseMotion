package com.loosescrews.follower;

public class WheelSpeeds {

    // in/s
    private double fL, fR, bL, bR;

    public WheelSpeeds(double fL, double fR, double bL, double bR) {
        this.fL = fL;
        this.fR = fR;
        this.bL = bL;
        this.bR = bR;
    }

    public double getfL() {
        return fL;
    }

    public double getfR() {
        return fR;
    }

    public double getbL() {
        return bL;
    }

    public double getbR() {
        return bR;
    }
}
