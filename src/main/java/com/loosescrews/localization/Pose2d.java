package com.loosescrews.localization;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class Pose2d {
    public double x,y,theta;

    public Pose2d() {
        this(0,0,0);
    }
    public Pose2d(double x, double y) {
        this(x,y,0);
    }

    public Pose2d(Vec2d vec, double heading) {
        this(vec.x, vec.y, heading);
    }

    public Pose2d(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Vec2d headingVec() {
        return Vec2d.fromCartesian(cos(theta), sin(theta));
    }

    public Vec2d vec() {
        return Vec2d.fromCartesian(x, y);
    }

    public Pose2d minus(Pose2d other) {
        return new Pose2d(x - other.x, y - other.y, theta - other.theta);
    }

    public double norm() {
        return vec().norm();
    }

    public Pose2d mul(double num) {
        return new Pose2d(x*num, y*num, theta * num);
    }
    public Pose2d mulVec(double num) {
        return new Pose2d(x*num, y*num, theta);
    }
    public Pose2d div(double num) {
        return new Pose2d(x/num, y/num, theta/num);
    }


    public double getX() {return x;}
    public double getY() {return y;}
    public double getHeading() {return theta;}

}

