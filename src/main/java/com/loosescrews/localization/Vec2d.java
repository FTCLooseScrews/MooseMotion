package com.loosescrews.localization;

import com.loosescrews.util.Angle;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class Vec2d {
    public double x, y;
    public double mag, theta;

    public Vec2d(double mag, double theta) {
        this.mag = mag;
        this.theta = theta;

        this.x = mag * cos(theta);
        this.y = mag * sin(theta);
    }

    public static Vec2d fromCartesian(double x, double y) {
        return new Vec2d(Math.sqrt(x*x+y*y), Angle.norm(Math.atan2(y, x)));
    }

    public Vec2d() {
        this(0, 0);
    }

    public Vec2d rotated(double angle) {
        return Vec2d.fromCartesian(x * cos(angle) - y * sin(angle), x * sin(angle) + y * cos(angle));
    }

    public double dotProduct(Vec2d other) {
        return x*other.x + y*other.y;
    }

    public double getAngle() {
        return theta;
    }

    public void setMagnitude(double magnitude) {
        this.mag = magnitude;

        this.x = magnitude * cos(theta);
        this.y = magnitude * sin(theta);
    }

    public double distTo(Vec2d other) {
        return Vec2d.fromCartesian(other.x - x, other.y - y).norm();
    }

    public double norm() {
        return mag;
    }

    public Vec2d mul(double scalar) {
        return new Vec2d(mag*scalar, theta);
    }

    public Vec2d plus(Vec2d other) {
        return Vec2d.fromCartesian(other.x + x, other.y + y);
    }

    public Vec2d minus(Vec2d other) {
        return Vec2d.fromCartesian(x - other.x, y - other.y);
    }
}

