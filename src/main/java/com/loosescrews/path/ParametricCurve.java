package com.loosescrews.path;

import com.loosescrews.localization.Pose2d;
import com.loosescrews.localization.Vec2d;

import java.util.ArrayList;
import java.util.List;

public abstract class ParametricCurve {
    protected List<Waypoint> waypoints = new ArrayList<>();

    public abstract Vec2d end();

    public abstract Vec2d deriv(double t);
    public abstract Vec2d secondDeriv(double t);

    public abstract double getCurvature(double t);

    public abstract Vec2d getSecondDerivHeading(double t);

    public abstract Vec2d get(double t);

    public abstract Waypoint getNextWaypoint(Pose2d pose, Pose2d last);

    public abstract double length();
    public abstract double lengthFromWaypoint(int wpIndex);

    public List<Waypoint> getWaypoints() {
        return waypoints;
    }

    public static class Waypoint {
        private Vec2d wp;
        private double t;
        public Waypoint(Vec2d a, double t) {
            this.wp = a;
            this.t = t;
        }

        public Waypoint(double x, double y, double t) {
            this(Vec2d.fromCartesian(x, y), t);
        }

        public Vec2d getWaypointVec() {
            return wp;
        }
        public double getT() {
            return t;
        }
    }
}
