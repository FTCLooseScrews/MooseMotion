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

    public abstract Vec2d getNextWaypoint(Pose2d pose, Pose2d last);

    public List<Waypoint> getWaypoints() {
        return waypoints;
    }

    public static class Waypoint {
        private Vec2d wp;
        public Waypoint(Vec2d a) {
            this.wp = a;
        }

        public Waypoint(double x, double y) {
            this(Vec2d.fromCartesian(x, y));
        }

        public Vec2d getWaypointVec() {
            return wp;
        }
    }
}
