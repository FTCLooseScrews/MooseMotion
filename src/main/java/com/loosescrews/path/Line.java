package com.loosescrews.path;

import com.loosescrews.localization.Pose2d;
import com.loosescrews.localization.Vec2d;

public class Line extends ParametricCurve {
    private double x1, y1, x2, y2;

    public Line(double x1, double y1, double x2, double y2) {
        this.x1 = x1;
        this.y2 = y2;
        this.x2 = x2;
        this.y1 = y1;

        Vec2d line = Vec2d.fromCartesian(x2-x1, y2-y1);
        for (int i = 1; i <= 1000; i++) {
            waypoints.add(new Waypoint(x1 + line.x * i/1000.0, y1 + line.y * i/1000.0, i/1000.0));
        }
    }

    @Override
    public Vec2d end() {
        return Vec2d.fromCartesian(x2, y2);
    }

    public Vec2d get(double t) {
        return Vec2d.fromCartesian((x2-x1)*t+x1, (y2-y1)*t+y1);
    }
    public Vec2d deriv(double t) {
        return Vec2d.fromCartesian(x2-x1, y2-y1);
    }
    public Vec2d secondDeriv(double t) {
        return Vec2d.fromCartesian(0, 0);
    }

    public Vec2d getSecondDerivHeading(double t) {
        return new Vec2d();
    }

    public double getCurvature(double t) {
        return 0.0d;
    }

    @Override
    public Waypoint getNextWaypoint(Pose2d pose, Pose2d last) {
        if (last == null) {
            return waypoints.get(0);
        }
        for (Waypoint waypoint : waypoints) {
            if (waypoint.getWaypointVec().distTo(Vec2d.fromCartesian(x1, y1)) > pose.vec().distTo(Vec2d.fromCartesian(x1, y1))) {
                return waypoint;
            }
        }
        return null;
    }

    @Override
    public double length() {
        return Vec2d.fromCartesian(x2-x1, y2-y1).norm();
    }

    public double lengthFromWaypoint(int wpIndex) {
        Vec2d waypoint = waypoints.get(wpIndex).getWaypointVec();
        return Vec2d.fromCartesian(x2 - waypoint.x, y2 - waypoint.y).norm();
    }
}
