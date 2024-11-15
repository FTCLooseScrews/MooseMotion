package com.loosescrews.path;

import com.loosescrews.localization.Pose2d;
import com.loosescrews.localization.Vec2d;

public class CubicBezier extends ParametricCurve {
    private final Vec2d fourth;
    private final CubicBezierCurve x;
    private final CubicBezierCurve y;

    public CubicBezier(Vec2d first, Vec2d second, Vec2d third, Vec2d fourth) {
//        this.first = first;
//        this.second = second;
//        this.third = third;
        this.fourth = fourth;

        x = new CubicBezierCurve(first.x, second.x, third.x, fourth.x);
        y = new CubicBezierCurve(first.y, second.y, third.y, fourth.y);

        for (int i = 1; i <= 1000; i++) {
            waypoints.add(new Waypoint(Vec2d.fromCartesian(x.get(i/1000.0), y.get(i/1000.0)), i/1000.0));
        }
    }
    @Override
    public Vec2d end() {
        return fourth;
    }

    @Override
    public Waypoint getNextWaypoint(Pose2d pose, Pose2d last) {
        ProjectedPoint point = ProjectedPoint.projectFrom(this, pose);

        if (point.getT() == 0) return waypoints.get(0);

        for (int i = 1; i < waypoints.size(); i++) {
            if (waypoints.get(i).getT() >= point.getT()) {
                if (i >= 995) {
                    return null;
                }
                if (waypoints.get(i).getWaypointVec().distTo(pose.vec()) <=
                        waypoints.get(i-1).getWaypointVec().distTo(pose.vec())) {
                    return waypoints.get(i == waypoints.size()-1 ? i : i+1);
                }
                return waypoints.get(i);
            }
        }

        return null;
    }
    public Vec2d get(double t) {
        return Vec2d.fromCartesian(x.get(t), y.get(t));
    }

    public Vec2d deriv(double t) {
        return Vec2d.fromCartesian(x.deriv(t), y.deriv(t));
    }

    public Vec2d secondDeriv(double t) {
        return Vec2d.fromCartesian(x.secondDeriv(t), y.secondDeriv(t));
    }

    public Vec2d getSecondDerivHeading(double t) {
        double derivAngle = deriv(t).getAngle();
        double deltaAngle = deriv(t+0.001).getAngle();

        return new Vec2d(1, deltaAngle-derivAngle);
    }

    public double length() {
        return lengthFromWaypoint(0);
    }

    public double lengthFromWaypoint(int wpIndex) {
        if (wpIndex == waypoints.size() || wpIndex == waypoints.size()-1) return 0;

        double sum = 0;
        for (int i = wpIndex+1; i < waypoints.size(); i++) {
            sum += waypoints.get(i).getWaypointVec().distTo(waypoints.get(i-1).getWaypointVec());
        }
        return sum;
    }

    public double getCurvature(double t) {
        t = Math.min(Math.max(0, t), 1);
        Vec2d deriv = deriv(t);
        Vec2d secondDeriv = secondDeriv(t);

        if (deriv.norm() == 0) return 0;
        return (deriv.x*secondDeriv.y-deriv.y-secondDeriv.x)/Math.pow(deriv.norm(), 3);
    }

    private static class CubicBezierCurve {
        public double c1,c2,c3,c4;

        public CubicBezierCurve(double a, double b, double c, double d) {
            this.c1 = a;
            this.c2 = b;
            this.c3 = c;
            this.c4 = d;
        }

        public double get(double t) {
            return Math.pow(1-t, 3) * c1 +
                    3*t*(1-t)*(1-t) * c2 +
                    3*t*t*(1-t) * c3 +
                    t*t*t*c4;
        }

        public double deriv(double t) {
            return -3 * c1 * (1-t) * (1-t) +
                    3 * c2 * (1-t) * (1-t) + -6 * t * c2 * (1-t) +
                    6 * t * (1-t) * c3 + -3 * t * t * c3 +
                    3 * t * t * c4;
        }

        public double secondDeriv(double t) {
            return 6 * c1 * (1-t) +
                    -6 * c2 * (1-t) + -6 * c2 * (1-t) + 6 * t * c2 +
                    6 * c3 * (1-t) - 6 * c3 * t - 6 * t * c3 +
                    6 * t * c4;
        }
    }
}
