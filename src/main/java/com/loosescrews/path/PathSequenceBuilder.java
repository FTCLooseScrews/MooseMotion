package com.loosescrews.path;

import com.loosescrews.localization.Pose2d;
import com.loosescrews.localization.Vec2d;
import com.loosescrews.path.exceptions.EmptyPathSequenceException;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PathSequenceBuilder {
    private ArrayList<Path> sequence;
    private Pose2d lastPose;

    public PathSequenceBuilder(Pose2d startPose) {
        sequence = new ArrayList<>();
        lastPose = startPose;
    }

    public PathSequenceBuilder lineTo(Vec2d endPosition) {
        Line l = new Line(lastPose.x, lastPose.y, endPosition.x, endPosition.y);
        sequence.add(new Path(l).setTangentHeadingInterpolation());

        lastPose = new Pose2d(endPosition, l.deriv(1).getAngle());

        return this;
    }

    public PathSequenceBuilder lineToConstantHeading(Vec2d endPosition, double endHeading) {
        sequence.add(new Path(new Line(lastPose.x, lastPose.y, endPosition.x, endPosition.y))
                .setConstantHeadingInterpolation(endHeading)
        );

        lastPose = new Pose2d(endPosition, endHeading);

        return this;
    }

    public PathSequenceBuilder lineToLinearHeading(Vec2d endPosition, double endHeading) {
        sequence.add(new Path(new Line(lastPose.x, lastPose.y, endPosition.x, endPosition.y))
                .setLinearHeadingInterpolation(lastPose.theta, endHeading)
        );

        lastPose = new Pose2d(endPosition, endHeading);

        return this;
    }

    public PathSequenceBuilder cubicBezierSplineTo(Vec2d c2, Vec2d c3, Vec2d end) {
        CubicBezier spline = new CubicBezier(lastPose.vec(), c2, c3, end);
        sequence.add(new Path(spline).setTangentHeadingInterpolation());

        lastPose = new Pose2d(end, spline.deriv(1).getAngle());

        return this;
    }

    public PathSequenceBuilder cubicBezierSplineToConstantHeading(Vec2d c2, Vec2d c3, Vec2d end, double endHeading) {
        CubicBezier spline = new CubicBezier(lastPose.vec(), c2, c3, end);
        sequence.add(new Path(spline).setConstantHeadingInterpolation(endHeading));

        lastPose = new Pose2d(end, endHeading);

        return this;
    }

    public PathSequenceBuilder cubicBezierSplineToLinearHeading(Vec2d c2, Vec2d c3, Vec2d end, double endHeading) {
        CubicBezier spline = new CubicBezier(lastPose.vec(), c2, c3, end);
        sequence.add(new Path(spline).setLinearHeadingInterpolation(lastPose.theta, endHeading));

        lastPose = new Pose2d(end, endHeading);

        return this;
    }

    public PathSequenceBuilder bezierSplineTo(Vec2d... points) {
        if (points.length == 0) throw new EmptyPathSequenceException();
        Bezier spline = new Bezier(addStartingPose(points));
        sequence.add(new Path(spline).setTangentHeadingInterpolation());

        lastPose = new Pose2d(points[points.length-1], spline.deriv(1).getAngle());

        return this;
    }

    public PathSequenceBuilder bezierSplineToConstantHeading(double endHeading, Vec2d... points) {
        if (points.length == 0) throw new EmptyPathSequenceException();
        Bezier spline = new Bezier(addStartingPose(points));
        sequence.add(new Path(spline).setConstantHeadingInterpolation(endHeading));

        lastPose = new Pose2d(points[points.length-1], endHeading);

        return this;
    }

    public PathSequenceBuilder bezierSplineToLinearHeading(double endHeading, Vec2d... points) {
        if (points.length == 0) throw new EmptyPathSequenceException();
        Bezier spline = new Bezier(addStartingPose(points));
        sequence.add(new Path(spline).setLinearHeadingInterpolation(lastPose.theta, endHeading));

        lastPose = new Pose2d(points[points.length-1], endHeading);

        return this;
    }


    // Do not use if this is not the first method in your path sequence.
    public PathSequenceBuilder addPath(Path path) {
        sequence.add(path);

        lastPose = path.end();

        return this;
    }

    private Vec2d[] addStartingPose(Vec2d[] controlPoints) {
        Vec2d[] ret = new Vec2d[controlPoints.length+1];
        ret[0] = lastPose.vec();
        System.arraycopy(controlPoints, 0, ret, 1, controlPoints.length);
        return ret;
    }

    public PathSequence build() {
        return new PathSequence(sequence);
    }
}
