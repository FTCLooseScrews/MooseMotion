package com.loosescrews.util;

import static java.lang.Math.PI;

public class Angle {
    private static final double TAU = 2 * PI;

    public static double norm(double angle) {
        return (((angle % TAU) + TAU) % TAU);
    }

    public static double normDelta(double angleDelta) {
        return norm(angleDelta) > PI ? norm(angleDelta) - TAU : norm(angleDelta);
    }

    public static double smallestDifference(double a, double b) {
        return Math.min(norm(a-b), norm(b-a));
    }

    public static double getRotationSide(double a, double b) {
        double diff = norm(b-a);
        return (diff >= 0 && diff <= PI ? 1 : -1);
    }

}