package com.loosescrews.util;

public abstract class NanoClock {
    public abstract double seconds();

    public static NanoClock system() {
        return new NanoClock() {
            @Override
            public double seconds() {
                return System.nanoTime() / 1e9;
            }
        };
    }
}

