package com.loosescrews.path;

import com.loosescrews.path.exceptions.EmptyPathSequenceException;

import java.util.ArrayList;

public class PathSequence {
    private ArrayList<Path> sequence;

    public PathSequence(ArrayList<Path> sequence) {
        if (sequence.isEmpty()) throw new EmptyPathSequenceException();
        this.sequence = sequence;
    }

    public Path get(int i) {
        return sequence.get(i);
    }

    public int size() {
        return sequence.size();
    }
}
