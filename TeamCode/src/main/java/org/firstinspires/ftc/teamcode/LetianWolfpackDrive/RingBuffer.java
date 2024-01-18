package org.firstinspires.ftc.teamcode.LetianWolfpackDrive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * A buffer of predetermined size. New values will override the oldest value if capacity is reached.
 */
public class RingBuffer {
    private final Pose2d[] array;
    private int head = -1; // read head, aka index of most recent element

    public RingBuffer(int capacity) {
        array = new Pose2d[capacity];
    }

    /**
     * Reads elements from buffer sorted by recency.
     * @param index Should be between 0 and capacity, where 0 returns most recently added element.
     * @return null if not added
     */
    public Pose2d read(int index) {
        // throw if out of bounds
        if (index < 0 || index >= array.length) {
            throw new IndexOutOfBoundsException();
        }

        // if empty, return null
        if (head == -1) {
            return null;
        }

        // otherwise, return element
        return array[(head - index + array.length) % array.length];
    }

    /**
     * Adds elements into ring buffer
     * @param element a new element to add into buffer
     */
    public void put(@NonNull Pose2d element) {
        head = (head + 1) % array.length;
        array[head] = element;
    }

    /**
     * Returns initialization capacity of ring buffer
     */
    public int capacity() {
        return array.length;
    }

    /**
     * Returns the number of elements in the buffer.
     * This number can never exceed capacity.
     */
    public int size() {
        int size = 0;
        for (Pose2d element : array) {
            if (element != null) size++;
            else break;
        }
        return size;
    }

    /**
     * Returns whether the buffer is completely full.
     */
    public boolean isFull() {
        for (Pose2d element : array) {
            if (element == null) return false;
        }
        return true;
    }

    @NonNull
    @Override
    public String toString() {
        if (size() == 0) return "[]";

        int readHead = head == -1 ? 0 : head; // if empty, read from 0

        StringBuilder b = new StringBuilder();
        b.append('[');

        for (int i = readHead; ; i = (i - 1 + array.length) % array.length) {
            if (array[i] == null) b.append("null");
            else b.append(String.format("(%.2f, %.2f, %.2f°)", array[i].position.x, array[i].position.y, Math.toDegrees(array[i].heading.log())));
            if ((i - 1 + array.length) % array.length == readHead) // check if i is last element
                return b.append(']').toString();
            b.append(", ");
        }
    }
}
