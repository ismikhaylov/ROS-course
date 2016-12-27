package com.kodoo.spb.astar;

/**
 * Created by kodoo on 22.11.16.
 */
public class MapPoint {
    public int x;
    public int y;

    public MapPoint(int x, int y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public boolean equals(Object obj) {
        return x == ((MapPoint) obj).x && y == ((MapPoint) obj).y;
    }

    @Override
    public int hashCode() {
        return x + 11133 * y;
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}
