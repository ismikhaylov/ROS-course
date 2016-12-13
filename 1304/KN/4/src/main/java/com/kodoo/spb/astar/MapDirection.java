package com.kodoo.spb.astar;

/**
 * Created by kodoo on 22.11.16.
 */
public enum MapDirection {
    UP, DOWN, LEFT, RIGHT;

    public MapPoint getPointForDirection(MapPoint point) {
        switch (this) {
            case UP:    return new MapPoint(point.x, point.y + 1);
            case DOWN:  return new MapPoint(point.x, point.y - 1);
            case LEFT:  return new MapPoint(point.x - 1, point.y);
            case RIGHT:  return new MapPoint(point.x + 1, point.y);
        }
        return null;
    }
}
