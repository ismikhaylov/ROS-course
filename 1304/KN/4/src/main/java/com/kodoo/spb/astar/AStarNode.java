package com.kodoo.spb.astar;

public class AStarNode {
    public final MapPoint point;

    public AStarNode parent;

    public int gValue; //points from start
    public int hValue; //distance from target
    public boolean isWall = false;

    private final int MOVEMENT_COST = 10;

    public AStarNode(MapPoint point) {
        this.point = point;
    }

    /**
     * Used for setting the starting node value to 0
     */
    public void setGValue(int amount) {
        this.gValue = amount;
    }

    public void calculateHValue(AStarNode destPoint) {
        this.hValue = (Math.abs(point.x - destPoint.point.x) + Math.abs(point.y - destPoint.point.y)) * this.MOVEMENT_COST;
    }

    public void calculateGValue(AStarNode point) {
        this.gValue = point.gValue + this.MOVEMENT_COST;
    }

    public int getFValue() {
        return this.gValue + this.hValue;
    }
}
