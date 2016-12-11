package com.kodoo.spb.astar;

import java.util.*;

/**
 * Created by kodoo on 22.11.16.
 */
public class BZAstar {
    private final int width;
    private final int height;

    private final Map<MapPoint, AStarNode> nodes = new HashMap<MapPoint, AStarNode>();

    @SuppressWarnings("rawtypes")
    private final Comparator fComparator = new Comparator<AStarNode>() {
        public int compare(AStarNode a, AStarNode b) {
            return Integer.compare(a.getFValue(), b.getFValue()); //ascending to get the lowest
        }
    };

    public BZAstar(int width, int height, boolean[][] wallPositions) {
        this.width = width;
        this.height = height;

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                MapPoint point = new MapPoint(x, y);
                this.nodes.put(point, new AStarNode(point));
            }
        }

        walls(wallPositions);
    }

    public void walls(boolean[][] wallPositions) {
        for (int y = 0; y < wallPositions.length; y++) {
            for (int x = 0; x < wallPositions.length; x++) {
                if (wallPositions[y][x] == false)
                    continue;

                AStarNode node = this.nodes.get(new MapPoint(x, wallPositions.length - 1 - y));
                node.isWall = true;
            }
        }
    }

    @SuppressWarnings("unchecked")
    public Deque<MapPoint> calculateAStarNoTerrain(MapPoint p1, MapPoint p2) {

        List<AStarNode> openList = new ArrayList<AStarNode>();
        List<AStarNode> closedList = new ArrayList<AStarNode>();

        AStarNode destNode = this.nodes.get(p2);

        AStarNode currentNode = this.nodes.get(p1);
        currentNode.parent = null;
        currentNode.setGValue(0);
        openList.add(currentNode);

        while(!openList.isEmpty()) {

            Collections.sort(openList, this.fComparator);
            currentNode = openList.get(0);

            if (currentNode.point.equals(destNode.point)) {
                return this.calculatePath(destNode);
            }

            openList.remove(currentNode);
            closedList.add(currentNode);

            for (MapDirection direction : MapDirection.values()) {
                MapPoint adjPoint = direction.getPointForDirection(currentNode.point);

                if (!this.isInsideBounds(adjPoint)) {
                    continue;
                }

                AStarNode adjNode = this.nodes.get(adjPoint);
                if (adjNode.isWall) {
                    continue;
                }

                if (!closedList.contains(adjNode)) {
                    if (!openList.contains(adjNode)) {
                        adjNode.parent = currentNode;
                        adjNode.calculateGValue(currentNode);
                        adjNode.calculateHValue(destNode);
                        openList.add(adjNode);
                    } else {
                        if (adjNode.gValue < currentNode.gValue) {
                            adjNode.calculateGValue(currentNode);
                            currentNode = adjNode;
                        }
                    }
                }
            }
        }

        return null;
    }

    private Deque<MapPoint> calculatePath(AStarNode destinationNode) {
        Deque<MapPoint> path = new ArrayDeque<>();
        AStarNode node = destinationNode;
        while (node.parent != null) {
            path.add(node.point);
            node = node.parent;
        }
        return path;
    }

    private boolean isInsideBounds(MapPoint point) {
        return point.x >= 0 &&
                point.x < this.width &&
                point.y >= 0 &&
                point.y < this.height;
    }

    public static void main(String[] args) {
        List<MapPoint> walls = new ArrayList<MapPoint>();
        walls.add(new MapPoint(1, 6));
        walls.add(new MapPoint(2, 5));
        walls.add(new MapPoint(4, 6));
        walls.add(new MapPoint(5, 5));
        walls.add(new MapPoint(6, 5));
        walls.add(new MapPoint(7, 5));
        walls.add(new MapPoint(7, 6));
        walls.add(new MapPoint(7, 7));

        BZAstar alg = new BZAstar(10, 10, new boolean[][] {
                {false, false, false, false, false, false, false, false, false, false},
                {false, false, false, false, false, false, false, false, false, false},
                {false, false, false, false, false, false, false, true, false, false},
                {false, true, false, false, true, false, false, true, true, false},
                {false, false, true, false, false, true, true, true, false, false},
                {false, false, false, false, false, false, false, false, false, false},
                {false, false, false, false, false, false, false, false, false, false},
                {false, false, false, false, false, false, false, true, false, false},
                {false, false, false, false, false, false, false, false, false, false},
                {false, false, false, false, false, false, false, false, false, false},
        });
        alg.nodes.forEach((node, value) -> {
            if (value.isWall)
                System.out.println(node);
        });
        System.out.println(alg.calculateAStarNoTerrain(new MapPoint(1, 5), new MapPoint(8, 9)));
    }
}
