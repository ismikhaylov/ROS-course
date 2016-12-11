package com.kodoo.spb;

import com.fasterxml.jackson.databind.JsonNode;
import com.kodoo.spb.astar.BZAstar;
import com.kodoo.spb.astar.MapPoint;
import ros.Publisher;
import ros.RosBridge;

import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.Deque;
import java.util.Iterator;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class Listener {
    public static final int MAP_SIZE = 100;
    public static final MapPoint DESTINATION = new MapPoint(20, 20);
    public static final double CELL_SIZE = 0.5;
    public static final int MAP_OFFSET = 20;

    public static final double ROBOT_OFFSET_X = -7;
    public static final double ROBOT_OFFSET_Y = -6;


    static volatile double robotX = 0;
    static volatile double robotY = 0;
    static volatile double robotAng = 3.14;

    static volatile double rangeMax;
    private static String bridge_addr;

    public static void main(String[] args) throws InterruptedException, IOException {

        if(args.length != 1){
            System.out.println("Need the rosbridge websocket URI provided as argument");
            System.exit(0);
        }

        bridge_addr = args[0];

        ExecutorService executorService = Executors.newFixedThreadPool(4);

        Mover mover = new Mover();

        BZAstar pathFinder = new BZAstar(MAP_SIZE, MAP_SIZE, new boolean[][] {});
        WorldMap map = new WorldMap(CELL_SIZE, MAP_SIZE, MAP_OFFSET, MAP_OFFSET);
        map.setWallAddedHandler(worldMap -> {
            pathFinder.walls(map.cells);
            Deque<MapPoint> path = pathFinder.calculateAStarNoTerrain(new MapPoint((int) (robotX / CELL_SIZE), (int) (robotY / CELL_SIZE)), DESTINATION);
            map.clearPath();
            for (MapPoint point : path) {
                map.path(point.x, point.y);
            }
            mover.moveThatWay(path);
        });

        executorService.execute(() -> {
            RosBridge bridge = RosBridge.createConnection(bridge_addr);
            bridge.waitForConnection();
            bridge.subscribe("/base_pose_ground_truth", "nav_msgs/Odometry",
                    (data, stringRep) -> {
                        robotX += getDoubleFromJson(data, "linear", "x")/10;
                        robotY += getDoubleFromJson(data, "linear", "y")/10;
                        robotAng += getDoubleFromJson(data, "angular", "z")/10;

//                        System.out.printf("Robot now: (%1.3f, %1.3f) ang: %1.3f\n", robotX, robotY, robotAng);
                    }
            );
        });

        executorService.execute(() -> {
            RosBridge bridge = RosBridge.createConnection(bridge_addr);
            bridge.waitForConnection();
            bridge.subscribe("/base_scan", "sensor_msgs/LaserScan",
                    (data, stringRep) -> {
                        JsonNode ranges = data.findValue("ranges");
                        rangeMax = getDoubleFromJson(data, "range_max");
                        double curAng = robotAng + getDoubleFromJson(data, "angle_min");
                        double delta = getDoubleFromJson(data, "angle_increment");
                        Iterator<JsonNode> elements = ranges.elements();
                        while (elements.hasNext()) {
                            JsonNode jsonNode = elements.next();
                            double range = Double.parseDouble(jsonNode.toString());
                            if (range > 0 && range < rangeMax) {
                                map.shoot(
                                        range * Math.cos(curAng) + robotX,
                                        range * Math.sin(curAng) + robotY
                                );
                            }
                            curAng += delta;
                        }
                    }
            );
        });

        executorService.execute(mover);

        executorService.execute(() -> {
            RosBridge bridge = RosBridge.createConnection(bridge_addr);
            bridge.waitForConnection();
            bridge.subscribe("/cmd_vel", "geometry_msgs/Twist",
                    (data, stringRep) -> {
//                        System.out.printf("From cmd_vel: %s\n", data);
                    }
            );
        });

        Executors.newSingleThreadScheduledExecutor().scheduleWithFixedDelay(() -> {
            try {
                map.mark(robotX, robotY);
                map.print(new OutputStreamWriter(System.out));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }, 10, 10, TimeUnit.SECONDS);

        executorService.awaitTermination(20, TimeUnit.SECONDS);
        executorService.shutdown();
    }

    static class Mover implements Runnable {

        private static final double DELTA = 0.05;
        private static final double ANG_DELTA = 0.05;

        private volatile Deque<MapPoint> mapPoints;

        void moveThatWay(Deque<MapPoint> mapPoints) {
            System.out.printf("update A* path: %s\n", mapPoints);
            this.mapPoints = mapPoints;
        }

        @Override
        public void run() {
            while (true) {
                RosBridge bridge = RosBridge.createConnection(bridge_addr);
                bridge.waitForConnection();

                Publisher pub = new Publisher("/cmd_vel", "geometry_msgs/Twist", bridge);

                if (mapPoints == null)
                    continue;

                MapPoint nextPointToMove = mapPoints.pollLast();

                double x = nextPointToMove.x * CELL_SIZE;
                double y = nextPointToMove.y * CELL_SIZE;

                System.out.printf("To the point %1.4f, %1.4f\n", x, y);

                //крутим к целе
                int sign = 1;
                if (robotAng > Math.atan2(y - robotY, x - robotX)) {
                    sign = -1;
                }
                while (Math.abs(Math.atan2(y - robotY, x - robotX) - robotAng) > ANG_DELTA * 1.1) {
                    pub.publishJsonMsg("{\"linear\":{\"y\":0.0,\"x\":0.0,\"z\":0.0},\"angular\":{\"y\":0.0,\"x\":0.0,\"z\":" + ANG_DELTA * sign +"}}");
//                    System.out.printf("ROLL IT ((%1.3f, %1.3f)%1.5f)\n",
//                            Math.atan2(y - robotY, x - robotX),
//                            robotAng,
//                            Math.abs(Math.atan2(y - robotY, x - robotX) - Math.toDegrees(robotAng)%360));
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                while (Math.abs(y - robotY) > DELTA * 1.1 || Math.abs(x - robotX) > DELTA * 1.1) {
                    pub.publishJsonMsg("{\"linear\":{\"y\":0.0,\"x\":" + DELTA + ",\"z\":0.0},\"angular\":{\"y\":0.0,\"x\":0.0,\"z\":0.0}}");
//                    System.out.printf("Go go go x delta = %1.3f, y_delta = %1.3f\n", Math.abs(x - robotX), Math.abs(y - robotY));
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }

                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }


    static double getDoubleFromJson(JsonNode node, String ... values) {
        JsonNode aimNode = node;
        for (int i = 0; i < values.length; i++) {
            aimNode = aimNode.findValue(values[i]);
        }
        return Double.parseDouble(aimNode.toString());
    }
}
