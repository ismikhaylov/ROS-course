package com.kodoo.spb;

import ros.Publisher;
import ros.RosBridge;

import java.util.HashMap;
import java.util.Map;

public class Talker {
    public static void main(String[] args) {

        if(args.length != 1){
            System.out.println("Need the rosbridge websocket URI provided as argument");
            System.exit(0);
        }

        RosBridge bridge = RosBridge.createConnection(args[0]);
        bridge.waitForConnection();

        Publisher pub = new Publisher("/cmd_vel", "geometry_msgs/Twist", bridge);
        final Map<String, String> strData = new HashMap<String, String>();

        for(int i = 0; i < 100; i++) {
            strData.put("data", "hello from java " + i);
            System.out.println("sending...");
            pub.publish(strData);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
