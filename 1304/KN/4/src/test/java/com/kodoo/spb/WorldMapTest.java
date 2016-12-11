package com.kodoo.spb;

import org.testng.Assert;
import org.testng.annotations.Test;

public class WorldMapTest {

    @Test
    public void testShoot() {
        WorldMap map = new WorldMap(0.5, 4, 1, 1);
        map.shoot(0.4, 0.4);
        Assert.assertTrue(map.cells[2][2]);

        map.shoot(-0.4, -0.4);
        Assert.assertTrue(map.cells[1][1]);

        map.shoot(-0.3, -0.6);
        Assert.assertTrue(map.cells[0][1]);
    }
}