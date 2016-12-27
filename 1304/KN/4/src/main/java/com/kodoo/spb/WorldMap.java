package com.kodoo.spb;

import java.io.IOException;
import java.io.Writer;
import java.util.function.Consumer;

public class WorldMap {

    private double cellSize;
    boolean[][] cells;
    private boolean[][] path;
    private double mapOffsetX;
    private double mapOffsetY;
    private Consumer<WorldMap> handler;
    private int markX;
    private int markY;

    WorldMap(double cellSize, int mapSize, double mapOffsetX, double mapOffsetY) {
        this.cellSize = cellSize;
        this.cells = new boolean[mapSize][mapSize];
        this.path = new boolean[mapSize][mapSize];
        this.mapOffsetX = mapOffsetX;
        this.mapOffsetY = mapOffsetY;

    }

    public void shoot(double x, double y) {
        int i = (int) Math.floor((y + mapOffsetY) / cellSize);
        int j = (int) Math.floor(((x + mapOffsetX) / cellSize));

        if (cells[i][j] == false && handler != null) {
            handler.accept(this);
        }

        cells[i][j] = true;
    }

    public void mark(double x, double y) {
        markY = (int) Math.floor((y + mapOffsetY) / cellSize);
        markX = (int) Math.floor(((x + mapOffsetX) / cellSize));
    }

    public void clearPath() {
        for (int y = cells.length - 1; y >= 0; y--) {
            for (int x = 0; x < cells.length; x++) {
                path[y][x] = false;
            }
        }
    }

    public void path(double x, double y) {
        int y_ = (int) Math.floor(y + markY);
        int x_ = (int) Math.floor(x + markX);
        path[y_][x_] = true;
    }

    public void print(Writer out) throws IOException {
        for (int y = cells.length - 1; y >= 0; y--) {
            for (int x = 0; x < cells.length; x++) {
                if (x == markX && y == markY) {
                    out.write("@");
                    continue;
                }

                if (path[y][x]) {
                    out.write(".");
                    continue;
                }

                if (cells[y][x]) {
                    out.write("#");
                } else {
                    out.write(" ");
                }
            }
            out.write("\n");
        }
    }

    public void setWallAddedHandler(Consumer<WorldMap> handler) {
        this.handler = handler;
    }
}
