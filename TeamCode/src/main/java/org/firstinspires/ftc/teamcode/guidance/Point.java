package org.firstinspires.ftc.teamcode.guidance;

/**
 * Utility class for representing a 2D coordinate point and providing
 * a rotation operation for frame coordination conversion.
 */
public class Point {
    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }
    public double x = 0d;
    public double y = 0d;

    public Point rotate(double theta){
        double rx = Math.cos(theta) * x - Math.sin(theta) * y;
        double ry = Math.sin(theta) * x + Math.cos(theta) * y;
        return new Point(rx,ry);
    }

}
