package frc.team670.robot.pathfinder;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.awt.Point;
import java.awt.Polygon;
import java.awt.Rectangle;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.Serializable;

/**
 * Polygon represnting an obstacle. Credits to Hemlock.
 * @author ethan c Dx
 */
public class Obstacle {
    PolygonDouble polygon;

    public Obstacle(double[] xPoints, double[] yPoints) {
        this.polygon = new PolygonDouble(xPoints, yPoints);
    }

    public void addNodes(ObstacleAvoidanceAStarMap nodes) {
        for (int i = 0; i < polygon.npoints; i++) {
            nodes.addNode(new PoseNode(polygon.xpoints[i], polygon.ypoints[i]));
        }
    }

    public double[] getXPoints() {
        return this.polygon.xpoints;
    }

    public double[] getYPoints() {
        return this.polygon.ypoints;
    }

    public Translation2d[] getCorners() {
        Translation2d[] corners = new Translation2d[polygon.npoints];
        for (int i = 0; i < polygon.npoints; i++)
            corners[i] = new Translation2d(this.polygon.xpoints[i], this.polygon.ypoints[i]);
        return corners;
    }

    /**
     * Creates a polygon that's offset by the distance passed in.
     *
     * Makes a copy of each point of the polygon, offset by a vector at 90deg from the angle of the
     * edges, then connects them together.
     *
     * Has functionality in place to prevent issues with concave shapes having overlapping lines and
     * other weirdness.
     *
     * @param distance Distance to expand the shape outwards by.
     * @return New obstacle, which has the distance passed in added to all sides.
     */
    public Obstacle offset(double distance) {
        // Get a list of all edges with the offsets added onto them
        List<ObstacleEdge> offsetEdges = new ArrayList<>();
        for (int i = 0; i < polygon.npoints; i++) {
            offsetEdges.add(new ObstacleEdge(polygon.xpoints[i], polygon.ypoints[i],
                    polygon.xpoints[(i + 1) % polygon.npoints],
                    polygon.ypoints[(i + 1) % polygon.npoints]).offset(distance));
        }
        List<Double> xPoints = new ArrayList<>();
        List<Double> yPoints = new ArrayList<>();

        // Loop through all edges, checking if there's an intersection between any of them.
        // If an intersection does occur, cut the point off at that intersection,
        // Creating a new point at the intersection, which should be the correct distance away.
        for (int i = 0; i < offsetEdges.size(); i++) {
            ObstacleEdge edge = offsetEdges.get(i);
            // Check against every other edge, including last -> first
            for (int j = i + 1; j <= offsetEdges.size(); j++) {
                // Wrap edges back to beginning so the last edge can be checked against the first
                // one
                ObstacleEdge otherEdge = offsetEdges.get(j % offsetEdges.size());
                Translation2d intersectionPoint = edge.findIntersectionPoint(otherEdge);
                if (intersectionPoint == null && !edge.hasBeenPlotted()) {
                    // If lines don't intersect, and the edge hasn't been plotted out already
                    if (!edge.hasBeenCleaned()) {
                        // If edge was cleaned from a previous loop, don't add the first point
                        xPoints.add(offsetEdges.get(i).point1.getX());
                        yPoints.add(offsetEdges.get(i).point1.getY());
                    }
                    xPoints.add(offsetEdges.get(i).point2.getX());
                    yPoints.add(offsetEdges.get(i).point2.getY());
                    edge.setHasBeenPlotted(true);
                } else {
                    // If lines do intersect
                    if (!edge.hasBeenPlotted()) {
                        // Don't duplicate points
                        if (!edge.hasBeenCleaned()) {
                            // If edge was cleaned from a previous loop, don't add the first point
                            xPoints.add(offsetEdges.get(i).point1.getX());
                            yPoints.add(offsetEdges.get(i).point1.getY());
                        }
                        xPoints.add(intersectionPoint.getX());
                        yPoints.add(intersectionPoint.getY());
                        otherEdge.setHasBeenCleaned(true);
                        edge.setHasBeenPlotted(true);
                    }
                }
            }
        }

        double[] xArr = new double[xPoints.size()];
        double[] yArr = new double[yPoints.size()];

        for (int i = 0; i < xArr.length; i++)
            xArr[i] = xPoints.get(i);
        for (int i = 0; i < yArr.length; i++)
            yArr[i] = yPoints.get(i);


        return new Obstacle(xArr, yArr);
    }

    public String toString() {
        String output = "Polygon(\n";
        for (int i = 0; i < polygon.npoints; i++) {
            output += Double.toString(Math.round(polygon.xpoints[i] * 100) / 100.0) + ", ";
            output += Double.toString(Math.round(polygon.ypoints[i] * 100) / 100.0) + "\n";
        }
        return output + ")";
    }

    private class ObstacleEdge {
        private Translation2d point1;
        private Translation2d point2;
        private boolean hasBeenCleaned = false;
        private boolean hasBeenPlotted = false;

        public ObstacleEdge(double x1, double y1, double x2, double y2) {
            point1 = new Translation2d(x1, y1);
            point2 = new Translation2d(x2, y2);
        }

        public ObstacleEdge(Translation2d point1, Translation2d point2) {
            this.point1 = point1;
            this.point2 = point2;
        }

        public ObstacleEdge offset(double distance) {
            // Calculate angle of edge
            Rotation2d angle = point2.minus(point1).getAngle();
            // Calculate perpendicular angle
            Rotation2d transformAngle = angle.plus(Rotation2d.fromDegrees(90));
            // Create offset vector using distance and angles
            Translation2d offset = new Translation2d(distance, 0).rotateBy(transformAngle);
            // Add offset to points
            return new ObstacleEdge(point1.plus(offset), point2.plus(offset));
        }

        /**
         * This is heavily based on an algorithm in the "Tricks of the Windows Game Programming
         * Gurus" book by Andre LeMothe
         *
         * @param other
         * @return
         */
        public Translation2d findIntersectionPoint(ObstacleEdge other) {
            // Calculate x distance of line 1
            double s1_x = this.point2.getX() - this.point1.getX();
            // Calculate x distance of line 2
            double s2_x = other.point2.getX() - other.point1.getX();
            // Calculate y distance of line 1
            double s1_y = this.point2.getY() - this.point1.getY();
            // Calculate y distance of line 2
            double s2_y = other.point2.getY() - other.point1.getY();

            double s, t;
            // Denominator portion of below equations, split into variable because it's the same
            // between the two
            double d = -s2_x * s1_y + s1_x * s2_y;

            // Magical math that I need to look into how it works more
            s = (-s1_y * (this.point1.getX() - other.point1.getX())
                    + s1_x * (this.point1.getY() - other.point1.getY())) / d;
            t = (s2_x * (this.point1.getY() - other.point1.getY())
                    - s2_y * (this.point1.getX() - other.point1.getX())) / d;

            double i_x, i_y;
            if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
                // Intersection found
                i_x = this.point1.getX() + (t * s1_x);
                i_y = this.point1.getY() + (t * s1_y);
                return new Translation2d(i_x, i_y);
            }
            return null;

        }

        public String toString() {
            return point1.toString() + "," + point2.toString();
        }

        public boolean hasBeenCleaned() {
            return hasBeenCleaned;
        }

        public void setHasBeenCleaned(boolean hasBeenCleaned) {
            this.hasBeenCleaned = hasBeenCleaned;
        }

        public boolean hasBeenPlotted() {
            return hasBeenPlotted;
        }

        public void setHasBeenPlotted(boolean hasBeenPlotted) {
            this.hasBeenPlotted = hasBeenPlotted;
        }
    }

    /*
     * 
     * Licensed to the Apache Software Foundation (ASF) under one or more contributor license
     * agreements. See the NOTICE file distributed with this work for additional information
     * regarding copyright ownership. The ASF licenses this file to You under the Apache License,
     * Version 2.0 (the "License"); you may not use this file except in compliance with the License.
     * You may obtain a copy of the License at
     * 
     * http://www.apache.org/licenses/LICENSE-2.0
     * 
     * Unless required by applicable law or agreed to in writing, software distributed under the
     * License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
     * either express or implied. See the License for the specific language governing permissions
     * and limitations under the License.
     * 
     */



    /**
     * This class is a Polygon with double coordinates.
     *
     * @version $Id: Polygon2D.java 594018 2007-11-12 04:17:41Z cam $
     */
    public class PolygonDouble implements Shape, Cloneable, Serializable {

        /**
         * The total number of points. The value of <code>npoints</code> represents the number of
         * valid points in this <code>Polygon</code>.
         */
        public int npoints;

        /**
         * The array of <i>x</i> coordinates. The value of {@link #npoints npoints} is equal to the
         * number of points in this <code>Polygon2D</code>.
         */
        public double[] xpoints;

        /**
         * The array of <i>x</i> coordinates. The value of {@link #npoints npoints} is equal to the
         * number of points in this <code>Polygon2D</code>.
         */
        public double[] ypoints;

        /**
         * Bounds of the Polygon2D.
         *
         * @see #getBounds()
         */
        protected Rectangle2D bounds;

        private GeneralPath path;
        private GeneralPath closedPath;

        /**
         * Creates an empty Polygon2D.
         */
        public PolygonDouble() {
            xpoints = new double[4];
            ypoints = new double[4];
        }

        /**
         * Constructs and initializes a <code>Polygon2D</code> from the specified Rectangle2D.
         *
         * @param rec the Rectangle2D
         * @throws NullPointerException rec is <code>null</code>.
         */
        public PolygonDouble(Rectangle2D rec) {
            if (rec == null) {
                throw new IndexOutOfBoundsException("null Rectangle");
            }
            npoints = 4;
            xpoints = new double[4];
            ypoints = new double[4];
            xpoints[0] = rec.getMinX();
            ypoints[0] = rec.getMinY();
            xpoints[1] = rec.getMaxX();
            ypoints[1] = rec.getMinY();
            xpoints[2] = rec.getMaxX();
            ypoints[2] = rec.getMaxY();
            xpoints[3] = rec.getMinX();
            ypoints[3] = rec.getMaxY();
            calculatePath();
        }

        /**
         * Constructs and initializes a <code>Polygon2D</code> from the specified Polygon.
         *
         * @param pol the Polygon
         * @throws NullPointerException pol is <code>null</code>.
         */
        public PolygonDouble(Polygon pol) {
            if (pol == null) {
                throw new IndexOutOfBoundsException("null Polygon");
            }
            this.npoints = pol.npoints;
            this.xpoints = new double[pol.npoints];
            this.ypoints = new double[pol.npoints];
            for (int i = 0; i < pol.npoints; i++) {
                xpoints[i] = pol.xpoints[i];
                ypoints[i] = pol.ypoints[i];
            }
            calculatePath();
        }

        /**
         * Constructs and initializes a <code>Polygon2D</code> from the specified parameters.
         *
         * @param xpoints an array of <i>x</i> coordinates
         * @param ypoints an array of <i>y</i> coordinates
         * @param npoints the total number of points in the <code>Polygon2D</code>
         * @throws NegativeArraySizeException if the value of <code>npoints</code> is negative.
         * @throws IndexOutOfBoundsException if <code>npoints</code> is greater than the length of
         *         <code>xpoints</code> or the length of <code>ypoints</code>.
         * @throws NullPointerException if <code>xpoints</code> or <code>ypoints</code> is
         *         <code>null</code>.
         */
        public PolygonDouble(double[] xpoints, double[] ypoints) {
            if (xpoints.length != ypoints.length) {
                throw new IndexOutOfBoundsException(
                        "npoints > xpoints.length || npoints > ypoints.length");
            }
            this.npoints = xpoints.length;
            this.xpoints = new double[npoints];
            this.ypoints = new double[npoints];
            System.arraycopy(xpoints, 0, this.xpoints, 0, npoints);
            System.arraycopy(ypoints, 0, this.ypoints, 0, npoints);
            calculatePath();
        }

        /**
         * Constructs and initializes a <code>Polygon2D</code> from the specified parameters.
         *
         * @param xpoints an array of <i>x</i> coordinates
         * @param ypoints an array of <i>y</i> coordinates
         * @param npoints the total number of points in the <code>Polygon2D</code>
         * @throws NegativeArraySizeException if the value of <code>npoints</code> is negative.
         * @throws IndexOutOfBoundsException if <code>npoints</code> is greater than the length of
         *         <code>xpoints</code> or the length of <code>ypoints</code>.
         * @throws NullPointerException if <code>xpoints</code> or <code>ypoints</code> is
         *         <code>null</code>.
         */
        public PolygonDouble(int[] xpoints, int[] ypoints, int npoints) {
            if (npoints > xpoints.length || npoints > ypoints.length) {
                throw new IndexOutOfBoundsException(
                        "npoints > xpoints.length || npoints > ypoints.length");
            }
            this.npoints = npoints;
            this.xpoints = new double[npoints];
            this.ypoints = new double[npoints];
            for (int i = 0; i < npoints; i++) {
                this.xpoints[i] = xpoints[i];
                this.ypoints[i] = ypoints[i];
            }
            calculatePath();
        }

        /**
         * Resets this <code>Polygon</code> object to an empty polygon.
         */
        public void reset() {
            npoints = 0;
            bounds = null;
            path = new GeneralPath();
            closedPath = null;
        }

        public Object clone() {
            PolygonDouble pol = new PolygonDouble();
            for (int i = 0; i < npoints; i++) {
                pol.addPoint(xpoints[i], ypoints[i]);
            }
            return pol;
        }

        private void calculatePath() {
            path = new GeneralPath();
            path.moveTo(xpoints[0], ypoints[0]);
            for (int i = 1; i < npoints; i++) {
                path.lineTo(xpoints[i], ypoints[i]);
            }
            bounds = path.getBounds2D();
            closedPath = null;
        }

        private void updatePath(double x, double y) {
            closedPath = null;
            if (path == null) {
                path = new GeneralPath(GeneralPath.WIND_EVEN_ODD);
                path.moveTo(x, y);
                bounds = new Rectangle2D.Double(x, y, 0, 0);
            } else {
                path.lineTo(x, y);
                double _xmax = bounds.getMaxX();
                double _ymax = bounds.getMaxY();
                double _xmin = bounds.getMinX();
                double _ymin = bounds.getMinY();
                if (x < _xmin)
                    _xmin = x;
                else if (x > _xmax)
                    _xmax = x;
                if (y < _ymin)
                    _ymin = y;
                else if (y > _ymax)
                    _ymax = y;
                bounds = new Rectangle2D.Double(_xmin, _ymin, _xmax - _xmin, _ymax - _ymin);
            }
        }

        /*
         * get the associated {@link Polyline2D}.
         */
        public Polyline2D getPolyline2D() {

            Polyline2D pol = new Polyline2D(xpoints, ypoints, npoints);

            pol.addPoint(xpoints[0], ypoints[0]);

            return pol;
        }

        public Polygon getPolygon() {
            int[] _xpoints = new int[npoints];
            int[] _ypoints = new int[npoints];
            for (int i = 0; i < npoints; i++) {
                _xpoints[i] = (int) xpoints[i]; // todo maybe rounding is better ?
                _ypoints[i] = (int) ypoints[i];
            }

            return new Polygon(_xpoints, _ypoints, npoints);
        }

        public void addPoint(Point2D p) {
            addPoint(p.getX(), p.getY());
        }

        /**
         * Appends the specified coordinates to this <code>Polygon2D</code>.
         *
         * @param x the specified x coordinate
         * @param y the specified y coordinate
         */
        public void addPoint(double x, double y) {
            if (npoints == xpoints.length) {
                double[] tmp;

                tmp = new double[npoints * 2];
                System.arraycopy(xpoints, 0, tmp, 0, npoints);
                xpoints = tmp;

                tmp = new double[npoints * 2];
                System.arraycopy(ypoints, 0, tmp, 0, npoints);
                ypoints = tmp;
            }
            xpoints[npoints] = x;
            ypoints[npoints] = y;
            npoints++;
            updatePath(x, y);
        }

        /**
         * Determines whether the specified {@link Point} is inside this <code>Polygon</code>.
         *
         * @param p the specified <code>Point</code> to be tested
         * @return <code>true</code> if the <code>Polygon</code> contains the <code>Point</code>;
         *         <code>false</code> otherwise.
         * @see #contains(double, double)
         */
        public boolean contains(Point p) {
            return contains(p.x, p.y);
        }

        /**
         * Determines whether the specified coordinates are inside this <code>Polygon</code>.
         * <p>
         *
         * @param x the specified x coordinate to be tested
         * @param y the specified y coordinate to be tested
         * @return <code>true</code> if this <code>Polygon</code> contains the specified
         *         coordinates, (<i>x</i>,&nbsp;<i>y</i>); <code>false</code> otherwise.
         */
        public boolean contains(int x, int y) {
            return contains(x, (double) y);
        }

        /**
         * Returns the high precision bounding box of the {@link Shape}.
         *
         * @return a {@link Rectangle2D} that precisely bounds the <code>Shape</code>.
         */
        public Rectangle2D getBounds2D() {
            return bounds;
        }

        public Rectangle getBounds() {
            if (bounds == null)
                return null;
            else
                return bounds.getBounds();
        }

        /**
         * Determines if the specified coordinates are inside this <code>Polygon</code>. For the
         * definition of <i>insideness</i>, see the class comments of {@link Shape}.
         *
         * @param x the specified x coordinate
         * @param y the specified y coordinate
         * @return <code>true</code> if the <code>Polygon</code> contains the specified coordinates;
         *         <code>false</code> otherwise.
         */
        public boolean contains(double x, double y) {
            if (npoints <= 2 || !bounds.contains(x, y)) {
                return false;
            }
            updateComputingPath();

            return closedPath.contains(x, y);
        }

        private void updateComputingPath() {
            if (npoints >= 1) {
                if (closedPath == null) {
                    closedPath = (GeneralPath) path.clone();
                    closedPath.closePath();
                }
            }
        }

        /**
         * Tests if a specified {@link Point2D} is inside the boundary of this <code>Polygon</code>.
         *
         * @param p a specified <code>Point2D</code>
         * @return <code>true</code> if this <code>Polygon</code> contains the specified
         *         <code>Point2D</code>; <code>false</code> otherwise.
         * @see #contains(double, double)
         */
        public boolean contains(Point2D p) {
            return contains(p.getX(), p.getY());
        }

        /**
         * Tests if the interior of this <code>Polygon</code> intersects the interior of a specified
         * set of rectangular coordinates.
         *
         * @param x the x coordinate of the specified rectangular shape's top-left corner
         * @param y the y coordinate of the specified rectangular shape's top-left corner
         * @param w the width of the specified rectangular shape
         * @param h the height of the specified rectangular shape
         * @return <code>true</code> if the interior of this <code>Polygon</code> and the interior
         *         of the specified set of rectangular coordinates intersect each other;
         *         <code>false</code> otherwise.
         */
        public boolean intersects(double x, double y, double w, double h) {
            if (npoints <= 0 || !bounds.intersects(x, y, w, h)) {
                return false;
            }
            updateComputingPath();
            return closedPath.intersects(x, y, w, h);
        }

        /**
         * Tests if the interior of this <code>Polygon</code> intersects the interior of a specified
         * <code>Rectangle2D</code>.
         *
         * @param r a specified <code>Rectangle2D</code>
         * @return <code>true</code> if this <code>Polygon</code> and the interior of the specified
         *         <code>Rectangle2D</code> intersect each other; <code>false</code> otherwise.
         */
        public boolean intersects(Rectangle2D r) {
            return intersects(r.getX(), r.getY(), r.getWidth(), r.getHeight());
        }

        /**
         * Tests if the interior of this <code>Polygon</code> entirely contains the specified set of
         * rectangular coordinates.
         *
         * @param x the x coordinate of the top-left corner of the specified set of rectangular
         *        coordinates
         * @param y the y coordinate of the top-left corner of the specified set of rectangular
         *        coordinates
         * @param w the width of the set of rectangular coordinates
         * @param h the height of the set of rectangular coordinates
         * @return <code>true</code> if this <code>Polygon</code> entirely contains the specified
         *         set of rectangular coordinates; <code>false</code> otherwise.
         */
        public boolean contains(double x, double y, double w, double h) {
            if (npoints <= 0 || !bounds.intersects(x, y, w, h)) {
                return false;
            }

            updateComputingPath();
            return closedPath.contains(x, y, w, h);
        }

        /**
         * Tests if the interior of this <code>Polygon</code> entirely contains the specified
         * <code>Rectangle2D</code>.
         *
         * @param r the specified <code>Rectangle2D</code>
         * @return <code>true</code> if this <code>Polygon</code> entirely contains the specified
         *         <code>Rectangle2D</code>; <code>false</code> otherwise.
         * @see #contains(double, double, double, double)
         */
        public boolean contains(Rectangle2D r) {
            return contains(r.getX(), r.getY(), r.getWidth(), r.getHeight());
        }

        /**
         * Returns an iterator object that iterates along the boundary of this <code>Polygon</code>
         * and provides access to the geometry of the outline of this <code>Polygon</code>. An
         * optional {@link AffineTransform} can be specified so that the coordinates returned in the
         * iteration are transformed accordingly.
         *
         * @param at an optional <code>AffineTransform</code> to be applied to the coordinates as
         *        they are returned in the iteration, or <code>null</code> if untransformed
         *        coordinates are desired
         * @return a {@link PathIterator} object that provides access to the geometry of this
         *         <code>Polygon</code>.
         */
        public PathIterator getPathIterator(AffineTransform at) {
            updateComputingPath();
            if (closedPath == null)
                return null;
            else
                return closedPath.getPathIterator(at);
        }

        /**
         * Returns an iterator object that iterates along the boundary of the <code>Polygon2D</code>
         * and provides access to the geometry of the outline of the <code>Shape</code>. Only
         * SEG_MOVETO, SEG_LINETO, and SEG_CLOSE point types are returned by the iterator. Since
         * polygons are already flat, the <code>flatness</code> parameter is ignored.
         *
         * @param at an optional <code>AffineTransform</code> to be applied to the coordinates as
         *        they are returned in the iteration, or <code>null</code> if untransformed
         *        coordinates are desired
         * @param flatness the maximum amount that the control points for a given curve can vary
         *        from colinear before a subdivided curve is replaced by a straight line connecting
         *        the endpoints. Since polygons are already flat the <code>flatness</code> parameter
         *        is ignored.
         * @return a <code>PathIterator</code> object that provides access to the <code>Shape</code>
         *         object's geometry.
         */
        public PathIterator getPathIterator(AffineTransform at, double flatness) {
            return getPathIterator(at);
        }
    }

    /*
     * 
     * Licensed to the Apache Software Foundation (ASF) under one or more contributor license
     * agreements. See the NOTICE file distributed with this work for additional information
     * regarding copyright ownership. The ASF licenses this file to You under the Apache License,
     * Version 2.0 (the "License"); you may not use this file except in compliance with the License.
     * You may obtain a copy of the License at
     * 
     * http://www.apache.org/licenses/LICENSE-2.0
     * 
     * Unless required by applicable law or agreed to in writing, software distributed under the
     * License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
     * either express or implied. See the License for the specific language governing permissions
     * and limitations under the License.
     * 
     */


    /**
     * This class has the same behavior than {@link Polygon2D}, except that the figure is not
     * closed.
     *
     * @version $Id: Polyline2D.java 594018 2007-11-12 04:17:41Z cam $
     */
    class Polyline2D implements Shape, Cloneable, Serializable {

        private static final double ASSUME_ZERO = 0.001f;

        /**
         * The total number of points. The value of <code>npoints</code> represents the number of
         * points in this <code>Polyline2D</code>.
         */
        public int npoints;

        /**
         * The array of <i>x</i> coordinates. The value of {@link #npoints npoints} is equal to the
         * number of points in this <code>Polyline2D</code>.
         */
        public double[] xpoints;

        /**
         * The array of <i>x</i> coordinates. The value of {@link #npoints npoints} is equal to the
         * number of points in this <code>Polyline2D</code>.
         */
        public double[] ypoints;

        /**
         * Bounds of the Polyline2D.
         *
         * @see #getBounds()
         */
        protected Rectangle2D bounds;

        private GeneralPath path;
        private GeneralPath closedPath;

        /**
         * Creates an empty Polyline2D.
         */
        public Polyline2D() {
            xpoints = new double[4];
            ypoints = new double[4];
        }

        /**
         * Constructs and initializes a <code>Polyline2D</code> from the specified parameters.
         *
         * @param xpoints an array of <i>x</i> coordinates
         * @param ypoints an array of <i>y</i> coordinates
         * @param npoints the total number of points in the <code>Polyline2D</code>
         * @throws NegativeArraySizeException if the value of <code>npoints</code> is negative.
         * @throws IndexOutOfBoundsException if <code>npoints</code> is greater than the length of
         *         <code>xpoints</code> or the length of <code>ypoints</code>.
         * @throws NullPointerException if <code>xpoints</code> or <code>ypoints</code> is
         *         <code>null</code>.
         */
        public Polyline2D(double[] xpoints, double[] ypoints, int npoints) {
            if (npoints > xpoints.length || npoints > ypoints.length) {
                throw new IndexOutOfBoundsException(
                        "npoints > xpoints.length || npoints > ypoints.length");
            }
            this.npoints = npoints;
            this.xpoints = new double[npoints + 1]; // make space for one more to close the polyline
            this.ypoints = new double[npoints + 1]; // make space for one more to close the polyline
            System.arraycopy(xpoints, 0, this.xpoints, 0, npoints);
            System.arraycopy(ypoints, 0, this.ypoints, 0, npoints);
            calculatePath();
        }

        /**
         * Constructs and initializes a <code>Polyline2D</code> from the specified parameters.
         *
         * @param xpoints an array of <i>x</i> coordinates
         * @param ypoints an array of <i>y</i> coordinates
         * @param npoints the total number of points in the <code>Polyline2D</code>
         * @throws NegativeArraySizeException if the value of <code>npoints</code> is negative.
         * @throws IndexOutOfBoundsException if <code>npoints</code> is greater than the length of
         *         <code>xpoints</code> or the length of <code>ypoints</code>.
         * @throws NullPointerException if <code>xpoints</code> or <code>ypoints</code> is
         *         <code>null</code>.
         */
        public Polyline2D(int[] xpoints, int[] ypoints, int npoints) {
            if (npoints > xpoints.length || npoints > ypoints.length) {
                throw new IndexOutOfBoundsException(
                        "npoints > xpoints.length || npoints > ypoints.length");
            }
            this.npoints = npoints;
            this.xpoints = new double[npoints];
            this.ypoints = new double[npoints];
            for (int i = 0; i < npoints; i++) {
                this.xpoints[i] = xpoints[i];
                this.ypoints[i] = ypoints[i];
            }
            calculatePath();
        }

        public Polyline2D(Line2D line) {
            npoints = 2;
            xpoints = new double[2];
            ypoints = new double[2];
            xpoints[0] = line.getX1();
            xpoints[1] = line.getX2();
            ypoints[0] = line.getY1();
            ypoints[1] = line.getY2();
            calculatePath();
        }

        /**
         * Resets this <code>Polyline2D</code> object to an empty polygon. The coordinate arrays and
         * the data in them are left untouched but the number of points is reset to zero to mark the
         * old vertex data as invalid and to start accumulating new vertex data at the beginning.
         * All internally-cached data relating to the old vertices are discarded. Note that since
         * the coordinate arrays from before the reset are reused, creating a new empty
         * <code>Polyline2D</code> might be more memory efficient than resetting the current one if
         * the number of vertices in the new polyline data is significantly smaller than the number
         * of vertices in the data from before the reset.
         */
        public void reset() {
            npoints = 0;
            bounds = null;
            path = new GeneralPath();
            closedPath = null;
        }

        public Object clone() {
            Polyline2D pol = new Polyline2D();
            for (int i = 0; i < npoints; i++) {
                pol.addPoint(xpoints[i], ypoints[i]);
            }
            return pol;
        }

        private void calculatePath() {
            path = new GeneralPath();
            path.moveTo(xpoints[0], ypoints[0]);
            for (int i = 1; i < npoints; i++) {
                path.lineTo(xpoints[i], ypoints[i]);
            }
            bounds = path.getBounds2D();
            closedPath = null;
        }

        private void updatePath(double x, double y) {
            closedPath = null;
            if (path == null) {
                path = new GeneralPath(GeneralPath.WIND_EVEN_ODD);
                path.moveTo(x, y);
                bounds = new Rectangle2D.Double(x, y, 0, 0);
            } else {
                path.lineTo(x, y);
                double _xmax = bounds.getMaxX();
                double _ymax = bounds.getMaxY();
                double _xmin = bounds.getMinX();
                double _ymin = bounds.getMinY();
                if (x < _xmin)
                    _xmin = x;
                else if (x > _xmax)
                    _xmax = x;
                if (y < _ymin)
                    _ymin = y;
                else if (y > _ymax)
                    _ymax = y;
                bounds = new Rectangle2D.Double(_xmin, _ymin, _xmax - _xmin, _ymax - _ymin);
            }
        }

        public void addPoint(Point2D p) {
            addPoint(p.getX(), p.getY());
        }

        /**
         * Appends the specified coordinates to this <code>Polyline2D</code>.
         * <p>
         * If an operation that calculates the bounding box of this <code>Polyline2D</code> has
         * already been performed, such as <code>getBounds</code> or <code>contains</code>, then
         * this method updates the bounding box.
         *
         * @param x the specified x coordinate
         * @param y the specified y coordinate
         * @see java.awt.Polygon#getBounds
         * @see java.awt.Polygon#contains(double, double)
         */
        public void addPoint(double x, double y) {
            if (npoints == xpoints.length) {
                double[] tmp;

                tmp = new double[npoints * 2];
                System.arraycopy(xpoints, 0, tmp, 0, npoints);
                xpoints = tmp;

                tmp = new double[npoints * 2];
                System.arraycopy(ypoints, 0, tmp, 0, npoints);
                ypoints = tmp;
            }
            xpoints[npoints] = x;
            ypoints[npoints] = y;
            npoints++;
            updatePath(x, y);
        }

        /**
         * Gets the bounding box of this <code>Polyline2D</code>. The bounding box is the smallest
         * {@link Rectangle} whose sides are parallel to the x and y axes of the coordinate space,
         * and can completely contain the <code>Polyline2D</code>.
         *
         * @return a <code>Rectangle</code> that defines the bounds of this <code>Polyline2D</code>.
         */
        public Rectangle getBounds() {
            if (bounds == null)
                return null;
            else
                return bounds.getBounds();
        }

        private void updateComputingPath() {
            if (npoints >= 1) {
                if (closedPath == null) {
                    closedPath = (GeneralPath) path.clone();
                    closedPath.closePath();
                }
            }
        }

        /**
         * Determines whether the specified {@link Point} is inside this <code>Polyline2D</code>.
         * This method is required to implement the Shape interface, but in the case of Line2D
         * objects it always returns false since a line contains no area.
         */
        public boolean contains(Point p) {
            return false;
        }

        /**
         * Determines if the specified coordinates are inside this <code>Polyline2D</code>. This
         * method is required to implement the Shape interface, but in the case of Line2D objects it
         * always returns false since a line contains no area.
         */
        public boolean contains(double x, double y) {
            return false;
        }

        /**
         * Determines whether the specified coordinates are inside this <code>Polyline2D</code>.
         * This method is required to implement the Shape interface, but in the case of Line2D
         * objects it always returns false since a line contains no area.
         */
        public boolean contains(int x, int y) {
            return false;
        }

        /**
         * Returns the high precision bounding box of the {@link Shape}.
         *
         * @return a {@link Rectangle2D} that precisely bounds the <code>Shape</code>.
         */
        public Rectangle2D getBounds2D() {
            return bounds;
        }

        /**
         * Tests if a specified {@link Point2D} is inside the boundary of this
         * <code>Polyline2D</code>. This method is required to implement the Shape interface, but in
         * the case of Line2D objects it always returns false since a line contains no area.
         */
        public boolean contains(Point2D p) {
            return false;
        }

        /**
         * Tests if the interior of this <code>Polygon</code> intersects the interior of a specified
         * set of rectangular coordinates.
         *
         * @param x the x coordinate of the specified rectangular shape's top-left corner
         * @param y the y coordinate of the specified rectangular shape's top-left corner
         * @param w the width of the specified rectangular shape
         * @param h the height of the specified rectangular shape
         * @return <code>true</code> if the interior of this <code>Polygon</code> and the interior
         *         of the specified set of rectangular coordinates intersect each other;
         *         <code>false</code> otherwise.
         */
        public boolean intersects(double x, double y, double w, double h) {
            if (npoints <= 0 || !bounds.intersects(x, y, w, h)) {
                return false;
            }
            updateComputingPath();
            return closedPath.intersects(x, y, w, h);
        }

        /**
         * Tests if the interior of this <code>Polygon</code> intersects the interior of a specified
         * <code>Rectangle2D</code>.
         *
         * @param r a specified <code>Rectangle2D</code>
         * @return <code>true</code> if this <code>Polygon</code> and the interior of the specified
         *         <code>Rectangle2D</code> intersect each other; <code>false</code> otherwise.
         */
        public boolean intersects(Rectangle2D r) {
            return intersects(r.getX(), r.getY(), r.getWidth(), r.getHeight());
        }

        /**
         * Tests if the interior of this <code>Polyline2D</code> entirely contains the specified set
         * of rectangular coordinates. This method is required to implement the Shape interface, but
         * in the case of Line2D objects it always returns false since a line contains no area.
         */
        public boolean contains(double x, double y, double w, double h) {
            return false;
        }

        /**
         * Tests if the interior of this <code>Polyline2D</code> entirely contains the specified
         * <code>Rectangle2D</code>. This method is required to implement the Shape interface, but
         * in the case of Line2D objects it always returns false since a line contains no area.
         */
        public boolean contains(Rectangle2D r) {
            return false;
        }

        /**
         * Returns an iterator object that iterates along the boundary of this <code>Polygon</code>
         * and provides access to the geometry of the outline of this <code>Polygon</code>. An
         * optional {@link AffineTransform} can be specified so that the coordinates returned in the
         * iteration are transformed accordingly.
         *
         * @param at an optional <code>AffineTransform</code> to be applied to the coordinates as
         *        they are returned in the iteration, or <code>null</code> if untransformed
         *        coordinates are desired
         * @return a {@link PathIterator} object that provides access to the geometry of this
         *         <code>Polygon</code>.
         */
        public PathIterator getPathIterator(AffineTransform at) {
            if (path == null)
                return null;
            else
                return path.getPathIterator(at);
        }

        /*
         * get the associated {@link Polygon2D}. This method take care that may be the last point
         * can be equal to the first. In that case it must not be included in the Polygon, as
         * polygons declare their first point only once.
         */
        public PolygonDouble getPolygon2D() {
            PolygonDouble pol = new PolygonDouble();
            for (int i = 0; i < npoints - 1; i++) {
                pol.addPoint(xpoints[i], ypoints[i]);
            }
            Point2D.Double p0 = new Point2D.Double(xpoints[0], ypoints[0]);
            Point2D.Double p1 = new Point2D.Double(xpoints[npoints - 1], ypoints[npoints - 1]);

            if (p0.distance(p1) > ASSUME_ZERO)
                pol.addPoint(xpoints[npoints - 1], ypoints[npoints - 1]);

            return pol;
        }

        /**
         * Returns an iterator object that iterates along the boundary of the <code>Shape</code> and
         * provides access to the geometry of the outline of the <code>Shape</code>. Only SEG_MOVETO
         * and SEG_LINETO, point types are returned by the iterator. Since polylines are already
         * flat, the <code>flatness</code> parameter is ignored.
         *
         * @param at an optional <code>AffineTransform</code> to be applied to the coordinates as
         *        they are returned in the iteration, or <code>null</code> if untransformed
         *        coordinates are desired
         * @param flatness the maximum amount that the control points for a given curve can vary
         *        from colinear before a subdivided curve is replaced by a straight line connecting
         *        the endpoints. Since polygons are already flat the <code>flatness</code> parameter
         *        is ignored.
         * @return a <code>PathIterator</code> object that provides access to the <code>Shape</code>
         *         object's geometry.
         */
        public PathIterator getPathIterator(AffineTransform at, double flatness) {
            return path.getPathIterator(at);
        }
    }

}
