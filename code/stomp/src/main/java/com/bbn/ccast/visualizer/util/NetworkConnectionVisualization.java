//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import gov.nasa.worldwind.WorldWind;
import gov.nasa.worldwind.geom.Extent;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Sphere;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.pick.PickSupport;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.OrderedRenderable;

import java.awt.*;
import java.util.HashSet;
import java.util.Set;

/**
 * Custom renderable for drawing a device's network links
 * <p>
 * Derived on the cube example in the WorldWind examples package by pabercrombie.
 * The cube example is Copyright NASA, under the licensing of WorldWind
 */
public class NetworkConnectionVisualization implements OrderedRenderable {
    /**
     * Geographic position of the device.
     */
    private Position position;
    /**
     * Geographic positions of neighbors
     */
    private Iterable<Position> neighbors;

    /**
     * Support object to help with pick resolution.
     */
    private PickSupport pickSupport = new PickSupport();

    // Determined each frame
    private long frameTimestamp = -1L;
    /**
     * Cartesian position of the device, computed from {@link #position}.
     */
    private Vec4 placePoint;
    /**
     * Cartesian positions of neighbors, computed from {@link #neighbors}
     */
    private Set<Vec4> nbrPoints = new HashSet<>();
    /**
     * Distance from the eye point to the device.
     */
    private double eyeDistance;
    private Extent extent;

    /**
     * Create a visualization of the network connections of one device.
     * Note that when there are bidirectional connections, these can be doubly drawn
     * e.g., from A to B and B to A
     *
     * @param pos  Position of the device whose connections are being visualized
     * @param nbrs Position of the devices it communicates with
     */
    public NetworkConnectionVisualization(final Position pos, final Iterable<Position> nbrs) {
        this.position = pos;
        this.neighbors = nbrs;
    }

    /**
     * Update the position of a device.
     *
     * @param pos new position
     */
    public void setPosition(final Position pos) {
        this.position = pos;
    }

    /**
     * Update the position of a device's neighbors.
     *
     * @param nbrs collection of new positions
     */
    public void setNeighbors(final Iterable<Position> nbrs) {
        this.neighbors = nbrs;
    }

    @Override
    public void render(final DrawContext dc) {
        // Render is called three times:
        // 1) During picking. The network is drawn in a single color.
        // 2) As a normal renderable. The network is added to the ordered renderable queue.
        // 3) As an OrderedRenderable. The network is drawn.

        if (this.extent != null) {
            // Ignore if not visible
            if (!this.intersectsFrustum(dc)) {
                return;
            }
            // If the shape is less that a pixel in size, don't render it.
            if (dc.isSmall(this.extent, 1)) {
                return;
            }
        }

        if (dc.isOrderedRenderingMode()) {
            this.drawOrderedRenderable(dc, this.pickSupport);
        } else {
            this.makeOrderedRenderable(dc);
        }
    }

    /**
     * Determines whether the network intersects the view frustum.
     *
     * @param dc the current draw context.
     * @return true if this network intersects the frustum, otherwise false.
     */
    protected boolean intersectsFrustum(final DrawContext dc) {
        if (this.extent == null) {
            return true; // don't know the visibility, shape hasn't been computed yet
        }

        if (dc.isPickingMode()) {
            return dc.getPickFrustums().intersectsAny(this.extent);
        }

        return dc.getView().getFrustumInModelCoordinates().intersects(this.extent);
    }

    @Override
    public double getDistanceFromEye() {
        return this.eyeDistance;
    }

    @Override
    public void pick(final DrawContext dc, final Point pickPoint) {
        // Use same code for rendering and picking.
        this.render(dc);
    }

    /**
     * Setup drawing state in preparation for drawing the network. State changed by this method must be restored in
     * endDrawing.
     *
     * @param dc Active draw context.
     */
    protected void beginDrawing(final DrawContext dc) {
        GL2 gl = dc.getGL().getGL2(); // GL initialization checks for GL2 compatibility.

        int attrMask = GL2.GL_CURRENT_BIT | GL.GL_COLOR_BUFFER_BIT;

        gl.glPushAttrib(attrMask);

//        if (!dc.isPickingMode()) {
//            dc.beginStandardLighting();
//            gl.glEnable(GL.GL_BLEND);
//            OGLUtil.applyBlending(gl, false);
//        }
    }

    /**
     * Restore drawing state changed in beginDrawing to the default.
     *
     * @param dc Active draw context.
     */
    protected void endDrawing(final DrawContext dc) {
        GL2 gl = dc.getGL().getGL2(); // GL initialization checks for GL2 compatibility.

//        if (!dc.isPickingMode()) {
//            dc.endStandardLighting();
//        }

        gl.glPopAttrib();
    }

    /**
     * Compute per-frame attributes, and add the ordered renderable to the ordered renderable list.
     *
     * @param dc Current draw context.
     */
    protected void makeOrderedRenderable(final DrawContext dc) {
        // This method is called twice each frame: once during picking and once during rendering. We only need to
        // compute the placePoint and eye distance once per frame, so check the frame timestamp to see if this is a
        // new frame.
        if (dc.getFrameTimeStamp() != this.frameTimestamp) {
            // Convert the device's geographic position to a position in Cartesian coordinates.
            this.placePoint = dc.computePointFromPosition(this.position, WorldWind.ABSOLUTE);
            dc.getVerticalExaggeration();

            // Compute the distance from the eye to the device's position.
            this.eyeDistance = dc.getView().getEyePoint().distanceTo3(this.placePoint);

            // Compute a sphere that encloses the network. We'll use this sphere for intersection calculations to determine
            // if the network is actually visible.
            double maxDist = 1; // minimum must be better than zero
            nbrPoints.clear();
            for (Position nbr : neighbors) {
                Vec4 nbrPoint = dc.computePointFromPosition(nbr, WorldWind.ABSOLUTE);
                nbrPoints.add(nbrPoint);
                double nbrDist = nbrPoint.distanceTo3(placePoint);
                maxDist = Math.max(maxDist, nbrDist);
            }
            this.extent = new Sphere(this.placePoint, maxDist);

            this.frameTimestamp = dc.getFrameTimeStamp();
        }

        // Add the device to the ordered renderable list. The SceneController sorts the ordered renderables by eye
        // distance, and then renders them back to front. render will be called again in ordered rendering mode, and at
        // that point we will actually draw the network.
        dc.addOrderedRenderable(this);
    }

    /**
     * Set up drawing state, and draw the network. This method is called when the
     * network is rendered in ordered rendering mode.
     *
     * @param dc             Current draw context.
     * @param pickCandidates Record of which objects may be considered for individual selection
     */
    protected void drawOrderedRenderable(final DrawContext dc, final PickSupport pickCandidates) {
        this.beginDrawing(dc);
        try {
            GL2 gl = dc.getGL().getGL2(); // GL initialization checks for GL2 compatibility.
            if (dc.isPickingMode()) {
                Color pickColor = dc.getUniquePickColor();
                pickCandidates.addPickableObject(pickColor.getRGB(), this, this.position);
                gl.glColor3ub((byte) pickColor.getRed(), (byte) pickColor.getGreen(), (byte) pickColor.getBlue());
            } else {
                // magenta links
                gl.glColor3f(1.0f, 0.0f, 1.0f);
            }

            this.drawEdges(dc);
        } finally {
            this.endDrawing(dc);
        }
    }

    /**
     * Actually draw the set of edges in the current (configured) drawing context.
     *
     * @param dc Current draw context.
     */
    protected void drawEdges(final DrawContext dc) {
        // Note: draw the network in OpenGL immediate mode for simplicity. Real applications may want
        // to use vertex arrays or vertex buffer objects to achieve better performance.
        GL2 gl = dc.getGL().getGL2(); // GL initialization checks for GL2 compatibility.
        gl.glBegin(GL.GL_LINES);
        try {
            for (Vec4 nbr : nbrPoints) {
                gl.glVertex3d(placePoint.x, placePoint.y, placePoint.z);
                gl.glVertex3d(nbr.x, nbr.y, nbr.z);
            }
        } finally {
            gl.glEnd();
        }
    }
}
