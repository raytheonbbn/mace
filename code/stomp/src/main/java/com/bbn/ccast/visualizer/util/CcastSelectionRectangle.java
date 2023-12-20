//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.fixedfunc.GLPointerFunc;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.OrderedRenderable;
import gov.nasa.worldwind.util.Logging;
import gov.nasa.worldwind.util.OGLStackHandler;
import gov.nasa.worldwind.util.OGLUtil;

import java.awt.*;

public class CcastSelectionRectangle implements OrderedRenderable {

	protected static final Color DEFAULT_INTERIOR_COLOR = new Color(255, 255, 255, 64);
	protected static final Color DEFAULT_BORDER_COLOR = Color.WHITE;

	protected Rectangle rect;
	protected Point startPoint;
	protected Point endPoint;
	protected Color interiorColor;
	protected Color borderColor;
	protected OGLStackHandler BEogsh = new OGLStackHandler();

	public CcastSelectionRectangle() {
		this.rect = new Rectangle();
		this.startPoint = new Point();
		this.endPoint = new Point();
	}

	public boolean hasSelection() {
		return !this.rect.isEmpty();
	}

	public Rectangle getSelection() {
		return this.rect;
	}

	public void startSelection(Point point) {
		if (point == null) {
			String msg = Logging.getMessage("nullValue.PointIsNull");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
		}

		this.startPoint.setLocation(point);
		this.endPoint.setLocation(point);
		this.rect.setRect(point.x, point.y, 0, 0);
	}

	public void endSelection(Point point) {
		if (point == null) {
			String msg = Logging.getMessage("nullValue.PointIsNull");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
		}

		this.endPoint.setLocation(point);

		// Compute the selection's extremes along the x axis.
		double minX, maxX;
		if (this.startPoint.x < this.endPoint.x) {
			minX = this.startPoint.x;
			maxX = this.endPoint.x;
		} else {
			minX = this.endPoint.x;
			maxX = this.startPoint.x;
		}

		// Compute the selection's extremes along the y axis. The selection is defined
		// in AWT screen coordinates, so
		// the origin is in the upper left corner and the y axis points down.
		double minY, maxY;
		if (this.startPoint.y < this.endPoint.y) {
			minY = this.startPoint.y;
			maxY = this.endPoint.y;
		} else {
			minY = this.endPoint.y;
			maxY = this.startPoint.y;
		}

		// If only one of the selection rectangle's dimensions is zero, then the
		// selection is either a horizontal or
		// vertical line. In this case, we set the zero dimension to 1 because both
		// dimensions must be nonzero to
		// perform a selection.
		if (minX == maxX && minY < maxY)
			maxX = minX + 1;
		if (minY == maxY && minX < maxX)
			minY = maxY - 1;

		this.rect.setRect(minX, maxY, maxX - minX, maxY - minY);
	}

	public void clearSelection() {
		this.startPoint.setLocation(0, 0);
		this.endPoint.setLocation(0, 0);
		this.rect.setRect(0, 0, 0, 0);
	}

	public Color getInteriorColor() {
		return this.interiorColor;
	}

	public void setInteriorColor(Color color) {
		this.interiorColor = color;
	}

	public Color getBorderColor() {
		return this.borderColor;
	}

	public void setBorderColor(Color color) {
		this.borderColor = color;
	}

	@Override
	public double getDistanceFromEye() {
		return 0; // Screen rectangle is drawn on top of other ordered renderables, except other
					// screen objects.
	}

	@Override
	public void pick(DrawContext dc, Point pickPoint) {
		// Intentionally left blank. SelectionRectangle is not pickable.
	}

	@Override
	public void render(DrawContext dc) {
		if (dc == null) {
			String msg = Logging.getMessage("nullValue.DrawContextIsNull");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
		}

		if (dc.isOrderedRenderingMode())
			this.drawOrderedRenderable(dc);
		else
			this.makeOrderedRenderable(dc);
	}

	protected void makeOrderedRenderable(DrawContext dc) {
		if (this.hasSelection())
			dc.addOrderedRenderable(this);
	}

	protected void drawOrderedRenderable(DrawContext dc) {
		int attrs = GL.GL_COLOR_BUFFER_BIT // For blend enable, alpha enable, blend func, alpha func.
				| GL2.GL_CURRENT_BIT // For current color.
				| GL.GL_DEPTH_BUFFER_BIT; // For depth test disable.

		Rectangle viewport = dc.getView().getViewport();
		Rectangle selection = this.getSelection();

		GL2 gl = dc.getGL().getGL2(); // GL initialization checks for GL2 compatibility.
		this.BEogsh.pushAttrib(gl, attrs);
		this.BEogsh.pushClientAttrib(gl, GLPointerFunc.GL_VERTEX_ARRAY);
		try {
			// Configure the modelview-projection matrix to transform vertex points from
			// screen rectangle
			// coordinates to clip coordinates without any perspective transformation. We
			// offset the rectangle by
			// 0.5 pixels to ensure that the line loop draws a line without a 1-pixel gap
			// between the line's
			// beginning and its end. We scale by (width - 1, height - 1) to ensure that
			// only the actual selected
			// area is filled. If we scaled by (width, height), GL line rasterization would
			// fill one pixel beyond
			// the actual selected area.
			this.BEogsh.pushProjectionIdentity(gl);
			gl.glOrtho(0, viewport.getWidth(), 0, viewport.getHeight(), -1, 1); // l, r, b, t, n, f
			this.BEogsh.pushModelviewIdentity(gl);
			gl.glTranslated(0.5, 0.5, 0.0);
			gl.glTranslated(selection.getX(), viewport.getHeight() - selection.getY(), 0);
			gl.glScaled(selection.getWidth() - 1, selection.getHeight() - 1, 1);

			// Disable the depth test and enable blending so this screen rectangle appears
			// on top of the existing
			// framebuffer contents.
			gl.glDisable(GL.GL_DEPTH_TEST);
			gl.glEnable(GL.GL_BLEND);
			OGLUtil.applyBlending(gl, false); // SelectionRectangle does not use pre-multiplied colors.

			// Draw this screen rectangle's interior as a filled quadrilateral.
			Color c = this.getInteriorColor() != null ? this.getInteriorColor() : DEFAULT_INTERIOR_COLOR;
			gl.glColor4ub((byte) c.getRed(), (byte) c.getGreen(), (byte) c.getBlue(), (byte) c.getAlpha());
			dc.drawUnitQuad();

			// Draw this screen rectangle's border as a line loop. This assumes the default
			// line width of 1.0.
			c = this.getBorderColor() != null ? this.getBorderColor() : DEFAULT_BORDER_COLOR;
			gl.glColor4ub((byte) c.getRed(), (byte) c.getGreen(), (byte) c.getBlue(), (byte) c.getAlpha());
			dc.drawUnitQuadOutline();
		} finally {
			this.BEogsh.pop(gl);
		}
	}
}
