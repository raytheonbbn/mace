//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import gov.nasa.worldwind.WWObjectImpl;
import gov.nasa.worldwind.WorldWindow;
import gov.nasa.worldwind.event.Message;
import gov.nasa.worldwind.event.MessageListener;
import gov.nasa.worldwind.event.SelectEvent;
import gov.nasa.worldwind.event.SelectListener;
import gov.nasa.worldwind.layers.Layer;
import gov.nasa.worldwind.layers.LayerList;
import gov.nasa.worldwind.layers.RenderableLayer;
import gov.nasa.worldwind.util.Logging;
import org.apache.log4j.Logger;

import java.awt.*;
import java.awt.event.InputEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.ArrayList;
import java.util.List;

public class CcastScreenSelector extends WWObjectImpl implements MouseListener, MouseMotionListener, SelectListener {

	private static final Logger logger = Logger.getLogger(CcastScreenSelector.class.getName());
	/**
	 * Message type indicating that the user has started their selection. The
	 * ScreenSelector's list of selected objects is empty, and does not change until
	 * a subsequent SELECTION_CHANGED event, if any. If this is followed by a
	 * SELECTION_ENDED event without any SELECTION_CHANGED event in between, then
	 * the user has selected nothing.
	 */
	public static final String SELECTION_STARTED = "ScreenSelector.SelectionStarted";
	/**
	 * Message type indicating that the list of selected objects has changed. This
	 * may be followed by one or more SELECTION_CHANGED events before the final
	 * SELECTION_ENDED event.
	 */
	public static final String SELECTION_CHANGED = "ScreenSelector.SelectionChanged";
	/**
	 * Message type indicating that the user has completed their selection. The
	 * ScreenSelector's list of selected objects does not changes until a subsequent
	 * SELECTION_STARTED event, if any.
	 */
	public static final String SELECTION_ENDED = "ScreenSelector.SelectionEnded";

	protected WorldWindow wwd;
	protected Layer layer;
	protected CcastSelectionRectangle selectionRect;
	protected List<Object> selectedObjects = new ArrayList<Object>();
	protected List<MessageListener> messageListeners = new ArrayList<MessageListener>();
	protected boolean armed;

	public CcastScreenSelector(WorldWindow worldWindow) {
		if (worldWindow == null) {
			String msg = Logging.getMessage("nullValue.WorldWindow");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
		}

		this.wwd = worldWindow;
		this.layer = this.createLayer();
		this.layer.setPickEnabled(false); // The screen selector is not pickable.
		this.selectionRect = this.createSelectionRectangle();
		((RenderableLayer) this.layer).addRenderable(this.selectionRect);
	}

	protected Layer createLayer() {
		return new RenderableLayer();
	}

	protected CcastSelectionRectangle createSelectionRectangle() {
		return new CcastSelectionRectangle();
	}

	public WorldWindow getWwd() {
		return this.wwd;
	}

	public Layer getLayer() {
		return this.layer;
	}

	public Color getInteriorColor() {
		return this.selectionRect.getInteriorColor();
	}

	public void setInteriorColor(Color color) {
		this.selectionRect.setInteriorColor(color);
	}

	public Color getBorderColor() {
		return this.selectionRect.getBorderColor();
	}

	public void setBorderColor(Color color) {
		this.selectionRect.setBorderColor(color);
	}

	public void enable() {
		// Clear any existing selection and clear set the SceneController's pick
		// rectangle. This ensures that this
		// ScreenSelector starts with the correct state when enabled.
		this.selectionRect.clearSelection();
		this.getWwd().getSceneController().setPickRectangle(null);

		// Add and enable the layer that displays this ScreenSelector's selection
		// rectangle.
		LayerList layers = this.getWwd().getModel().getLayers();

		if (!layers.contains(this.getLayer()))
			layers.add(this.getLayer());

		if (!this.getLayer().isEnabled())
			this.getLayer().setEnabled(true);

		// Listen for mouse input on the WorldWindow.
		this.getWwd().getInputHandler().addMouseListener(this);
		this.getWwd().getInputHandler().addMouseMotionListener(this);
	}

	public void disable() {
		// Clear the selection, clear the SceneController's pick rectangle, and stop
		// listening for changes in the pick
		// rectangle selection. These steps should have been done when the selection
		// ends, but do them here in case the
		// caller disables this ScreenSelector before the selection ends.
		this.selectionRect.clearSelection();
		this.getWwd().getSceneController().setPickRectangle(null);
		this.getWwd().removeSelectListener(this);

		// Remove the layer that displays this ScreenSelector's selection rectangle.
		this.getWwd().getModel().getLayers().remove(this.getLayer());

		// Stop listening for mouse input on the WorldWindow.
		this.getWwd().getInputHandler().removeMouseListener(this);
		this.getWwd().getInputHandler().removeMouseMotionListener(this);
	}

	public List<?> getSelectedObjects() {
		return this.selectedObjects;
	}

	public void addMessageListener(MessageListener listener) {
		if (listener == null) {
			String msg = Logging.getMessage("nullValue.ListenerIsNull");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
		}

		this.messageListeners.add(listener);
	}

	public void removeMessageListener(MessageListener listener) {
		if (listener == null) {
			String msg = Logging.getMessage("nullValue.ListenerIsNull");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
		}

		this.messageListeners.remove(listener);
	}

	protected void sendMessage(Message message) {
		for (MessageListener listener : this.messageListeners) {
			try {
				listener.onMessage(message);
			} catch (Exception e) {
				String msg = Logging.getMessage("generic.ExceptionInvokingMessageListener");
				Logging.logger().severe(msg);
				// Don't throw an exception, just log a severe message and continue to the next
				// listener.
			}
		}
	}

	@Override
	public void mouseClicked(MouseEvent mouseEvent) {
		// Intentionally left blank. ScreenSelector does not respond to mouse clicked
		// events.
	}

	@Override
	public void mousePressed(MouseEvent mouseEvent) {
		if (mouseEvent == null) // Ignore null events.
			return;

		    int onmask = InputEvent.ALT_DOWN_MASK | InputEvent.BUTTON1_DOWN_MASK;
		    if ((mouseEvent.getModifiersEx() & onmask) == onmask) {
				this.armed = true;
				this.selectionStarted(mouseEvent);
				mouseEvent.consume(); // Consume the mouse event to prevent the view from responding to it.
		    } else {
		    		return;
		    }
		 
	}

	@Override
	public void mouseReleased(MouseEvent mouseEvent) {
		if (mouseEvent == null) // Ignore null events.
			return;

		if (!this.armed) // Respond to mouse released events when armed.
			return;

		this.armed = false;
		this.selectionEnded(mouseEvent);
		mouseEvent.consume(); // Consume the mouse event to prevent the view from responding to it.
	}

	@Override
	public void mouseEntered(MouseEvent mouseEvent) {
		// Intentionally left blank. ScreenSelector does not respond to mouse entered
		// events.
	}

	@Override
	public void mouseExited(MouseEvent mouseEvent) {
		// Intentionally left blank. ScreenSelector does not respond to mouse exited
		// events.
	}

	@Override
	public void mouseDragged(MouseEvent mouseEvent) {
		if (mouseEvent == null) // Ignore null events.
			return;

		if (!this.armed) // Respond to mouse dragged events when armed.
			return;

		this.selectionChanged(mouseEvent);
		mouseEvent.consume(); // Consume the mouse event to prevent the view from responding to it.
	}

	@Override
	public void mouseMoved(MouseEvent mouseEvent) {
		// Intentionally left blank. ScreenSelector does not respond to mouse moved
		// events.
	}

	protected void selectionStarted(MouseEvent mouseEvent) {
		this.selectionRect.startSelection(mouseEvent.getPoint());
		this.getWwd().getSceneController().setPickRectangle(null);
		this.getWwd().addSelectListener(this); // Listen for changes in the pick rectangle selection.
		this.getWwd().redraw();

		// Clear the list of selected objects and send a message indicating that the
		// user has started a selection.
		this.selectedObjects.clear();
		this.sendMessage(new Message(SELECTION_STARTED, this));
	}

	@SuppressWarnings("unused")
	protected void selectionEnded(MouseEvent mouseEvent) {
		this.selectionRect.clearSelection();
		this.getWwd().getSceneController().setPickRectangle(null);
		this.getWwd().removeSelectListener(this); // Stop listening for changes the pick rectangle selection.
		this.getWwd().redraw();

		// Send a message indicating that the user has completed their selection. We
		// don't clear the list of selected
		// objects in order to preserve the list of selected objects for the caller.
		this.sendMessage(new Message(SELECTION_ENDED, this));
	}

	protected void selectionChanged(MouseEvent mouseEvent) {
		// Limit the end point to the WorldWindow's viewport rectangle. This ensures
		// that a user drag event to define
		// the selection does not exceed the viewport and the viewing frustum. This is
		// only necessary during mouse drag
		// events because those events are reported when the cursor is outside the
		// WorldWindow's viewport.
		Point p = this.limitPointToWorldWindow(mouseEvent.getPoint());

		// Specify the selection's end point and set the scene controller's pick
		// rectangle to the selected rectangle.
		// We create a copy of the selected rectangle to insulate the scene controller
		// from changes to rectangle
		// returned by ScreenRectangle.getSelection.
		this.selectionRect.endSelection(p);
		this.getWwd().getSceneController().setPickRectangle(
				this.selectionRect.hasSelection() ? new Rectangle(this.selectionRect.getSelection()) : null);
		this.getWwd().redraw();
	}

	/**
	 * Limits the specified point's x and y coordinates to the WorldWindow's
	 * viewport, and returns a new point with the limited coordinates. For example,
	 * if the WorldWindow's viewport rectangle is x=0, y=0, width=100, height=100
	 * and the point's coordinates are x=50, y=200 this returns a new point with
	 * coordinates x=50, y=100. If the specified point is already inside the
	 * WorldWindow's viewport, this returns a new point with the same x and y
	 * coordinates as the specified point.
	 *
	 * @param point
	 *            the point to limit.
	 *
	 * @return a new Point representing the specified point limited to the
	 *         WorldWindow's viewport rectangle.
	 */
	protected Point limitPointToWorldWindow(Point point) {
		Rectangle viewport = this.getWwd().getView().getViewport();

		int x = point.x;
		if (x < viewport.x)
			x = viewport.x;
		if (x > viewport.x + viewport.width)
			x = viewport.x + viewport.width;

		int y = point.y;
		if (y < viewport.y)
			y = viewport.y;
		if (y > viewport.y + viewport.height)
			y = viewport.y + viewport.height;

		return new Point(x, y);
	}

	@Override
	public void selected(SelectEvent event) {
		try {
			// Respond to box rollover select events when armed.
			if (event.getEventAction().equals(SelectEvent.BOX_ROLLOVER) && this.armed)
				this.selectObjects(event.getAllTopObjects());
		} catch (Exception e) {
			// Wrap the handler in a try/catch to keep exceptions from bubbling up
			logger.warn(e.getMessage() != null ? e.getMessage() : e.toString());
		}
	}

	protected void selectObjects(List<?> list) {
		if (this.selectedObjects.equals(list))
			return; // Same thing selected.

		this.selectedObjects.clear();

		// If the selection is empty, then we've cleared the list of selected objects
		// and there's nothing left to do.
		// Otherwise, we add the selected objects to our list.
		if (list != null)
			this.selectedObjects.addAll(list);

		// Send a message indicating that the user has ended selection. We don't clear
		// the list of selected objects
		// in order to preserve the list of selected objects for the caller.
		this.sendMessage(new Message(SELECTION_CHANGED, this));
	}

	public boolean isEnabled() {
		LayerList layers = this.getWwd().getModel().getLayers();
		if (layers.contains(this.getLayer())) {
			return true;
		}
		return false;
	}
}
