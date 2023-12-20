//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer;

import org.apache.log4j.Logger;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.net.URL;

public class WorldWindAppAboutDialog {

	private static final Logger logger = Logger.getLogger(WorldWindAppAboutDialog.class.getName());

	private Object content;
	private String contentType;
	private Dimension preferredSize;

	public WorldWindAppAboutDialog() {
		setContent("WorldWindAppAbout.html");
		setContentType("text/html");
		setPreferredSize(new Dimension(400, 230));
	}

	public Object getContent() {
		return this.content;
	}

	public void setContent(Object content) {
		this.content = content;
	}

	public String getContentType() {
		return this.contentType;
	}

	public void setContentType(String contentType) {
		this.contentType = contentType;
	}

	public Dimension getPreferredSize() {
		return this.preferredSize;
	}

	public void setPreferredSize(Dimension preferredSize) {
		this.preferredSize = preferredSize;
	}

	public void showDialog(Component parentComponent) {
		Component component = makeContentComponent();
		showContentDialog(parentComponent, component);
	}

	private static void showContentDialog(Component parentComponent, Component component) {
		try {
			final JDialog dialog;
			if (parentComponent instanceof Dialog)
				dialog = new JDialog((Dialog) parentComponent);
			else if (parentComponent instanceof Frame)
				dialog = new JDialog((Frame) parentComponent);
			else
				dialog = new JDialog();

			component.addMouseListener(new MouseAdapter() {
				@Override
				public void mouseClicked(MouseEvent event) {
					dialog.setVisible(false);
					dialog.dispose();
				}
			});

			dialog.getContentPane().setLayout(new BorderLayout());
			dialog.getContentPane().add(component, BorderLayout.CENTER);
			dialog.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
			dialog.setModal(true);
			dialog.setResizable(false);
			dialog.pack();
			centerWindowInDesktop(dialog);
			dialog.setVisible(true);
		} catch (Exception e) {
			String message = "Exception while displaying content dialog";
			logger.error(message);
		}
	}

	private Component makeContentComponent() {
		JEditorPane editor = null;
		try {
			if (this.content != null) {
				URL url = getClass().getResource(this.content.toString());
				editor = new JEditorPane();
				if (this.contentType != null)
					editor.setContentType(this.contentType);
				editor.setPage(url);
			}

			if (editor != null) {
				editor.setEditable(false);
				if (this.preferredSize != null)
					editor.setPreferredSize(this.preferredSize);
			}
		} catch (Exception e) {
			String message = "Exception while fetching content";
			logger.error(message);
			editor = null;
		}
		return editor;
	}

	public static void centerWindowInDesktop(Window window) {
		if (window != null) {
			int screenWidth = Toolkit.getDefaultToolkit().getScreenSize().width;
			int screenHeight = Toolkit.getDefaultToolkit().getScreenSize().height;
			Insets screenInsets = Toolkit.getDefaultToolkit().getScreenInsets(window.getGraphicsConfiguration());
			int desktopWidth = screenWidth - screenInsets.left - screenInsets.right;
			int desktopHeight = screenHeight - screenInsets.bottom - screenInsets.top;
			int frameWidth = window.getSize().width;
			int frameHeight = window.getSize().height;

			if (frameWidth > desktopWidth)
				frameWidth = Math.min(frameWidth, desktopWidth);
			if (frameHeight > desktopHeight)
				frameHeight = Math.min(frameHeight, desktopHeight);

			window.setPreferredSize(new Dimension(frameWidth, frameHeight));
			window.pack();
			window.setLocation((desktopWidth - frameWidth) / 2 + screenInsets.left,
					(desktopHeight - frameHeight) / 2 + screenInsets.top);
		}
	}

}
