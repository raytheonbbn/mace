//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



/*
 * Copyright (C) 2012 United States Government as represented by the Administrator of the
 * National Aeronautics and Space Administration.
 * All Rights Reserved.
 */
package com.bbn.ccast.visualizer.util;

import com.bbn.ccast.visualizer.tracks.SDOTrack;
import com.bbn.ccast.visualizer.tracks.SaveSDOFilter;

import javax.swing.*;
import javax.swing.border.EmptyBorder;
import javax.swing.filechooser.FileFilter;
import java.awt.*;
import java.io.File;

/**
 * @author dcollins
 * @version $Id: SaveTrackDialog.java 1171 2013-02-11 21:45:02Z dcollins $
 */
public class SaveSDODialog {
	private JFileChooser fileChooser;
	private JCheckBox saveAnnotations;

	public static final int APPROVE_OPTION = JFileChooser.APPROVE_OPTION;
	public static final int CANCEL_OPTION = JFileChooser.CANCEL_OPTION;
	public static final int ERROR_OPTION = JFileChooser.ERROR_OPTION;

	public SaveSDODialog() {
		initComponents();
	}

	public File getSelectedFile() {
		File file = this.fileChooser.getSelectedFile();
		if (file == null)
			return null;

		SaveSDOFilter filter = this.getSelectedSaveFilter();
		if (filter != null)
			file = filter.appendSuffix(file);

		return file;
	}

	public void setSelectedFile(File file) {
		this.fileChooser.setSelectedFile(file);
	}

	public void setSelectedFile(SDOTrack track) {
		if (track != null) {
			if (track.getFile() != null)
				this.fileChooser.setSelectedFile(track.getFile());
			else if (track.getName() != null && this.fileChooser.getCurrentDirectory() != null)
				this.fileChooser.setSelectedFile(new File(this.fileChooser.getCurrentDirectory(), track.getName()));
		}
	}

	public boolean isSaveAnnotations() {
		return this.saveAnnotations.isSelected();
	}

	public void setSaveAnnotations(boolean saveAnnotations) {
		this.saveAnnotations.setSelected(saveAnnotations);
	}

	public SaveSDOFilter getSelectedSaveFilter() {
		FileFilter filter = this.fileChooser.getFileFilter();
		return (filter != null && filter instanceof SaveSDOFilter) ? (SaveSDOFilter) filter : null;
	}

	public int getFileFormat() {
		SaveSDOFilter filter = this.getSelectedSaveFilter();
		return (filter != null) ? filter.getFormat() : 0;
	}

	public void setFileFormat(int format) {
		FileFilter ff = filterForFormat(format);
		if (ff != null)
			this.fileChooser.setFileFilter(ff);
	}

	public void setFileFormat(SDOTrack track) {
		if (track != null) {
			FileFilter ff = filterForFormat(track.getFormat());
			if (ff == null) // If the track format is invalid, default to MAVLink.
				ff = filterForFormat(SDOTrack.FORMAT_MAVLINK);
			if (ff != null)
				this.fileChooser.setFileFilter(ff);
		}
	}

	public File getCurrentDirectory() {
		return this.fileChooser.getCurrentDirectory();
	}

	public void setCurrentDirectory(File dir) {
		this.fileChooser.setCurrentDirectory(dir);
	}

	public String getDialogTitle() {
		return this.fileChooser.getDialogTitle();
	}

	public void setDialogTitle(String dialogTitle) {
		this.fileChooser.setDialogTitle(dialogTitle);
	}

	public void setDialogTitle(SDOTrack track) {
		String title = null;
		String formatString = "Save \"%s\" As";
		if (track.getName() != null)
			title = String.format(formatString, track.getName());
		else if (track.getFile() != null)
			title = String.format(formatString, track.getFile().getName());

		if (title != null)
			this.fileChooser.setDialogTitle(title);
	}

	public int showSaveDialog(Component parent) throws HeadlessException {
		return this.fileChooser.showSaveDialog(parent);
	}

	public static int showSaveChangesPrompt(Component parent, SDOTrack track) {
		String title = "Save";

		String formatString = "Save changes to the Track\n\"%s\" before closing?";
		String message = null;
		if (track != null && track.getName() != null) {
				message = String.format(formatString, track.getName());		
		} else if (track != null && track.getFile() != null) {
				message = String.format(formatString, track.getFile().getName());
		}

		return JOptionPane.showOptionDialog(parent, // parentComponent
				message, title, JOptionPane.YES_NO_CANCEL_OPTION, // optionType
				JOptionPane.WARNING_MESSAGE, // messageType
				null, // icon
				new Object[] { "Save", "Don't Save", "Cancel" }, // options
				"Save"); // initialValue
	}

	public static int showOverwritePrompt(Component parent, File file) {
		String title = "Save";
		String message = null;
		if (file != null)
			message = String.format("Overwrite existing file\n\"%s\"?", file.getPath());
		else
			message = "Overwrite existing file?";

		return JOptionPane.showOptionDialog(parent, // parentComponent
				message, title, JOptionPane.YES_NO_OPTION, // optionType
				JOptionPane.WARNING_MESSAGE, // messageType
				null, // icon
				new Object[] { "Overwrite", "Cancel" }, // options
				"Overwrite"); // initialValue
	}

	private void initComponents() {
		this.fileChooser = new JFileChooser() {

			private static final long serialVersionUID = 1L;

			@Override
			public void approveSelection() {
				if (doApproveSelection())
					super.approveSelection();
			}
		};
		this.fileChooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
		this.fileChooser.setMultiSelectionEnabled(false);
		makeAccessory();
		makeFileFilters();
	}

	private boolean doApproveSelection() {
		File f = this.getSelectedFile();
		if (f != null && f.exists()) {
			int state = showOverwritePrompt(this.fileChooser, f);
			if (state != JOptionPane.YES_OPTION)
				return false;
		}

		return true;
	}

	private void makeAccessory() {
		Box box = Box.createVerticalBox();
		box.setBorder(new EmptyBorder(0, 10, 0, 10));

		JLabel label = new JLabel("Options");
		label.setAlignmentX(Component.LEFT_ALIGNMENT);
		box.add(label);
		box.add(Box.createVerticalStrut(5));

		this.saveAnnotations = new JCheckBox("Save Annotations");
		this.saveAnnotations.setAlignmentX(Component.LEFT_ALIGNMENT);
		this.saveAnnotations.setSelected(true);
		box.add(this.saveAnnotations);

		this.fileChooser.setAccessory(box);
	}

	private void makeFileFilters() {
		FileFilter[] filters = new FileFilter[] {
				new SaveSDOFilter(SDOTrack.FORMAT_MAVLINK, "MAVLink (*.mavlink)",
						new String[] { ".mavlink" }), };

		for (FileFilter filter : filters) {
			this.fileChooser.addChoosableFileFilter(filter);
		}

		this.fileChooser.setAcceptAllFileFilterUsed(false);
		this.fileChooser.setFileFilter(filters[0]);
	}

	private FileFilter filterForFormat(int format) {
		FileFilter result = null;

		for (FileFilter filter : this.fileChooser.getChoosableFileFilters()) {
			if (filter instanceof SaveSDOFilter) {
				if (((SaveSDOFilter) filter).getFormat() == format) {
					result = filter;
					break;
				}
			}
		}

		return result;
	}
}
