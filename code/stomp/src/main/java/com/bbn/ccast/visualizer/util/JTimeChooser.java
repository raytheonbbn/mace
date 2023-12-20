//  Approved for public release: distribution is unlimited. PA Case Number AFRL-2023-4617.

//  © 2023 Raytheon BBN Technologies Corp. All rights reserved. Sponsored by the Air Force Research Laboratory (AFRL)  



package com.bbn.ccast.visualizer.util;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import java.awt.*;
import java.text.DecimalFormat;
import java.util.Date;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.logging.Logger;

/**
 *
 * @author Ruslan López Carro
 */
public final class JTimeChooser extends javax.swing.JPanel implements Runnable {

	private static final long serialVersionUID = 1L;

	private Date currentTime;
    private static final Logger LOGGER = Logger.getLogger(JTimeChooser.class.getName());
    // Variables declaration - do not modify//GEN-BEGIN:variables
    private JCheckBox currentTimeChk;
    private JSpinner hourSpin;
    private JSpinner minuteSpin;
    private JSpinner secondSpin;
    // End of variables declaration//GEN-END:variables

    /**
     * Creates a Hour and Minute Chooser with the specified values.
     */
    public JTimeChooser(Date date) {
        setName("JTimeChooser");
        initComponents();
        currentTime = date;
        updateCurrentTimeInGUI();
        startTimerThread();
    }

    /**
     * Creates a Hour and Minute Chooser with the current time set.
     */
    public JTimeChooser() {
        setName("JTimeChooser");
        initComponents();
        setCurrentTime();
        startTimerThread();
    }

    private void startTimerThread() {
        Thread timingThread = new Thread(this);
        timingThread.start();
    }

    @Override
    public void setEnabled(boolean enable) {
        hourSpin.setEnabled(enable);
        minuteSpin.setEnabled(enable);
        secondSpin.setEnabled(enable);
        currentTimeChk.setEnabled(enable);
    }

    public void setCurrentTime() {
        currentTime = new Date();
        updateCurrentTimeInGUI();
    }

    private void updateCurrentTimeInGUI() {
        LOGGER.finest(currentTime.toString());
        // System.out.println("minutes"+currentTime.getMinutes());
        hourSpin.setValue(numberFormat(currentTime.getHours(), "##"));
        minuteSpin.setValue(numberFormat(currentTime.getMinutes(), "##"));
        secondSpin.setValue(numberFormat(currentTime.getSeconds(), "##"));
    }

    public static String numberFormat(long src, String fmt) {//Format : ###.####        
        DecimalFormat df = new DecimalFormat(fmt.replaceAll("#", "0"));
        return df.format(src);
    }

    public String getHourStr() {
        return numberFormat(getHours(), "##");
    }

    public int getHours() {
        return Integer.parseInt(hourSpin.getValue().toString().trim());
    }

    public String getMinuteStr() {
        return numberFormat(getMinutes(), "##");
    }

    public int getMinutes() {
        return Integer.parseInt(minuteSpin.getValue().toString().trim());
    }

    public String getSecondsStr() {
        return numberFormat(getSeconds(), "##");
    }

    public int getSeconds() {
        return Integer.parseInt(secondSpin.getValue().toString().trim());
    }

    public Date getTime() {
        Date ret = new Date();
        ret.setHours(getHours());
        ret.setMinutes(getMinutes());
        ret.setSeconds(getSeconds());
        return ret;
    }

    public String getTimeStr() {
        return getHourStr() + ":" + getMinuteStr() + ":" + getSecondsStr();
    }

    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        hourSpin = new JSpinner();
        minuteSpin = new JSpinner();
        secondSpin = new JSpinner();
        currentTimeChk = new JCheckBox();

        setLayout(new FlowLayout(FlowLayout.LEFT, 3, 0));

        hourSpin.setFont(new Font("Tahoma", 1, 11)); // NOI18N
        hourSpin.setModel(new SpinnerListModel(new String[] {"00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24" }));
        add(hourSpin);

        minuteSpin.setFont(new Font("Tahoma", 1, 11)); // NOI18N
        minuteSpin.setModel(new SpinnerListModel(new String[] {"00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "31", "32", "33", "34", "35", "36", "37", "38", "39", "40", "41", "42", "43", "44", "45", "46", "47", "48", "49", "50", "51", "52", "53", "54", "55", "56", "57", "58", "59"}));
        add(minuteSpin);

        secondSpin.setFont(new Font("Tahoma", 1, 11)); // NOI18N
        secondSpin.setModel(new SpinnerListModel(new String[] {"00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "31", "32", "33", "34", "35", "36", "37", "38", "39", "40", "41", "42", "43", "44", "45", "46", "47", "48", "49", "50", "51", "52", "53", "54", "55", "56", "57", "58", "59"}));
        add(secondSpin);

        currentTimeChk.setText("currentTime");
        currentTimeChk.addChangeListener(new ChangeListener() {
            @Override
			public void stateChanged(ChangeEvent evt) {
                currentTimeChkStateChanged(evt);
            }
        });
        add(currentTimeChk);
    }// </editor-fold>//GEN-END:initComponents

    private void currentTimeChkStateChanged(ChangeEvent evt) {//GEN-FIRST:event_currentTimeChkStateChanged
        if (currentTimeChk.isSelected()) {
            hourSpin.setEnabled(false);
            minuteSpin.setEnabled(false);
            secondSpin.setEnabled(false);
        } else {
            hourSpin.setEnabled(true);
            minuteSpin.setEnabled(true);
            secondSpin.setEnabled(true);
        }
    }//GEN-LAST:event_currentTimeChkStateChanged

    @Override
    public void run() {
        final ScheduledExecutorService executorService
                = Executors.newSingleThreadScheduledExecutor();
        executorService.scheduleAtFixedRate(
                new Runnable() {
                    @Override
                    public void run() {
                        if (isEnabled() && currentTimeChk.isSelected()) {
                            setCurrentTime();
                        }
                    }
                }, 0, 1, TimeUnit.SECONDS
        );
    }

    public Date getCurrentTime() {
        return new Date(currentTime.getTime());
    }
}
