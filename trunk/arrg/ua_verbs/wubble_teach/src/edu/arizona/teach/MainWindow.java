/*
 * MainWindow.java
 */
package edu.arizona.teach;

import ros.NodeHandle;
import ros.Ros;
import ros.pkg.oomdp_msgs.msg.MDPClassDescription;
import ros.pkg.oomdp_msgs.srv.DescribeMDP;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.event.*;
import org.jdesktop.application.Action;
import org.jdesktop.application.ResourceMap;
import org.jdesktop.application.SingleFrameApplication;
import org.jdesktop.application.FrameView;
import org.jdesktop.application.TaskMonitor;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.FileInputStream;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.Timer;
import javax.swing.Icon;
import javax.swing.ImageIcon;
import javax.swing.JComponent;
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import javax.swing.border.BevelBorder;
import ros.RosException;
import ros.ServiceClient;
import ros.pkg.oomdp_msgs.msg.MDPObjectState;
import ros.pkg.oomdp_msgs.srv.InitializeEnvironment;

/**
 * The application's main frame.
 */
public class MainWindow extends FrameView {

    public MainWindow(SingleFrameApplication app) {
        super(app);

        initComponents();

        jButton7.setEnabled(false); //disable BUILD VERB and TRAIN VERB buttons, since I can't alter initialization code
        jButton5.setEnabled(false);

        setScenarios();

        setVerbs();

        // status bar initialization - message timeout, idle icon and busy animation, etc
        ResourceMap resourceMap = getResourceMap();
        int messageTimeout = resourceMap.getInteger("StatusBar.messageTimeout");
        messageTimer = new Timer(messageTimeout, new ActionListener() {

            public void actionPerformed(ActionEvent e) {
                //took out statusMessageLabel setText()
            }
        });
        messageTimer.setRepeats(false);
        int busyAnimationRate = resourceMap.getInteger("StatusBar.busyAnimationRate");
        for (int i = 0; i < busyIcons.length; i++) {
            busyIcons[i] = resourceMap.getIcon("StatusBar.busyIcons[" + i + "]");
        }
        busyIconTimer = new Timer(busyAnimationRate, new ActionListener() {

            public void actionPerformed(ActionEvent e) {
                busyIconIndex = (busyIconIndex + 1) % busyIcons.length;
                //took out statusAnimationIcon setIcon()
            }
        });
        idleIcon = resourceMap.getIcon("StatusBar.idleIcon");
        //took out statusAnimationLabel.setIcon(idleIcon);
        //took out progressBar.setVisible(false);

        // connecting action tasks to status bar via TaskMonitor
        TaskMonitor taskMonitor = new TaskMonitor(getApplication().getContext());
        taskMonitor.addPropertyChangeListener(new java.beans.PropertyChangeListener() {

            public void propertyChange(java.beans.PropertyChangeEvent evt) {
                String propertyName = evt.getPropertyName();
                /*if ("started".equals(propertyName)) {
                if (!busyIconTimer.isRunning()) {
                statusAnimationLabel.setIcon(busyIcons[0]);
                busyIconIndex = 0;
                busyIconTimer.start();
                }
                progressBar.setVisible(true);
                progressBar.setIndeterminate(true);
                } else if ("done".equals(propertyName)) {
                busyIconTimer.stop();
                statusAnimationLabel.setIcon(idleIcon);
                progressBar.setVisible(false);
                progressBar.setValue(0);
                } else if ("message".equals(propertyName)) {
                String text = (String)(evt.getNewValue());
                statusMessageLabel.setText((text == null) ? "" : text);
                messageTimer.restart();
                } else if ("progress".equals(propertyName)) {
                int value = (Integer)(evt.getNewValue());
                progressBar.setVisible(true);
                progressBar.setIndeterminate(false);
                progressBar.setValue(value);
                }*/
            }
        });
    }

    public void makeChooseVerb() {
        ChooseVerbButton.setEnabled(true);
    }

    public void makeBuildVerb() {
        jButton7.setEnabled(true);
    }

    public void makeTrainVerb() {
        jButton5.setEnabled(true);
    }

    public void setScenarios() {
        /* set scenario combo box with existing scenario names */
        File scenarioBank = new File("__wubble_scenarios__");    //__wubble_scenarios__ is name of parent folder
        File[] scenarioFiles = scenarioBank.listFiles();
        int len;
        if (scenarioFiles != null) {
            len = scenarioFiles.length;      //number of scenarios that already exist
        } else {
            len = 0;
        }
        String[] scenarioNames = new String[len];    //make String[] of scenario names
        for (int i = 0; i < len; i++) {
            scenarioNames[i] = scenarioFiles[i].getName();
        }

        String[] scenarioOptions = new String[len + 3];    //array to hold scenario choices
        scenarioOptions[0] = "CHOOSE SCENARIO";
        scenarioOptions[1] = "create scenario";
        scenarioOptions[2] = "delete scenario";
        for (int i = 0; i < len; i++) {
            scenarioOptions[i + 3] = scenarioNames[i];
        }
        ChooseScenarioButton.setModel(new javax.swing.DefaultComboBoxModel(scenarioOptions));
    }

    public void setVerbs() {

        /* set verb combo box with existing verb names */
        File verbBank = new File("__wubble_verbs__");    //__wubble_verbs__ is name of parent folder
        File[] verbFiles = verbBank.listFiles();
        int vlen;
        if (verbFiles != null) {
            vlen = verbFiles.length;      //number of verbs that already exist
        } else {
            vlen = 0;
        }
        String[] verbNames = new String[vlen];    //make String[] of verb names
        for (int i = 0; i < vlen; i++) {
            verbNames[i] = verbFiles[i].getName();
        }

        String[] verbOptions = new String[vlen + 2];    //array to hold verb choices
        verbOptions[0] = "CHOOSE VERB";
        verbOptions[1] = "create verb";
        for (int i = 0; i < vlen; i++) {
            verbOptions[i + 2] = verbNames[i];
        }
        ChooseVerbButton.setModel(new javax.swing.DefaultComboBoxModel(verbOptions));
        ChooseVerbButton.setEnabled(false);

    }

    @Action
    public void showAboutBox() {
        if (aboutBox == null) {
            JFrame mainFrame = WubbleTeach.getApplication().getMainFrame();
            aboutBox = new TeachVerbsMockApp2AboutBox(mainFrame);
            aboutBox.setLocationRelativeTo(mainFrame);
        }
        WubbleTeach.getApplication().show(aboutBox);
    }

    /*set parameter values in the item table */
    public void setParams(ArrayList<String> paramVals) {
        System.out.println("The size of paramVals is " + paramVals.size());
        String[][] paramArray = new String[paramVals.size()][2];
        for (int i = 0, j = 0; j < paramVals.size(); i++, j += 2) {
            paramArray[i][0] = paramVals.get(j);
            paramArray[i][1] = paramVals.get(j + 1);
        }
        clearParamTable();
        jTable1.setModel(new javax.swing.table.DefaultTableModel( //replace table with model based on passed vals
                paramArray,
                new String[]{
                    "parameter", "item"
                }));
    }

    /*set scenario name in view panel */
    public void setScenarioName(String thisName) {
        jTextField1.setForeground(Color.black);
        jTextField1.setText(thisName);
    }

    /*set verb name in view panel */
    public void setVerbName(String thisName) {
        System.out.println("You are in setVerbName.");
        jTextField2.setForeground(Color.black);
        jTextField2.setText(thisName);
    }

    /*get scenario name in view panel */
    public String getScenarioName() {
        return jTextField1.getText();
    }

    /*get verb name in view panel */
    public String getVerbName() {
        return jTextField2.getText();
    }

    /** This method is called from within the constructor to
     * initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is
     * always regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        mainPanel = new javax.swing.JPanel();
        jPanel4 = new javax.swing.JPanel();
        jPanel5 = new javax.swing.JPanel();
        jLabel3 = new javax.swing.JLabel();
        jTextField1 = new javax.swing.JTextField();
        jTextField2 = new javax.swing.JTextField();
        jLabel4 = new javax.swing.JLabel();
        jButton15 = new javax.swing.JButton();
        jButton16 = new javax.swing.JButton();
        jScrollPane1 = new javax.swing.JScrollPane();
        jTable1 = new javax.swing.JTable();
        jButton1 = new javax.swing.JButton();
        jPanel2 = new javax.swing.JPanel();
        jPanel3 = new javax.swing.JPanel();
        jLabel2 = new javax.swing.JLabel();
        jPanel12 = new javax.swing.JPanel();
        jPanel13 = new javax.swing.JPanel();
        jLabel9 = new javax.swing.JLabel();
        ChooseScenarioButton = new javax.swing.JComboBox();
        jPanel14 = new javax.swing.JPanel();
        jLabel10 = new javax.swing.JLabel();
        jButton5 = new javax.swing.JButton();
        jPanel15 = new javax.swing.JPanel();
        jLabel11 = new javax.swing.JLabel();
        ChooseVerbButton = new javax.swing.JComboBox();
        jPanel16 = new javax.swing.JPanel();
        jLabel12 = new javax.swing.JLabel();
        jButton7 = new javax.swing.JButton();

        mainPanel.setName("mainPanel"); // NOI18N

        jPanel4.setName("jPanel4"); // NOI18N

        jPanel5.setName("jPanel5"); // NOI18N

        org.jdesktop.application.ResourceMap resourceMap = org.jdesktop.application.Application.getInstance(edu.arizona.teach.WubbleTeach.class).getContext().getResourceMap(MainWindow.class);
        jLabel3.setText(resourceMap.getString("jLabel3.text")); // NOI18N
        jLabel3.setName("jLabel3"); // NOI18N

        jTextField1.setEditable(false);
        jTextField1.setForeground(resourceMap.getColor("jTextField1.foreground")); // NOI18N
        jTextField1.setText(resourceMap.getString("jTextField1.text")); // NOI18N
        jTextField1.setName("jTextField1"); // NOI18N
        jTextField1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jTextField1ActionPerformed(evt);
            }
        });

        jTextField2.setForeground(resourceMap.getColor("jTextField2.foreground")); // NOI18N
        jTextField2.setText(resourceMap.getString("jTextField2.text")); // NOI18N
        jTextField2.setName("jTextField2"); // NOI18N

        jLabel4.setText(resourceMap.getString("jLabel4.text")); // NOI18N
        jLabel4.setName("jLabel4"); // NOI18N

        jButton15.setForeground(resourceMap.getColor("jButton15.foreground")); // NOI18N
        jButton15.setText(resourceMap.getString("jButton15.text")); // NOI18N
        jButton15.setToolTipText(resourceMap.getString("jButton15.toolTipText")); // NOI18N
        jButton15.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.RAISED));
        jButton15.setName("jButton15"); // NOI18N
        jButton15.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButton15ActionPerformed(evt);
            }
        });

        jButton16.setForeground(resourceMap.getColor("jButton16.foreground")); // NOI18N
        jButton16.setText(resourceMap.getString("jButton16.text")); // NOI18N
        jButton16.setToolTipText(resourceMap.getString("jButton16.toolTipText")); // NOI18N
        jButton16.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.RAISED));
        jButton16.setName("jButton16"); // NOI18N
        jButton16.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButton16ActionPerformed(evt);
            }
        });

        jScrollPane1.setName("jScrollPane1"); // NOI18N

        jTable1.setModel(new javax.swing.table.DefaultTableModel(
            new Object [][] {
                {null, null},
                {null, null},
                {null, null},
                {null, null}
            },
            new String [] {
                "parameter", "item"
            }
        ));
        jTable1.setName("jTable1"); // NOI18N
        jScrollPane1.setViewportView(jTable1);
        jTable1.getColumnModel().getColumn(0).setHeaderValue(resourceMap.getString("jTable1.columnModel.title0")); // NOI18N
        jTable1.getColumnModel().getColumn(1).setHeaderValue(resourceMap.getString("jTable1.columnModel.title1")); // NOI18N

        org.jdesktop.layout.GroupLayout jPanel5Layout = new org.jdesktop.layout.GroupLayout(jPanel5);
        jPanel5.setLayout(jPanel5Layout);
        jPanel5Layout.setHorizontalGroup(
            jPanel5Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(org.jdesktop.layout.GroupLayout.TRAILING, jPanel5Layout.createSequentialGroup()
                .add(jPanel5Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.TRAILING)
                    .add(org.jdesktop.layout.GroupLayout.LEADING, jScrollPane1, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, 295, Short.MAX_VALUE)
                    .add(jPanel5Layout.createSequentialGroup()
                        .add(jPanel5Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
                            .add(jLabel3)
                            .add(jLabel4))
                        .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .add(jPanel5Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING, false)
                            .add(org.jdesktop.layout.GroupLayout.TRAILING, jPanel5Layout.createSequentialGroup()
                                .add(jTextField1, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 163, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                                .add(jButton15, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 16, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE))
                            .add(org.jdesktop.layout.GroupLayout.TRAILING, jPanel5Layout.createSequentialGroup()
                                .add(jTextField2)
                                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                                .add(jButton16, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 16, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)))))
                .addContainerGap())
        );
        jPanel5Layout.setVerticalGroup(
            jPanel5Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel5Layout.createSequentialGroup()
                .addContainerGap(org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .add(jPanel5Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.BASELINE)
                    .add(jTextField1, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                    .add(jButton15, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 14, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                    .add(jLabel3))
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(jPanel5Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.BASELINE)
                    .add(jTextField2, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                    .add(jButton16, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 14, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                    .add(jLabel4, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 16, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE))
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(jScrollPane1, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 101, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .add(39, 39, 39))
        );

        javax.swing.ActionMap actionMap = org.jdesktop.application.Application.getInstance(edu.arizona.teach.WubbleTeach.class).getContext().getActionMap(MainWindow.class, this);
        jButton1.setAction(actionMap.get("quit")); // NOI18N
        jButton1.setText(resourceMap.getString("jButton1.text")); // NOI18N
        jButton1.setName("jButton1"); // NOI18N

        org.jdesktop.layout.GroupLayout jPanel4Layout = new org.jdesktop.layout.GroupLayout(jPanel4);
        jPanel4.setLayout(jPanel4Layout);
        jPanel4Layout.setHorizontalGroup(
            jPanel4Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel4Layout.createSequentialGroup()
                .addContainerGap()
                .add(jPanel4Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
                    .add(org.jdesktop.layout.GroupLayout.TRAILING, jPanel4Layout.createSequentialGroup()
                        .add(jPanel5, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, 302, Short.MAX_VALUE)
                        .add(21, 21, 21))
                    .add(org.jdesktop.layout.GroupLayout.TRAILING, jPanel4Layout.createSequentialGroup()
                        .add(jButton1, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 93, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                        .add(34, 34, 34))))
        );
        jPanel4Layout.setVerticalGroup(
            jPanel4Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel4Layout.createSequentialGroup()
                .add(39, 39, 39)
                .add(jPanel5, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED, 116, Short.MAX_VALUE)
                .add(jButton1, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 47, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addContainerGap())
        );

        jPanel2.setName("jPanel2"); // NOI18N

        jPanel3.setName("jPanel3"); // NOI18N

        jLabel2.setFont(resourceMap.getFont("jLabel2.font")); // NOI18N
        jLabel2.setText(resourceMap.getString("jLabel2.text")); // NOI18N
        jLabel2.setName("jLabel2"); // NOI18N

        org.jdesktop.layout.GroupLayout jPanel3Layout = new org.jdesktop.layout.GroupLayout(jPanel3);
        jPanel3.setLayout(jPanel3Layout);
        jPanel3Layout.setHorizontalGroup(
            jPanel3Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(org.jdesktop.layout.GroupLayout.TRAILING, jPanel3Layout.createSequentialGroup()
                .add(20, 20, 20)
                .add(jLabel2, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .add(333, 333, 333))
        );
        jPanel3Layout.setVerticalGroup(
            jPanel3Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel3Layout.createSequentialGroup()
                .addContainerGap()
                .add(jLabel2, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 35, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(14, Short.MAX_VALUE))
        );

        jPanel12.setName("jPanel12"); // NOI18N

        jPanel13.setBackground(resourceMap.getColor("jPanel13.background")); // NOI18N
        jPanel13.setName("jPanel13"); // NOI18N

        jLabel9.setFont(resourceMap.getFont("jLabel9.font")); // NOI18N
        jLabel9.setForeground(resourceMap.getColor("jLabel9.foreground")); // NOI18N
        jLabel9.setText(resourceMap.getString("jLabel9.text")); // NOI18N
        jLabel9.setName("jLabel9"); // NOI18N

        ChooseScenarioButton.setModel(new javax.swing.DefaultComboBoxModel(new String[] { "CHOOSE SCENARIO", "create scenario" }));
        ChooseScenarioButton.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.RAISED));
        ChooseScenarioButton.setName("ChooseScenarioButton"); // NOI18N
        ChooseScenarioButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                ChooseScenarioButtonActionPerformed(evt);
            }
        });

        org.jdesktop.layout.GroupLayout jPanel13Layout = new org.jdesktop.layout.GroupLayout(jPanel13);
        jPanel13.setLayout(jPanel13Layout);
        jPanel13Layout.setHorizontalGroup(
            jPanel13Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel13Layout.createSequentialGroup()
                .addContainerGap()
                .add(jLabel9)
                .add(18, 18, 18)
                .add(ChooseScenarioButton, 0, 232, Short.MAX_VALUE)
                .addContainerGap())
        );
        jPanel13Layout.setVerticalGroup(
            jPanel13Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel13Layout.createSequentialGroup()
                .addContainerGap()
                .add(jPanel13Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.BASELINE)
                    .add(jLabel9)
                    .add(ChooseScenarioButton, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE))
                .addContainerGap(45, Short.MAX_VALUE))
        );

        jPanel14.setBackground(resourceMap.getColor("jPanel14.background")); // NOI18N
        jPanel14.setName("jPanel14"); // NOI18N

        jLabel10.setFont(resourceMap.getFont("jLabel10.font")); // NOI18N
        jLabel10.setForeground(resourceMap.getColor("jLabel10.foreground")); // NOI18N
        jLabel10.setText(resourceMap.getString("jLabel10.text")); // NOI18N
        jLabel10.setName("jLabel10"); // NOI18N

        jButton5.setText(resourceMap.getString("jButton5.text")); // NOI18N
        jButton5.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.RAISED));
        jButton5.setName("jButton5"); // NOI18N
        jButton5.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButton5ActionPerformed(evt);
            }
        });

        org.jdesktop.layout.GroupLayout jPanel14Layout = new org.jdesktop.layout.GroupLayout(jPanel14);
        jPanel14.setLayout(jPanel14Layout);
        jPanel14Layout.setHorizontalGroup(
            jPanel14Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel14Layout.createSequentialGroup()
                .addContainerGap()
                .add(jLabel10)
                .add(18, 18, 18)
                .add(jButton5, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, 232, Short.MAX_VALUE)
                .addContainerGap())
        );
        jPanel14Layout.setVerticalGroup(
            jPanel14Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel14Layout.createSequentialGroup()
                .addContainerGap()
                .add(jPanel14Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.BASELINE)
                    .add(jLabel10)
                    .add(jButton5, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 26, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE))
                .addContainerGap(44, Short.MAX_VALUE))
        );

        jPanel15.setBackground(resourceMap.getColor("jPanel15.background")); // NOI18N
        jPanel15.setName("jPanel15"); // NOI18N

        jLabel11.setFont(resourceMap.getFont("jLabel11.font")); // NOI18N
        jLabel11.setForeground(resourceMap.getColor("jLabel11.foreground")); // NOI18N
        jLabel11.setText(resourceMap.getString("jLabel11.text")); // NOI18N
        jLabel11.setName("jLabel11"); // NOI18N

        ChooseVerbButton.setModel(new javax.swing.DefaultComboBoxModel(new String[] { "CHOOSE VERB" }));
        ChooseVerbButton.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.RAISED));
        ChooseVerbButton.setName("ChooseVerbButton"); // NOI18N
        ChooseVerbButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                ChooseVerbButtonActionPerformed(evt);
            }
        });

        org.jdesktop.layout.GroupLayout jPanel15Layout = new org.jdesktop.layout.GroupLayout(jPanel15);
        jPanel15.setLayout(jPanel15Layout);
        jPanel15Layout.setHorizontalGroup(
            jPanel15Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel15Layout.createSequentialGroup()
                .addContainerGap()
                .add(jLabel11)
                .add(18, 18, 18)
                .add(ChooseVerbButton, 0, 232, Short.MAX_VALUE)
                .addContainerGap())
        );
        jPanel15Layout.setVerticalGroup(
            jPanel15Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel15Layout.createSequentialGroup()
                .addContainerGap()
                .add(jPanel15Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.BASELINE)
                    .add(jLabel11)
                    .add(ChooseVerbButton, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE))
                .addContainerGap(44, Short.MAX_VALUE))
        );

        jPanel16.setBackground(resourceMap.getColor("jPanel16.background")); // NOI18N
        jPanel16.setName("jPanel16"); // NOI18N

        jLabel12.setFont(resourceMap.getFont("jLabel12.font")); // NOI18N
        jLabel12.setForeground(resourceMap.getColor("jLabel12.foreground")); // NOI18N
        jLabel12.setText(resourceMap.getString("jLabel12.text")); // NOI18N
        jLabel12.setName("jLabel12"); // NOI18N

        jButton7.setText(resourceMap.getString("jButton7.text")); // NOI18N
        jButton7.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.RAISED));
        jButton7.setName("jButton7"); // NOI18N
        jButton7.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButton7ActionPerformed(evt);
            }
        });

        org.jdesktop.layout.GroupLayout jPanel16Layout = new org.jdesktop.layout.GroupLayout(jPanel16);
        jPanel16.setLayout(jPanel16Layout);
        jPanel16Layout.setHorizontalGroup(
            jPanel16Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel16Layout.createSequentialGroup()
                .addContainerGap()
                .add(jLabel12)
                .add(18, 18, 18)
                .add(jButton7, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, 232, Short.MAX_VALUE)
                .addContainerGap())
        );
        jPanel16Layout.setVerticalGroup(
            jPanel16Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel16Layout.createSequentialGroup()
                .addContainerGap()
                .add(jPanel16Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.BASELINE)
                    .add(jLabel12)
                    .add(jButton7, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 26, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE))
                .addContainerGap(44, Short.MAX_VALUE))
        );

        org.jdesktop.layout.GroupLayout jPanel12Layout = new org.jdesktop.layout.GroupLayout(jPanel12);
        jPanel12.setLayout(jPanel12Layout);
        jPanel12Layout.setHorizontalGroup(
            jPanel12Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel12Layout.createSequentialGroup()
                .addContainerGap(org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .add(jPanel12Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.TRAILING, false)
                    .add(org.jdesktop.layout.GroupLayout.LEADING, jPanel14, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .add(org.jdesktop.layout.GroupLayout.LEADING, jPanel16, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .add(org.jdesktop.layout.GroupLayout.LEADING, jPanel15, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .add(org.jdesktop.layout.GroupLayout.LEADING, jPanel13, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)))
        );
        jPanel12Layout.setVerticalGroup(
            jPanel12Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel12Layout.createSequentialGroup()
                .addContainerGap()
                .add(jPanel13, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(jPanel15, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(jPanel16, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(jPanel14, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(18, Short.MAX_VALUE))
        );

        org.jdesktop.layout.GroupLayout jPanel2Layout = new org.jdesktop.layout.GroupLayout(jPanel2);
        jPanel2.setLayout(jPanel2Layout);
        jPanel2Layout.setHorizontalGroup(
            jPanel2Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(jPanel2Layout.createSequentialGroup()
                .add(jPanel2Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
                    .add(jPanel3, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .add(jPanel12, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE))
                .addContainerGap())
        );
        jPanel2Layout.setVerticalGroup(
            jPanel2Layout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(org.jdesktop.layout.GroupLayout.TRAILING, jPanel2Layout.createSequentialGroup()
                .add(jPanel3, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(jPanel12, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .add(12, 12, 12))
        );

        org.jdesktop.layout.GroupLayout mainPanelLayout = new org.jdesktop.layout.GroupLayout(mainPanel);
        mainPanel.setLayout(mainPanelLayout);
        mainPanelLayout.setHorizontalGroup(
            mainPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(mainPanelLayout.createSequentialGroup()
                .add(11, 11, 11)
                .add(jPanel2, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, 359, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(org.jdesktop.layout.LayoutStyle.RELATED)
                .add(jPanel4, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                .addContainerGap(102, Short.MAX_VALUE))
        );
        mainPanelLayout.setVerticalGroup(
            mainPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
            .add(mainPanelLayout.createSequentialGroup()
                .add(mainPanelLayout.createParallelGroup(org.jdesktop.layout.GroupLayout.LEADING)
                    .add(jPanel2, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)
                    .add(mainPanelLayout.createSequentialGroup()
                        .add(23, 23, 23)
                        .add(jPanel4, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE, org.jdesktop.layout.GroupLayout.DEFAULT_SIZE, org.jdesktop.layout.GroupLayout.PREFERRED_SIZE)))
                .addContainerGap(377, Short.MAX_VALUE))
        );

        setComponent(mainPanel);
    }// </editor-fold>//GEN-END:initComponents
    /* Diana's objects */
    CreateScenario csWin = new CreateScenario(this);    //give this instance of MainWindow to CreateScenario object
    JPopupMenu dsPop;
    CreateVerb cvWin;
    BuildVerb bvWin;
    TrainVerb tvWin;
    Dimension winSize = new Dimension(500, 600);

    private void ChooseScenarioButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_ChooseScenarioButtonActionPerformed
        int choice = ChooseScenarioButton.getSelectedIndex();
        if (choice == 0) {/*do nothing*/

        } else if (choice == 1) {      //create scenario
            csWin.setSize(winSize);
            csWin.setVisible(true);
        } else if (choice == 2) {      //select a scenario for deletion
            dsPop = new JPopupMenu();  //create popup with scenario choices
            JLabel dsLabel = new JLabel("Choose a scenario to delete.");
            dsLabel.setOpaque(true);
            dsPop.add(dsLabel);
            dsPop.setPopupSize(250, 300);
            dsPop.setLocation(250, 100);
            dsPop.setVisible(true);
            for (int i = 3; i < ChooseScenarioButton.getItemCount(); i++) {
                JMenuItem it = new JMenuItem(ChooseScenarioButton.getItemAt(i).toString());
                it.addActionListener(new ActionListener() {    //if scenario is selected
                    public void actionPerformed(ActionEvent e) {
                        //Execute when button is pressed
                        ScenarioBank sb = new ScenarioBank();
                        sb.deleteScene("crazy scene");
                        System.out.println("I listened");
                    }
                });
                dsPop.add(it);
                i++;
            }

        } else {
            String scenarioName = ChooseScenarioButton.getSelectedItem().toString();  //use existing scenario
            System.out.println("You have selected: " + scenarioName);
            File scenarioFile = new File("__wubble_scenarios__", scenarioName);
            File[] objectArray = scenarioFile.listFiles();
            // String[][] scenarioItems = new String[objectArray.length][2];
            //System.out.println("You have " + objectArray.length + " items.");
            ArrayList<String> scenarioItems = new ArrayList<String>();
            try {
                InitializeEnvironment.Request req = new InitializeEnvironment.Request();    //initialize environment
                req.object_states = new ArrayList<MDPObjectState>();
                for (int i = 0; i < objectArray.length; i++) {
                    try {
                        System.out.println("And the items include: " + objectArray[i].getName());
                        File objectFile = objectArray[i];
                        FileChannel wChannel = new FileInputStream(objectFile).getChannel();
                        // Write the ByteBuffer contents; the bytes between the ByteBuffer's
                        // position and the limit is written to the file
                        // System.out.println(wChannel.size());
                        ByteBuffer bb = ByteBuffer.allocate((int) wChannel.size());
                        wChannel.read(bb);
                        // Close the file
                        wChannel.close();
                        bb.position(0);

                        MDPObjectState myObject = new MDPObjectState();
                        myObject.deserialize(bb);
                        scenarioItems.add(myObject.class_name);
                        scenarioItems.add(myObject.name);
                        //scenarioItems[i][0] = myObject.name;
                        //scenarioItems[i][1] = objectFile.getName();
                        System.out.println("The name of your stored item is: " + myObject.name);
                        System.out.println("It is in the file called: " + objectFile.getName());
                        System.out.println("... whose parent is: " + objectFile.getParent());
                        req.object_states.add(myObject);

                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                    setParams(scenarioItems);
                }
                ServiceClient<InitializeEnvironment.Request, InitializeEnvironment.Response, InitializeEnvironment> sc = WubbleTeach.nh.serviceClient("verb_learning/initialize_environment", new InitializeEnvironment());

//                ServiceClient

                try {
                    sc.call(req);
                } catch (RosException ex) {
                    Logger.getLogger(WubbleTeach.class.getName()).log(Level.SEVERE, null, ex);
                }
                setScenarioName(scenarioName);  //make name visible in View panel
                makeChooseVerb();              //enable Choose Verb combo box
            } catch (NullPointerException E) {
                /* ScenarioBank is already printing out an error message */
            }
        }
    }//GEN-LAST:event_ChooseScenarioButtonActionPerformed

    private void ChooseVerbButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_ChooseVerbButtonActionPerformed
        cvWin = new CreateVerb(this);
        int choice = ChooseVerbButton.getSelectedIndex();
        if (choice == 0) {/*do nothing*/

        } else if (choice == 1) {      //create verb
            cvWin.setSize(winSize);
            cvWin.setVisible(true);
        } else {
            String verbName = ChooseVerbButton.getSelectedItem().toString();  //use existing verb
            System.out.println("You have selected: " + verbName);
            File verbFile = new File("__wubble_verbs__", verbName);
            File[] objectArray = verbFile.listFiles();
            ArrayList<String> verbParams = new ArrayList<String>();
            try {
                for (int i = 0; i < objectArray.length; i++) {
                    try {
                        System.out.println("And they include: " + objectArray[i].getName());
                        File objectFile = objectArray[i];
                        FileChannel wChannel = new FileInputStream(objectFile).getChannel();
                        // Write the ByteBuffer contents; the bytes between the ByteBuffer's
                        // position and the limit is written to the file
                        // System.out.println(wChannel.size());
                        ByteBuffer bb = ByteBuffer.allocate((int) wChannel.size());
                        wChannel.read(bb);
                        // Close the file
                        wChannel.close();
                        bb.position(0);

                        verbParams.add(bb.toString());
                        System.out.println("The name of your stored verb parameter is: " + bb.toString());
                        System.out.println("It is in the file called: " + objectFile.getName());
                        System.out.println("... whose parent is: " + objectFile.getParent());

                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
                setVerbName(verbName);  //make name visible in View panel
                makeBuildVerb();        //enable Build Verb button
            } catch (NullPointerException E) {
            }
        }
    }//GEN-LAST:event_ChooseVerbButtonActionPerformed

    private void jButton7ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButton7ActionPerformed
//        bvWin.show();        // TODO add your handling code here:
        bvWin = new BuildVerb(this);
        bvWin.setSize(winSize);
        bvWin.setVisible(true);
    }//GEN-LAST:event_jButton7ActionPerformed

    private void jButton5ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButton5ActionPerformed
        tvWin = new TrainVerb(this);
        tvWin.setSize(winSize);
        tvWin.setVisible(true);        // TODO add your handling code here:
    }//GEN-LAST:event_jButton5ActionPerformed

    private void jTextField1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jTextField1ActionPerformed
        // TODO add your handling code here:
    }//GEN-LAST:event_jTextField1ActionPerformed

    private void jButton15ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButton15ActionPerformed
        clearParamTable();
    }

    private void clearParamTable() {
        System.out.println("You called clearParamTable.");
        jTextField1.setForeground(Color.gray);      //clear selection of scenario and its components
        jTextField1.setText("scenario");
        jTable1.setModel(new javax.swing.table.DefaultTableModel(
                new Object[][]{
                    {null, null},
                    {null, null},
                    {null, null},
                    {null, null}
                },
                new String[]{
                    "parameter", "item"
                }));
    }//GEN-LAST:event_jButton15ActionPerformed

    private void jButton16ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButton16ActionPerformed
        jTextField2.setForeground(Color.gray);
        jTextField2.setText("verb");
    }//GEN-LAST:event_jButton16ActionPerformed
    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JComboBox ChooseScenarioButton;
    private javax.swing.JComboBox ChooseVerbButton;
    private javax.swing.JButton jButton1;
    private javax.swing.JButton jButton15;
    private javax.swing.JButton jButton16;
    private javax.swing.JButton jButton5;
    private javax.swing.JButton jButton7;
    private javax.swing.JLabel jLabel10;
    private javax.swing.JLabel jLabel11;
    private javax.swing.JLabel jLabel12;
    private javax.swing.JLabel jLabel2;
    private javax.swing.JLabel jLabel3;
    private javax.swing.JLabel jLabel4;
    private javax.swing.JLabel jLabel9;
    private javax.swing.JPanel jPanel12;
    private javax.swing.JPanel jPanel13;
    private javax.swing.JPanel jPanel14;
    private javax.swing.JPanel jPanel15;
    private javax.swing.JPanel jPanel16;
    private javax.swing.JPanel jPanel2;
    private javax.swing.JPanel jPanel3;
    private javax.swing.JPanel jPanel4;
    private javax.swing.JPanel jPanel5;
    private javax.swing.JScrollPane jScrollPane1;
    private javax.swing.JTable jTable1;
    private javax.swing.JTextField jTextField1;
    private javax.swing.JTextField jTextField2;
    private javax.swing.JPanel mainPanel;
    // End of variables declaration//GEN-END:variables
    private final Timer messageTimer;
    private final Timer busyIconTimer;
    private final Icon idleIcon;
    private final Icon[] busyIcons = new Icon[15];
    private int busyIconIndex = 0;
    private JDialog aboutBox;
}
