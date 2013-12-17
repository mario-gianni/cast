/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * WizardFrame.java
 *
 * Created on Dec 2, 2011, 2:23:34 PM
 */
package eu.nifti.wizard;

import eu.nifti.dialogue.actions.MoveInDirectionAction.Direction;

/**
 *
 * @author janicek
 */
public class NiceDialogueWizardClient extends DialogueWizardClient {

	/** Creates new form WizardFrame */
	public NiceDialogueWizardClient() {
		initComponents();
	}

	
	/** This method is called from within the constructor to
	 * initialize the form.
	 * WARNING: Do NOT modify this code. The content of this method is
	 * always regenerated by the Form Editor.
	 */
	@SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        autonomyPanel = new javax.swing.JPanel();
        traverseGapButton = new javax.swing.JButton();
        returnToBaseButton = new javax.swing.JButton();
        askGapTraversable = new javax.swing.JButton();
        motionPanel = new javax.swing.JPanel();
        moveForwardButton = new javax.swing.JButton();
        moveLeftButton = new javax.swing.JButton();
        moveRightButton = new javax.swing.JButton();
        moveBackwardButton = new javax.swing.JButton();
        turningPanel = new javax.swing.JPanel();
        turnLeftButton = new javax.swing.JButton();
        turnRightButton = new javax.swing.JButton();
        communicationPanel = new javax.swing.JPanel();
        okButton = new javax.swing.JButton();
        sorryDidNotUnderstandButton = new javax.swing.JButton();
        showMeOnTheMapButton = new javax.swing.JButton();
        repeatLastUtteranceButton = new javax.swing.JButton();
        useTheGUIButton = new javax.swing.JButton();
        sorryCannotDoButton = new javax.swing.JButton();
        stopPanel = new javax.swing.JPanel();
        stopButton = new javax.swing.JButton();
        goingToPlacesPanel = new javax.swing.JPanel();
        goToSelectionButton = new javax.swing.JButton();
        goToLastReferentButton = new javax.swing.JButton();
        lastRobotSpokenOutputItemLabel = new javax.swing.JLabel();

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
        setResizable(false);

        autonomyPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(null, "Autonomy", javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION, javax.swing.border.TitledBorder.DEFAULT_POSITION, new java.awt.Font("Ubuntu", 0, 12))); // NOI18N

        traverseGapButton.setText("Traverse Gap");
        traverseGapButton.setMaximumSize(new java.awt.Dimension(90, 30));
        traverseGapButton.setMinimumSize(new java.awt.Dimension(90, 30));
        traverseGapButton.setOpaque(true);
        traverseGapButton.setPreferredSize(new java.awt.Dimension(90, 30));
        traverseGapButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                traverseGapButtonActionPerformed(evt);
            }
        });

        returnToBaseButton.setText("Return To Base");
        returnToBaseButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                returnToBaseButtonActionPerformed(evt);
            }
        });

        askGapTraversable.setText("Is Gap Traversable?");
        askGapTraversable.setMaximumSize(new java.awt.Dimension(90, 30));
        askGapTraversable.setMinimumSize(new java.awt.Dimension(90, 30));
        askGapTraversable.setOpaque(true);
        askGapTraversable.setPreferredSize(new java.awt.Dimension(90, 30));
        askGapTraversable.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                askGapTraversableActionPerformed(evt);
            }
        });

        javax.swing.GroupLayout autonomyPanelLayout = new javax.swing.GroupLayout(autonomyPanel);
        autonomyPanel.setLayout(autonomyPanelLayout);
        autonomyPanelLayout.setHorizontalGroup(
            autonomyPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(autonomyPanelLayout.createSequentialGroup()
                .addContainerGap()
                .addGroup(autonomyPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(askGapTraversable, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, 176, Short.MAX_VALUE)
                    .addComponent(returnToBaseButton, javax.swing.GroupLayout.DEFAULT_SIZE, 176, Short.MAX_VALUE)
                    .addComponent(traverseGapButton, javax.swing.GroupLayout.DEFAULT_SIZE, 176, Short.MAX_VALUE))
                .addContainerGap())
        );
        autonomyPanelLayout.setVerticalGroup(
            autonomyPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(autonomyPanelLayout.createSequentialGroup()
                .addContainerGap()
                .addComponent(askGapTraversable, javax.swing.GroupLayout.PREFERRED_SIZE, 63, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addComponent(traverseGapButton, javax.swing.GroupLayout.PREFERRED_SIZE, 66, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, 50, Short.MAX_VALUE)
                .addComponent(returnToBaseButton, javax.swing.GroupLayout.PREFERRED_SIZE, 65, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addContainerGap())
        );

        motionPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(null, "Motion", javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION, javax.swing.border.TitledBorder.DEFAULT_POSITION, new java.awt.Font("Ubuntu", 0, 12))); // NOI18N

        moveForwardButton.setText("FORWARD");
        moveForwardButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                moveForwardButtonActionPerformed(evt);
            }
        });

        moveLeftButton.setText("LEFT");
        moveLeftButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                moveLeftButtonActionPerformed(evt);
            }
        });

        moveRightButton.setText("RIGHT");
        moveRightButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                moveRightButtonActionPerformed(evt);
            }
        });

        moveBackwardButton.setText("BACKWARD");
        moveBackwardButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                moveBackwardButtonActionPerformed(evt);
            }
        });

        javax.swing.GroupLayout motionPanelLayout = new javax.swing.GroupLayout(motionPanel);
        motionPanel.setLayout(motionPanelLayout);
        motionPanelLayout.setHorizontalGroup(
            motionPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(motionPanelLayout.createSequentialGroup()
                .addGroup(motionPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, motionPanelLayout.createSequentialGroup()
                        .addContainerGap()
                        .addComponent(moveLeftButton, javax.swing.GroupLayout.PREFERRED_SIZE, 77, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(moveBackwardButton, javax.swing.GroupLayout.PREFERRED_SIZE, 128, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .addComponent(moveRightButton, javax.swing.GroupLayout.PREFERRED_SIZE, 79, javax.swing.GroupLayout.PREFERRED_SIZE))
                    .addGroup(motionPanelLayout.createSequentialGroup()
                        .addGap(104, 104, 104)
                        .addComponent(moveForwardButton, javax.swing.GroupLayout.PREFERRED_SIZE, 107, javax.swing.GroupLayout.PREFERRED_SIZE)))
                .addContainerGap())
        );
        motionPanelLayout.setVerticalGroup(
            motionPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(motionPanelLayout.createSequentialGroup()
                .addGap(6, 6, 6)
                .addComponent(moveForwardButton, javax.swing.GroupLayout.PREFERRED_SIZE, 58, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addGroup(motionPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                    .addGroup(motionPanelLayout.createSequentialGroup()
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                        .addGroup(motionPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(moveLeftButton, javax.swing.GroupLayout.PREFERRED_SIZE, 70, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(moveRightButton, javax.swing.GroupLayout.PREFERRED_SIZE, 74, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addContainerGap())
                    .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, motionPanelLayout.createSequentialGroup()
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .addComponent(moveBackwardButton, javax.swing.GroupLayout.PREFERRED_SIZE, 59, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(78, 78, 78))))
        );

        turningPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(null, "Turning", javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION, javax.swing.border.TitledBorder.DEFAULT_POSITION, new java.awt.Font("Ubuntu", 0, 12))); // NOI18N

        turnLeftButton.setText("LEFT");
        turnLeftButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                turnLeftButtonActionPerformed(evt);
            }
        });

        turnRightButton.setText("RIGHT");
        turnRightButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                turnRightButtonActionPerformed(evt);
            }
        });

        javax.swing.GroupLayout turningPanelLayout = new javax.swing.GroupLayout(turningPanel);
        turningPanel.setLayout(turningPanelLayout);
        turningPanelLayout.setHorizontalGroup(
            turningPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(turningPanelLayout.createSequentialGroup()
                .addContainerGap()
                .addComponent(turnLeftButton, javax.swing.GroupLayout.PREFERRED_SIZE, 100, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, 70, Short.MAX_VALUE)
                .addComponent(turnRightButton, javax.swing.GroupLayout.PREFERRED_SIZE, 95, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addContainerGap())
        );
        turningPanelLayout.setVerticalGroup(
            turningPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, turningPanelLayout.createSequentialGroup()
                .addContainerGap()
                .addGroup(turningPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addComponent(turnRightButton, javax.swing.GroupLayout.Alignment.LEADING, javax.swing.GroupLayout.DEFAULT_SIZE, 110, Short.MAX_VALUE)
                    .addComponent(turnLeftButton, javax.swing.GroupLayout.Alignment.LEADING, javax.swing.GroupLayout.DEFAULT_SIZE, 110, Short.MAX_VALUE))
                .addContainerGap())
        );

        communicationPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(null, "Communication", javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION, javax.swing.border.TitledBorder.DEFAULT_POSITION, new java.awt.Font("Ubuntu", 0, 12))); // NOI18N

        okButton.setText("OK");
        okButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                okButtonActionPerformed(evt);
            }
        });

        sorryDidNotUnderstandButton.setText("Sorry, Didn't Understand");
        sorryDidNotUnderstandButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                sorryDidNotUnderstandButtonActionPerformed(evt);
            }
        });

        showMeOnTheMapButton.setText("Show Me On The Map");
        showMeOnTheMapButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                showMeOnTheMapButtonActionPerformed(evt);
            }
        });

        repeatLastUtteranceButton.setText("Repeat Last Utterance");
        repeatLastUtteranceButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                repeatLastUtteranceButtonActionPerformed(evt);
            }
        });

        useTheGUIButton.setText("Use The GUI For That");
        useTheGUIButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                useTheGUIButtonActionPerformed(evt);
            }
        });

        sorryCannotDoButton.setText("Sorry, Can't Do");
        sorryCannotDoButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                sorryCannotDoButtonActionPerformed(evt);
            }
        });

        javax.swing.GroupLayout communicationPanelLayout = new javax.swing.GroupLayout(communicationPanel);
        communicationPanel.setLayout(communicationPanelLayout);
        communicationPanelLayout.setHorizontalGroup(
            communicationPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(communicationPanelLayout.createSequentialGroup()
                .addContainerGap()
                .addGroup(communicationPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(okButton, javax.swing.GroupLayout.DEFAULT_SIZE, 234, Short.MAX_VALUE)
                    .addComponent(showMeOnTheMapButton, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, 234, Short.MAX_VALUE)
                    .addComponent(sorryDidNotUnderstandButton, javax.swing.GroupLayout.DEFAULT_SIZE, 234, Short.MAX_VALUE)
                    .addComponent(useTheGUIButton, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, 234, Short.MAX_VALUE)
                    .addComponent(sorryCannotDoButton, javax.swing.GroupLayout.DEFAULT_SIZE, 234, Short.MAX_VALUE)
                    .addComponent(repeatLastUtteranceButton, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, 234, Short.MAX_VALUE))
                .addContainerGap())
        );
        communicationPanelLayout.setVerticalGroup(
            communicationPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(communicationPanelLayout.createSequentialGroup()
                .addContainerGap()
                .addComponent(okButton, javax.swing.GroupLayout.PREFERRED_SIZE, 65, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(sorryCannotDoButton, javax.swing.GroupLayout.PREFERRED_SIZE, 65, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(sorryDidNotUnderstandButton, javax.swing.GroupLayout.PREFERRED_SIZE, 67, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addComponent(showMeOnTheMapButton, javax.swing.GroupLayout.PREFERRED_SIZE, 59, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(useTheGUIButton, javax.swing.GroupLayout.PREFERRED_SIZE, 67, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, 41, Short.MAX_VALUE)
                .addComponent(repeatLastUtteranceButton, javax.swing.GroupLayout.PREFERRED_SIZE, 74, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addContainerGap())
        );

        stopPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(null, "Stop", javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION, javax.swing.border.TitledBorder.DEFAULT_POSITION, new java.awt.Font("Ubuntu", 0, 12))); // NOI18N

        stopButton.setFont(new java.awt.Font("Ubuntu", 0, 36)); // NOI18N
        stopButton.setText("STOP");
        stopButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                stopButtonActionPerformed(evt);
            }
        });

        javax.swing.GroupLayout stopPanelLayout = new javax.swing.GroupLayout(stopPanel);
        stopPanel.setLayout(stopPanelLayout);
        stopPanelLayout.setHorizontalGroup(
            stopPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(stopPanelLayout.createSequentialGroup()
                .addContainerGap()
                .addComponent(stopButton, javax.swing.GroupLayout.DEFAULT_SIZE, 1103, Short.MAX_VALUE)
                .addContainerGap())
        );
        stopPanelLayout.setVerticalGroup(
            stopPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(stopPanelLayout.createSequentialGroup()
                .addContainerGap()
                .addComponent(stopButton, javax.swing.GroupLayout.DEFAULT_SIZE, 70, Short.MAX_VALUE)
                .addContainerGap())
        );

        goingToPlacesPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(null, "Go To Place", javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION, javax.swing.border.TitledBorder.DEFAULT_POSITION, new java.awt.Font("Ubuntu", 0, 12))); // NOI18N

        goToSelectionButton.setText("Go To Selection");
        goToSelectionButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                goToSelectionButtonActionPerformed(evt);
            }
        });

        goToLastReferentButton.setText("Go To Last Referent");
        goToLastReferentButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                goToLastReferentButtonActionPerformed(evt);
            }
        });

        javax.swing.GroupLayout goingToPlacesPanelLayout = new javax.swing.GroupLayout(goingToPlacesPanel);
        goingToPlacesPanel.setLayout(goingToPlacesPanelLayout);
        goingToPlacesPanelLayout.setHorizontalGroup(
            goingToPlacesPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(goingToPlacesPanelLayout.createSequentialGroup()
                .addContainerGap()
                .addGroup(goingToPlacesPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addComponent(goToSelectionButton, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, 161, Short.MAX_VALUE)
                    .addComponent(goToLastReferentButton, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, 161, Short.MAX_VALUE))
                .addContainerGap())
        );
        goingToPlacesPanelLayout.setVerticalGroup(
            goingToPlacesPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, goingToPlacesPanelLayout.createSequentialGroup()
                .addComponent(goToSelectionButton, javax.swing.GroupLayout.DEFAULT_SIZE, 65, Short.MAX_VALUE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(goToLastReferentButton, javax.swing.GroupLayout.PREFERRED_SIZE, 64, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addContainerGap())
        );

        lastRobotSpokenOutputItemLabel.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        lastRobotSpokenOutputItemLabel.setText("[nothing]");

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
        getContentPane().setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(layout.createSequentialGroup()
                        .addComponent(autonomyPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(26, 26, 26)
                        .addComponent(goingToPlacesPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(18, 18, 18)
                        .addComponent(communicationPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, 46, Short.MAX_VALUE)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(motionPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(turningPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addGap(50, 50, 50))
                    .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, layout.createSequentialGroup()
                        .addComponent(lastRobotSpokenOutputItemLabel)
                        .addGap(541, 541, 541))
                    .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, layout.createSequentialGroup()
                        .addComponent(stopPanel, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .addContainerGap())))
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(layout.createSequentialGroup()
                        .addGap(46, 46, 46)
                        .addComponent(motionPanel, javax.swing.GroupLayout.PREFERRED_SIZE, 189, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(49, 49, 49)
                        .addComponent(turningPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                    .addGroup(layout.createSequentialGroup()
                        .addGap(23, 23, 23)
                        .addComponent(goingToPlacesPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                    .addGroup(layout.createSequentialGroup()
                        .addGap(56, 56, 56)
                        .addComponent(autonomyPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                    .addGroup(layout.createSequentialGroup()
                        .addContainerGap()
                        .addComponent(communicationPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(lastRobotSpokenOutputItemLabel)))
                .addGap(18, 18, 18)
                .addComponent(stopPanel, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addContainerGap())
        );

        pack();
    }// </editor-fold>//GEN-END:initComponents

	private void moveLeftButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_moveLeftButtonActionPerformed
		getListener().onGoInDirection(Direction.LEFT);
	}//GEN-LAST:event_moveLeftButtonActionPerformed

	private void traverseGapButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_traverseGapButtonActionPerformed
		getListener().onTraverseGap();
	}//GEN-LAST:event_traverseGapButtonActionPerformed

	private void returnToBaseButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_returnToBaseButtonActionPerformed
		getListener().onReturnToBase();
	}//GEN-LAST:event_returnToBaseButtonActionPerformed

	private void goToSelectionButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_goToSelectionButtonActionPerformed
		getListener().onGoToSelection();
	}//GEN-LAST:event_goToSelectionButtonActionPerformed

	private void goToLastReferentButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_goToLastReferentButtonActionPerformed
		getListener().onGoToLastMention();
	}//GEN-LAST:event_goToLastReferentButtonActionPerformed

	private void okButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_okButtonActionPerformed
		getListener().onSayOk();
	}//GEN-LAST:event_okButtonActionPerformed

	private void sorryDidNotUnderstandButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_sorryDidNotUnderstandButtonActionPerformed
		getListener().onSorryDidNotUnderstand();
	}//GEN-LAST:event_sorryDidNotUnderstandButtonActionPerformed

	private void showMeOnTheMapButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_showMeOnTheMapButtonActionPerformed
		getListener().onShowMeOnTheMap();
	}//GEN-LAST:event_showMeOnTheMapButtonActionPerformed

	private void repeatLastUtteranceButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_repeatLastUtteranceButtonActionPerformed
		getListener().onRepeatLastUtterance();
	}//GEN-LAST:event_repeatLastUtteranceButtonActionPerformed

	private void stopButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_stopButtonActionPerformed
		getListener().onStop();
	}//GEN-LAST:event_stopButtonActionPerformed

	private void moveForwardButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_moveForwardButtonActionPerformed
		getListener().onGoInDirection(Direction.FORWARD);
	}//GEN-LAST:event_moveForwardButtonActionPerformed

	private void moveBackwardButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_moveBackwardButtonActionPerformed
		getListener().onGoInDirection(Direction.BACKWARD);
	}//GEN-LAST:event_moveBackwardButtonActionPerformed

	private void moveRightButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_moveRightButtonActionPerformed
		getListener().onGoInDirection(Direction.RIGHT);
	}//GEN-LAST:event_moveRightButtonActionPerformed

	private void turnLeftButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_turnLeftButtonActionPerformed
		getListener().onTurnInDirection(eu.nifti.dialogue.actions.TurnInDirectionAction.Direction.LEFT);
	}//GEN-LAST:event_turnLeftButtonActionPerformed

	private void turnRightButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_turnRightButtonActionPerformed
		getListener().onTurnInDirection(eu.nifti.dialogue.actions.TurnInDirectionAction.Direction.RIGHT);
	}//GEN-LAST:event_turnRightButtonActionPerformed

	private void useTheGUIButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_useTheGUIButtonActionPerformed
		getListener().onPleaseUseGUIToDoThat();
	}//GEN-LAST:event_useTheGUIButtonActionPerformed

	private void askGapTraversableActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_askGapTraversableActionPerformed
		getListener().onAskGapTraversable();
	}//GEN-LAST:event_askGapTraversableActionPerformed

	private void sorryCannotDoButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_sorryCannotDoButtonActionPerformed
		getListener().onSorryCannotDo();
	}//GEN-LAST:event_sorryCannotDoButtonActionPerformed

	/**
	 * @param args the command line arguments
	 */
	public static void main(String args[]) {
		/* Set the Nimbus look and feel */
		//<editor-fold defaultstate="collapsed" desc=" Look and feel setting code (optional) ">
        /* If Nimbus (introduced in Java SE 6) is not available, stay with the default look and feel.
		 * For details see http://download.oracle.com/javase/tutorial/uiswing/lookandfeel/plaf.html 
		 */
		try {
			for (javax.swing.UIManager.LookAndFeelInfo info : javax.swing.UIManager.getInstalledLookAndFeels()) {
				if ("Nimbus".equals(info.getName())) {
					javax.swing.UIManager.setLookAndFeel(info.getClassName());
					break;
				}
			}
		} catch (ClassNotFoundException ex) {
			java.util.logging.Logger.getLogger(NiceDialogueWizardClient.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
		} catch (InstantiationException ex) {
			java.util.logging.Logger.getLogger(NiceDialogueWizardClient.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
		} catch (IllegalAccessException ex) {
			java.util.logging.Logger.getLogger(NiceDialogueWizardClient.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
		} catch (javax.swing.UnsupportedLookAndFeelException ex) {
			java.util.logging.Logger.getLogger(NiceDialogueWizardClient.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
		}
		//</editor-fold>

		/* Create and display the form */
		java.awt.EventQueue.invokeLater(new Runnable() {

			@Override
			public void run() {
				new NiceDialogueWizardClient().setVisible(true);
			}
		});
	}
    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JButton askGapTraversable;
    private javax.swing.JPanel autonomyPanel;
    private javax.swing.JPanel communicationPanel;
    private javax.swing.JButton goToLastReferentButton;
    private javax.swing.JButton goToSelectionButton;
    private javax.swing.JPanel goingToPlacesPanel;
    private javax.swing.JLabel lastRobotSpokenOutputItemLabel;
    private javax.swing.JPanel motionPanel;
    private javax.swing.JButton moveBackwardButton;
    private javax.swing.JButton moveForwardButton;
    private javax.swing.JButton moveLeftButton;
    private javax.swing.JButton moveRightButton;
    private javax.swing.JButton okButton;
    private javax.swing.JButton repeatLastUtteranceButton;
    private javax.swing.JButton returnToBaseButton;
    private javax.swing.JButton showMeOnTheMapButton;
    private javax.swing.JButton sorryCannotDoButton;
    private javax.swing.JButton sorryDidNotUnderstandButton;
    private javax.swing.JButton stopButton;
    private javax.swing.JPanel stopPanel;
    private javax.swing.JButton traverseGapButton;
    private javax.swing.JButton turnLeftButton;
    private javax.swing.JButton turnRightButton;
    private javax.swing.JPanel turningPanel;
    private javax.swing.JButton useTheGUIButton;
    // End of variables declaration//GEN-END:variables

	@Override
	public void setLastRobotSpokenItem(final String s) {
		java.awt.EventQueue.invokeLater(new Runnable() {

			@Override
			public void run() {
				lastRobotSpokenOutputItemLabel.setText("\"" + s + "\"");
			}
			
		});
	}

}
