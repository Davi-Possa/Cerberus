package edu.tigers.sumatra.botoverview.view;

import edu.tigers.sumatra.ai.VisualizationFrame;
import edu.tigers.sumatra.ai.data.BotAiInformation;
import edu.tigers.sumatra.botoverview.BotOverviewTableModel;
import edu.tigers.sumatra.ids.BotID;
import org.jdesktop.swingx.JXTable;

import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.Timer;
import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.Serial;
import java.util.Map;

public class BotOverviewPanel extends JPanel {

    @Serial
    private static final long serialVersionUID = -8536401073164037476L;
    private final BotOverviewTableModel model;
    private final JXTable table;
    private Timer saveTimer;
    private boolean isSaving = false;

    public BotOverviewPanel() {
        setLayout(new BorderLayout());
        model = new BotOverviewTableModel();
        table = new JXTable(model);
        table.setColumnControlVisible(true);
        table.setHorizontalScrollEnabled(true);
        table.setSortable(false);
        table.updateUI();
        JScrollPane scrollPane = new JScrollPane(table);
        add(scrollPane, BorderLayout.CENTER);

        JButton saveButton = new JButton("Save Robot Data");
        saveButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                if (isSaving) {
                    stopSaving();
                    saveButton.setText("Save Robot Data");
                } else {
                    startSaving();
                    saveButton.setText("Stop Saving");
                }
            }
        });
        add(saveButton, BorderLayout.SOUTH);
    }

    private void startSaving() {
        saveTimer = new Timer(5000, new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                model.saveRobotsData();
            }
        });
        saveTimer.start();
        isSaving = true;
    }

    private void stopSaving() {
        if (saveTimer != null) {
            saveTimer.stop();
        }
        isSaving = false;
    }

    public void update(final VisualizationFrame frame) {
        // Remove bots desaparecidos
        for (BotID botId : model.getBots()) {
            if ((botId.getTeamColor() == frame.getTeamColor()) && !frame.getAiInfos().containsKey(botId)) {
                model.removeBot(botId);
            }
        }

        // Adiciona informações de AI para todos os bots conhecidos
        for (Map.Entry<BotID, BotAiInformation> entry : frame.getAiInfos().entrySet()) {
            BotID botId = entry.getKey();
            BotAiInformation aiInfo = entry.getValue();
            if (aiInfo != null) {
                BotOverviewColumn column = new BotOverviewColumn(aiInfo);
                BotOverviewColumn prevColumn = model.putBot(botId, column);
                if (prevColumn == null) {
                    updateColumnSize();
                }
            }
        }
    }

    private void updateColumnSize() {
        table.getColumnModel().getColumn(0).setMinWidth(155);
        for (int i = 1; i < model.getColumnCount(); i++) {
            table.getColumnModel().getColumn(i).setMinWidth(65);
        }
    }
}
