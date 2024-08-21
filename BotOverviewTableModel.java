package edu.tigers.sumatra.botoverview;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.SortedMap;
import java.util.concurrent.ConcurrentSkipListMap;

import javax.swing.table.AbstractTableModel;

import edu.tigers.sumatra.ai.data.EBotInformation;
import edu.tigers.sumatra.botoverview.view.BotOverviewColumn;
import edu.tigers.sumatra.ids.BotID;
import edu.tigers.sumatra.ids.ETeamColor;

public class BotOverviewTableModel extends AbstractTableModel {
    private static final long serialVersionUID = 4712760313475190867L;
    private final SortedMap<BotID, BotOverviewColumn> data = new ConcurrentSkipListMap<>(BotID.getComparator());
    private BotID[] sortedBots = new BotID[0];

    public BotOverviewColumn putBot(final BotID botId, final BotOverviewColumn columnData) {
        BotOverviewColumn col = data.get(botId);
        if (col == null) {
            data.put(botId, columnData);
            update();
            fireTableStructureChanged();
        } else if (!col.equals(columnData)) {
            data.put(botId, columnData);
            fireTableDataChanged();
        }
        return col;
    }

    public void removeBot(final BotID botId) {
        Object rem = data.remove(botId);
        if (rem != null) {
            update();
            fireTableStructureChanged();
        }
    }

    private void update() {
        sortedBots = data.keySet().toArray(new BotID[0]);
    }

    public List<BotID> getBots() {
        return Arrays.asList(sortedBots);
    }

    @Override
    public int getColumnCount() {
        return data.size() + 1;
    }

    @Override
    public int getRowCount() {
        return EBotInformation.values().length;
    }

    @Override
    public String getColumnName(final int col) {
        if (col == 0) {
            return "";
        }
        return "Bot " + sortedBots[col - 1].getNumber() + " "
                + (sortedBots[col - 1].getTeamColor() == ETeamColor.YELLOW ? "Y" : "B");
    }

    @Override
    public Object getValueAt(final int row, final int col) {
        if (col == 0) {
            return EBotInformation.values()[row].getLabel();
        }
        return data.get(sortedBots[col - 1]).getData().get(row);
    }

    @Override
    public Class<?> getColumnClass(final int c) {
        return getValueAt(0, c).getClass();
    }

    public void saveRobotsData() {
        SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_HHmmss");
        String timestamp = sdf.format(new Date());

        File mainDir = new File("robot_data");
        if (!mainDir.exists()) {
            mainDir.mkdirs();
        }

        for (BotID botId : data.keySet()) {
            String teamDir = "robot_data/" + (botId.getTeamColor() == ETeamColor.YELLOW ? "YellowTeam" : "BlueTeam");
            File teamFolder = new File(teamDir);
            if (!teamFolder.exists()) {
                teamFolder.mkdirs();
            }

            String robotDir = teamDir + "/Bot_" + botId.getNumber();
            File dir = new File(robotDir);
            if (!dir.exists()) {
                dir.mkdirs();
            }

            String fileName = "data_" + timestamp + ".txt";
            File file = new File(dir, fileName);

            try (BufferedWriter writer = new BufferedWriter(new FileWriter(file))) {
                BotOverviewColumn column = data.get(botId);
                if (column != null) {
                    writer.write("Bot ID: " + botId + "\n");
                    for (int row = 0; row < getRowCount(); row++) {
                        String label = EBotInformation.values()[row].getLabel();
                        Object value = getValueAt(row, Arrays.asList(sortedBots).indexOf(botId) + 1);
                        writer.write(label + ": " + value + "\n");
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
