package com.ma5951.utils;

import java.util.HashMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class MAShuffleboard {
    private ShuffleboardTab board;
    private HashMap<String, GenericEntry> values;

    public MAShuffleboard(String tab) {
        board = Shuffleboard.getTab(tab);
        values = new HashMap<String, GenericEntry>();
    }

    public void addNum(String title, double num) {
        if (!values.containsKey(title)) {
            values.put(title, board.add(title, num).getEntry());
        } else {
            values.get(title).setDouble(num);
        }
    }

    public void addString(String title, String str) {
        if(!values.containsKey(title)) {
            values.put(title, board.add(title, str).getEntry());
        } else {
            values.get(title).setString(str);
        }
    }

    public void addBoolean(String title, boolean bol) {
        if (!values.containsKey(title)) {
            values.put(title, board.add(title, bol).getEntry());
        } else {
            values.get(title).setBoolean(bol);
        }
    }
    
    public double getNum(String title) {
        if (values.containsKey(title)) {
            return values.get(title).getDouble(0);
        }
        System.err.println("none existing title: " + title);
        return 0;
    }

    public String getString(String title) {

        if (values.containsKey(title)) {
            return values.get(title).getString("null");
        }
        System.err.println("none existing title: " + title);
        return "null";
    }

    public Boolean getBoolean(String title) {

        if (values.containsKey(title)) {
            return values.get(title).getBoolean(false);
        }
        System.err.println("none existing title: " + title);
        return false;
    }
}