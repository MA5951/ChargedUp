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
        try {
            values.get(title).setDouble(num);
        }
        catch (Exception e) {
            values.put(title, board.add(title, num).getEntry());
        }
    }

    public void addString(String title, String str) {
        try {
            values.get(title).setString(str);
        }
        catch (Exception e) {
            values.put(title, board.add(title, str).getEntry());
        }
    }

    public void addBoolean(String title, boolean bol) {
        try {
            values.get(title).setBoolean(bol);
        }
        catch (Exception e) {
            values.put(title, board.add(title, bol).getEntry());
        }
    }
    
    public double getNum(String title) {
        try {
            return values.get(title).getDouble(0);
        } catch (Exception e){
            System.err.println("none existing title");
        }
        return 0;
    }

    public String getString(String title) {
        try {
            return values.get(title).getString("null");
        } catch (Exception e){
            System.err.println("none existing title");
        }
        return "null";
    }

    public Boolean getBoolean(String title) {
        try {
            return values.get(title).getBoolean(false);
        } catch (Exception e){
            System.err.println("none existing title");
        }
        return false;
    }
}