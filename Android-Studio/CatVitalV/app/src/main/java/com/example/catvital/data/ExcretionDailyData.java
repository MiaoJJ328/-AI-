package com.example.catvital.data;

public class ExcretionDailyData {
    private String date;
    private int count;

    public ExcretionDailyData(String date, int count) {
        this.date = date;
        this.count = count;
    }

    public String getDate() { return date; }
    public int getCount() { return count; }
}