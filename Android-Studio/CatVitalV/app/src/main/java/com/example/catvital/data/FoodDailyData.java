package com.example.catvital.data;

public class FoodDailyData {
    private final String date;
    private final int avgIntake;
    private final int minIntake;
    private final int maxIntake;

    public FoodDailyData(String date, int avg, int min, int max) {
        this.date = date;
        this.avgIntake = avg;
        this.minIntake = min;
        this.maxIntake = max;
    }

    // Getters
    public String getDate() { return date; }
    public int getAvgIntake() { return avgIntake; }
    public int getMinIntake() { return minIntake; }
    public int getMaxIntake() { return maxIntake; }
}