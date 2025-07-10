package com.example.catvital.ListView;

public class TimeWeightItem {
    private String time;
    private String weight;
    private boolean isExpanded;

    public TimeWeightItem(String time, String weight) {
        this.time = time;
        this.weight = weight;
        this.isExpanded = false; // 默认折叠
    }

    public TimeWeightItem(String time) {
        this.time = time;
        this.isExpanded = false; // 默认折叠
    }

    public boolean isExpanded() { return isExpanded; }
    public void setExpanded(boolean expanded) { isExpanded = expanded; }
    public String getTime() { return time; }
    public String getWeight() { return weight; }
}
