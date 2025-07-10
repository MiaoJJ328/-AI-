// ExcretionBarChart.java
package com.example.catvital.Chart;

import android.content.Context;
import android.graphics.Color;
import android.util.AttributeSet;

import com.github.mikephil.charting.charts.BarChart;
import com.github.mikephil.charting.components.XAxis;
import com.github.mikephil.charting.data.BarData;
import com.github.mikephil.charting.data.BarDataSet;
import com.github.mikephil.charting.data.BarEntry;
import com.github.mikephil.charting.formatter.ValueFormatter;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;
import java.util.Locale;

public class ExcretionBarChart extends BarChart {
    private final List<String> dateLabels = new ArrayList<>();
    private static final int EXCRETION_COLOR = Color.parseColor("#9C27B0");
    private static final int BACKGROUND_COLOR = Color.WHITE;

    public ExcretionBarChart(Context context) {
        super(context);
        initChart();
    }

    public ExcretionBarChart(Context context, AttributeSet attrs) {
        super(context, attrs);
        initChart();
    }

    private void initChart() {
        // 基础样式配置
        getDescription().setEnabled(false);
        setDrawGridBackground(false);
        setTouchEnabled(true);
        setPinchZoom(false);
        setBackgroundColor(BACKGROUND_COLOR);

        // X轴配置
        XAxis xAxis = getXAxis();
        xAxis.setPosition(XAxis.XAxisPosition.BOTTOM);
        xAxis.setTextColor(Color.BLACK);
        xAxis.setGranularity(1f);
        xAxis.setValueFormatter(new DateAxisFormatter());
        xAxis.setLabelCount(7);

        // Y轴配置
        getAxisRight().setEnabled(false);
        getAxisLeft().setTextColor(Color.BLACK);
        getAxisLeft().setAxisMinimum(0f);
    }

    public void loadLast7DaysData(List<Integer> values) {
        generateDateLabels();

        List<BarEntry> entries = new ArrayList<>();
        for (int i = 0; i < values.size(); i++) {
            entries.add(new BarEntry(i, values.get(i)));
        }

        BarDataSet dataSet = new BarDataSet(entries, "排泄次数");
        dataSet.setColor(EXCRETION_COLOR);
        dataSet.setValueTextColor(Color.DKGRAY);

        BarData barData = new BarData(dataSet);
        barData.setBarWidth(0.5f);
        setData(barData);
        invalidate();
    }

    private void generateDateLabels() {
        dateLabels.clear();
        SimpleDateFormat sdf = new SimpleDateFormat("MM/dd", Locale.getDefault());
        Calendar calendar = Calendar.getInstance();

        for (int i = 6; i >= 0; i--) {
            calendar.add(Calendar.DAY_OF_YEAR, -i);
            dateLabels.add(sdf.format(calendar.getTime()));
            calendar.add(Calendar.DAY_OF_YEAR, i);
        }
    }

    private class DateAxisFormatter extends ValueFormatter {
        @Override
        public String getFormattedValue(float value) {
            int index = (int) value;
            return (index >= 0 && index < dateLabels.size()) ? dateLabels.get(index) : "";
        }
    }
}