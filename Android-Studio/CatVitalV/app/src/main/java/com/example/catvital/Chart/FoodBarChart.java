package com.example.catvital.Chart;

import android.content.Context;
import android.graphics.Color;
import android.util.AttributeSet;
import android.widget.Toast;

import com.example.catvital.data.FoodDailyData;
import com.github.mikephil.charting.charts.BarChart;
import com.github.mikephil.charting.components.XAxis;
import com.github.mikephil.charting.data.BarData;
import com.github.mikephil.charting.data.BarDataSet;
import com.github.mikephil.charting.data.BarEntry;
import com.github.mikephil.charting.data.Entry;
import com.github.mikephil.charting.formatter.ValueFormatter;
import com.github.mikephil.charting.highlight.Highlight;
import com.github.mikephil.charting.listener.OnChartValueSelectedListener;

import java.util.ArrayList;
import java.util.List;

public class FoodBarChart extends BarChart {
    private final List<FoodDailyData> dataList = new ArrayList<>();
    private final int FOOD_COLOR = Color.parseColor("#4CAF50"); // 绿色

    // 必须的三个构造函数
    public FoodBarChart(Context context) {
        super(context);
        initChart();
    }

    public FoodBarChart(Context context, AttributeSet attrs) {
        super(context, attrs);
        initChart();
    }

    public FoodBarChart(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        initChart();
    }

    private void initChart() {
        // 基础配置
        getDescription().setEnabled(false);
        setTouchEnabled(true);
        setDrawGridBackground(false);

        // X轴配置
        XAxis xAxis = getXAxis();
        xAxis.setPosition(XAxis.XAxisPosition.BOTTOM);
        xAxis.setGranularity(1f);
        xAxis.setValueFormatter(new DateFormatter());
        xAxis.setLabelCount(7);
        xAxis.setLabelRotationAngle(-45);

        // Y轴配置
        getAxisLeft().setAxisMinimum(0);
        getAxisLeft().setAxisMaximum(30); // 假设最大摄入量为30g
        getAxisRight().setEnabled(false);

        // 点击事件
        setOnChartValueSelectedListener(new OnChartValueSelectedListener() {
            @Override
            public void onValueSelected(Entry e, Highlight h) {
                int index = (int) e.getX();
                FoodDailyData data = dataList.get(index);
                String msg = data.getDate() + "\n平均: " + data.getAvgIntake() + "g\n范围: " +
                        data.getMinIntake() + "-" + data.getMaxIntake() + "g";
                Toast.makeText(getContext(), msg, Toast.LENGTH_SHORT).show();
            }

            @Override
            public void onNothingSelected() {}
        });
    }

    public void loadData(List<FoodDailyData> data) {
        dataList.clear();
        dataList.addAll(data);

        List<BarEntry> entries = new ArrayList<>();
        for (int i = 0; i < data.size(); i++) {
            entries.add(new BarEntry(i, data.get(i).getAvgIntake()));
        }

        BarDataSet dataSet = new BarDataSet(entries, "饮食监测（g）");
        dataSet.setColor(FOOD_COLOR);
        BarData barData = new BarData(dataSet);
        setData(barData);
        invalidate();
    }

    private class DateFormatter extends ValueFormatter {
        @Override
        public String getFormattedValue(float value) {
            int index = (int) value;
            return (index >= 0 && index < dataList.size()) ?
                    dataList.get(index).getDate() : "";
        }
    }
}