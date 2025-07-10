package com.example.catvital.Chart;

import android.content.Context;
import android.graphics.Color;
import android.util.AttributeSet;
import android.widget.Toast;

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

public class TempBarChart extends BarChart {
    private static final List<TempDailyData> dataList = new ArrayList<>();
    private final int TEMP_COLOR = Color.parseColor("#FFA726"); // 橙色

    public TempBarChart(Context context) {
        super(context);
        initChart();
    }

    // 新增构造函数（修复错误的关键）
    public TempBarChart(Context context, AttributeSet attrs) {
        super(context, attrs);
        initChart();
    }

    private void initChart() {
        // 基础配置
        getDescription().setEnabled(false);
        setTouchEnabled(true);
        setDrawGridBackground(false);

        // 图表初始化逻辑
        getDescription().setEnabled(false);
        setTouchEnabled(true);
        setDrawGridBackground(false);

        // X轴配置
        XAxis xAxis = getXAxis();
        xAxis.setPosition(XAxis.XAxisPosition.BOTTOM);
        xAxis.setGranularity(1f);
        xAxis.setValueFormatter(new DateFormatter());

        // Y轴配置
        getAxisLeft().setAxisMinimum(35f);
        getAxisLeft().setAxisMaximum(42f);
        getAxisRight().setEnabled(false);

        // 点击事件
        setOnChartValueSelectedListener(new OnChartValueSelectedListener() {
            @Override
            public void onValueSelected(Entry e, Highlight h) {
                int index = (int) e.getX();
                TempDailyData data = dataList.get(index);
                String msg = data.date + "\n平均: " + data.avgTemp + "°C\n范围: " + data.minTemp + "-" + data.maxTemp + "°C";
                Toast.makeText(getContext(), msg, Toast.LENGTH_SHORT).show();
            }

            @Override
            public void onNothingSelected() {}
        });
    }

    public void loadData(List<TempDailyData> data) {
        dataList.clear();
        dataList.addAll(data);

        List<BarEntry> entries = new ArrayList<>();
        for (int i = 0; i < data.size(); i++) {
            entries.add(new BarEntry(i, data.get(i).avgTemp));
        }

        BarDataSet dataSet = new BarDataSet(entries, "体温监测");
        dataSet.setColor(TEMP_COLOR);
        BarData barData = new BarData(dataSet);
        setData(barData);
        invalidate();
    }

    private static class DateFormatter extends ValueFormatter {
        @Override
        public String getFormattedValue(float value) {
            int index = (int) value;
            if (index >= 0 && index < dataList.size()) {
                // 直接显示完整日期（格式：MM/dd）
                return dataList.get(index).date;
            }
            return "";
        }
    }

    public static class TempDailyData {
        public final String date;
        public final float avgTemp;
        public final float minTemp;
        public final float maxTemp;

        public TempDailyData(String date, float avg, float min, float max) {
            this.date = date;
            this.avgTemp = avg;
            this.minTemp = min;
            this.maxTemp = max;
        }
    }
}