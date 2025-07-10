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

public class HeartRateBarChart extends BarChart {
    private final List<String> dateLabels = new ArrayList<>();
    private static final int HEART_RATE_COLOR = Color.parseColor("#FF5252"); // 直接定义颜色

    public HeartRateBarChart(Context context) {
        super(context);
        initChart();
    }

    public HeartRateBarChart(Context context, AttributeSet attrs) {
        super(context, attrs);
        initChart();
    }

    private void initChart() {
        // 基础样式配置
        getDescription().setEnabled(false);
        setDrawGridBackground(false);
        setTouchEnabled(true);
        setPinchZoom(false);

        // X轴配置
        XAxis xAxis = getXAxis();
        xAxis.setPosition(XAxis.XAxisPosition.BOTTOM);
        xAxis.setGranularity(1f);
        xAxis.setValueFormatter(new DateAxisFormatter());
        xAxis.setLabelCount(7); // 固定显示7个标签

        // Y轴配置
        getAxisRight().setEnabled(false);
        getAxisLeft().setAxisMinimum(0f);
    }

    // 加载最近7天数据
    public void loadLast7DaysData(List<Float> values) {
        // 生成日期标签
        generateDateLabels();

        // 创建数据条目
        List<BarEntry> entries = new ArrayList<>();
        for (int i = 0; i < values.size(); i++) {
            entries.add(new BarEntry(i, values.get(i)));
        }

        // 数据集配置
        BarDataSet dataSet = new BarDataSet(entries, "心率（BPM）");
        dataSet.setColor(HEART_RATE_COLOR);
        dataSet.setValueTextColor(Color.DKGRAY);

        // 绑定数据
        BarData barData = new BarData(dataSet);
        barData.setBarWidth(0.5f);
        setData(barData);
        invalidate();
    }

    private void generateDateLabels() {
        dateLabels.clear();
        SimpleDateFormat sdf = new SimpleDateFormat("MM/dd", Locale.CHINA);
        Calendar calendar = Calendar.getInstance();

        // 生成过去7天日期（含今天）
        for (int i = 6; i >= 0; i--) {
            calendar.add(Calendar.DAY_OF_YEAR, -i);
            dateLabels.add(sdf.format(calendar.getTime()));
            calendar.add(Calendar.DAY_OF_YEAR, i); // 重置
        }
    }

    // 日期格式化器
    private class DateAxisFormatter extends ValueFormatter {
        @Override
        public String getFormattedValue(float value) {
            int index = (int) value;
            return (index >= 0 && index < dateLabels.size()) ? dateLabels.get(index) : "";
        }
    }
}