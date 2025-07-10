package com.example.catvital.Chart;

import android.content.Context;
import android.graphics.Color;
import android.util.AttributeSet;

import androidx.core.content.ContextCompat;

import com.example.catvital.R;
import com.github.mikephil.charting.charts.BarChart;
import com.github.mikephil.charting.components.Description;
import com.github.mikephil.charting.components.Legend;
import com.github.mikephil.charting.components.XAxis;
import com.github.mikephil.charting.components.YAxis;
import com.github.mikephil.charting.data.BarData;
import com.github.mikephil.charting.data.BarDataSet;
import com.github.mikephil.charting.data.BarEntry;
import com.github.mikephil.charting.formatter.ValueFormatter;

import java.sql.Date;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class BarChartView extends BarChart {

    private final List<BarEntry> entries = new ArrayList<>();
    private long startTime = System.currentTimeMillis();
    private String chartTitle = "实时数据监测";
    private String dataLabel = "数值";
    private int barColor = Color.rgb(0, 200, 255); // 青色柱状图
    private int highlightColor = Color.rgb(255, 100, 100);
    private int dataCount = 0; // 用于记录数据点数量，实现等间距
    private SimpleDateFormat timeFormat = new SimpleDateFormat("HH:mm", Locale.getDefault());

    public BarChartView(Context context) {
        super(context);
        init(context);
    }

    public BarChartView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    public BarChartView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        init(context);
    }

    private void init(Context context) {
        post(() -> {
            setupChartStyle();
            setupAxis();

            // 初始化默认数据
            if (entries.isEmpty()) {
                entries.add(new BarEntry(0, 0));
                updateChartData();
            }
        });
    }

    private void setupChartStyle() {
        // 基础配置
        setTouchEnabled(true);
        setDragEnabled(true);
        setScaleEnabled(true);
        setPinchZoom(true);
        setBackgroundColor(ContextCompat.getColor(getContext(), R.color.littleBlack));
        setDrawGridBackground(false);
        setDrawBarShadow(false);
        setDrawValueAboveBar(true);
        setBorderColor(Color.argb(100, 255, 255, 255)); // 半透明边框

        // 描述文本
        Description description = new Description();
        description.setText(chartTitle);
        description.setTextColor(Color.WHITE);
        description.setTextSize(12f);
        setDescription(description);

        // 图例样式
        Legend legend = getLegend();
        legend.setTextColor(Color.WHITE);
        legend.setForm(Legend.LegendForm.SQUARE);
        legend.setFormSize(12f);
        legend.setFormToTextSpace(5f);

        // 动画效果
        animateY(1500);
        animateX(1500);

        // 显示范围设置
        setVisibleXRangeMaximum(20);
        setVisibleXRangeMinimum(5);
        setAutoScaleMinMaxEnabled(true);
    }

    private void setupAxis() {
        // X轴设置
        XAxis xAxis = getXAxis();
        xAxis.setPosition(XAxis.XAxisPosition.BOTTOM);
        xAxis.setTextColor(Color.WHITE);
        xAxis.setGridColor(Color.argb(50, 0, 255, 255)); // 青色网格线
        xAxis.enableGridDashedLine(10f, 10f, 0f);
        xAxis.setGranularity(1f);
        xAxis.setValueFormatter(new TimeAxisFormatter());
        xAxis.setDrawGridLines(true);
        xAxis.setDrawAxisLine(true);
        xAxis.setAxisLineColor(Color.argb(100, 255, 255, 255));

        // Y轴设置
        YAxis leftAxis = getAxisLeft();
        leftAxis.setTextColor(Color.WHITE);
        leftAxis.setGridColor(Color.argb(50, 0, 255, 255));
        leftAxis.enableGridDashedLine(10f, 10f, 0f);
        leftAxis.setGranularity(1f);
        leftAxis.setAxisMinimum(0f); // 从0开始
        leftAxis.setAxisLineColor(Color.argb(100, 255, 255, 255));

        // 禁用右侧Y轴
        getAxisRight().setEnabled(false);
    }

//    public void addDataPoint(float value) {
//        float elapsedTimeSec = (System.currentTimeMillis() - startTime) / 1000f;
//        entries.add(new BarEntry(elapsedTimeSec, value));
//
//        // 限制显示最近20秒数据
//        if (elapsedTimeSec > 20) {
//            entries.remove(0);
//            getXAxis().setAxisMinimum(elapsedTimeSec - 20);
//        } else {
//            getXAxis().setAxisMinimum(0);
//        }
//
//        getXAxis().setAxisMaximum(elapsedTimeSec + 5);
//        moveViewToX(elapsedTimeSec);
//
//        updateChartData();
//    }
public void addDataPoint(float value) {
    // 使用数据点数量作为X值，实现等间距
    float xValue = dataCount++;
    entries.add(new BarEntry(xValue, value));

    // 限制显示最近20个数据点
    if (entries.size() > 20) {
        entries.remove(0);
        // 重置所有条目的X值以保持连续
        for (int i = 0; i < entries.size(); i++) {
            entries.get(i).setX(i);
        }
        dataCount = entries.size(); // 重置计数器
    }

    // 更新X轴范围
    getXAxis().setAxisMinimum(0);
    getXAxis().setAxisMaximum(Math.max(20, xValue + 1)); // 保持最小显示20个单位

    // 移动视图到最新位置
    moveViewToX(xValue);
    updateChartData();
}

    private void updateChartData() {
        if (entries.isEmpty()) return;

        BarDataSet dataSet;
        if (getData() == null || getData().getDataSetCount() == 0) {
            dataSet = createDataSet();
            BarData barData = new BarData(dataSet);
            barData.setBarWidth(0.5f); // 设置柱状图宽度
            setData(barData);
        } else {
            dataSet = (BarDataSet) getData().getDataSetByIndex(0);
            dataSet.setValues(entries);
        }

        getData().notifyDataChanged();
        // 在 updateChartData() 中
        post(() -> {
            notifyDataSetChanged();
            invalidate();
        });
    }

    private BarDataSet createDataSet() {
        BarDataSet dataSet = new BarDataSet(entries, dataLabel);

        // 柱状图样式
        dataSet.setColor(barColor);
        dataSet.setHighLightColor(highlightColor);
        dataSet.setHighlightEnabled(true);

        // 数值显示
        dataSet.setValueTextColor(Color.rgb(255, 200, 200));
        dataSet.setValueTextSize(10f);
        dataSet.setValueFormatter(new ValueFormatter() {
            @Override
            public String getFormattedValue(float value) {
                return String.valueOf((int) value);
            }
        });

        return dataSet;
    }

    // 自定义方法
    public void setChartTitle(String title) {
        this.chartTitle = title;
        getDescription().setText(title);
    }

    public void setDataLabel(String label) {
        this.dataLabel = label;
        if (getData() != null && getData().getDataSetCount() > 0) {
            getData().getDataSetByIndex(0).setLabel(label);
        }
    }

    public void setBarColor(int color) {
        this.barColor = color;
        if (getData() != null && getData().getDataSetCount() > 0) {
            BarDataSet dataSet = (BarDataSet) getData().getDataSetByIndex(0);
            dataSet.setColor(color);
        }
    }

    // 自定义时间轴格式化
//    private static class TimeAxisFormatter extends ValueFormatter {
//        @Override
//        public String getFormattedValue(float value) {
//            return String.valueOf((int) value);
//        }
//    }
    // 修改时间轴格式化器
    private class TimeAxisFormatter extends ValueFormatter {
        @Override
        public String getFormattedValue(float value) {
            int index = (int) value;
            if (index >= 0 && index < entries.size()) {
                long timestamp = System.currentTimeMillis() -
                        (long)((entries.size() - 1 - index) * 1000); // 假设每秒一个数据点
                return timeFormat.format(new Date(timestamp));
            }
            return "";
        }
    }
//    // 新增方法：清空数据
//    public void clearData() {
//        entries.clear();
//        dataCount = 0;
//        updateChartData();
//    }





    // 新增方法：专门用于处理日期型柱状图
//    public void addDailyDataPoint(float value, String date) {
//        // 用独立计数器避免影响原有逻辑
//        int dailyDataCount = entries.size();
//
//        // 添加数据点（X轴使用日期索引）
//        entries.add(new BarEntry(dailyDataCount, value));
//
//        // 保持最多7个数据点
//        if (entries.size() > 7) {
//            entries.remove(0);
//            // 重置所有X值保持连续
//            for (int i = 0; i < entries.size(); i++) {
//                entries.get(i).setX(i);
//            }
//        }
//
//        updateChartData();
//    }
//
//    // 新增日期格式化配置
//    public void enableDateMode(SimpleDateFormat dateFormat) {
//        getXAxis().setValueFormatter(new ValueFormatter() {
//            @Override
//            public String getFormattedValue(float value) {
//                int index = (int) value;
//                if (index >= 0 && index < dateLabels.size()) {
//                    return dateLabels.get(index); // 直接返回预存的日期标签
//                }
//                return "";
//            }
//        });
//    }
//
//    // 临时存储日期标签
//    private List<String> dateLabels = new ArrayList<>();
//
//    public void setDateLabels(List<String> labels) {
//        this.dateLabels = labels;
//    }




    // 模式标识 (新增)
    private static final int MODE_REALTIME = 0;
    private static final int MODE_DAILY = 1;
    private int currentMode = MODE_REALTIME;

    // 日期相关配置 (新增)
    private List<String> dateLabels = new ArrayList<>();
    private SimpleDateFormat dateFormat = new SimpleDateFormat("MM/dd", Locale.getDefault());

    // 修改X轴格式化器 (关键改动)
    private class DynamicAxisFormatter extends ValueFormatter {
        @Override
        public String getFormattedValue(float value) {
            if (currentMode == MODE_DAILY) {
                int index = (int) value;
                return (index >= 0 && index < dateLabels.size()) ? dateLabels.get(index) : "";
            }
            // 保留原有实时模式逻辑
            long timestamp = System.currentTimeMillis() - (long) ((entries.size() - 1 - value) * 1000);
            return timeFormat.format(new Date(timestamp));
        }
    }

    // 新增方法：设置日期模式 (关键API)
    public void enableDailyMode(List<String> dates) {
        this.currentMode = MODE_DAILY;
        this.dateLabels = new ArrayList<>(dates);
        getXAxis().setValueFormatter(new DynamicAxisFormatter());
        invalidate();
    }

    // 新增方法：添加日期模式数据点 (关键API)
    public void addDailyPoint(float value, int dayOffset) {
        entries.add(new BarEntry(dayOffset, value));
        if (entries.size() > 7) entries.remove(0);
        updateChartData();
    }
}
