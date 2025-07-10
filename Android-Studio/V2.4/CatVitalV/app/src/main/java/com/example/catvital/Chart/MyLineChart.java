package com.example.catvital.Chart;

import android.content.Context;
import android.graphics.BlurMaskFilter;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;

import androidx.core.content.ContextCompat;

import com.example.catvital.R;
import com.github.mikephil.charting.charts.LineChart;
import com.github.mikephil.charting.components.Description;
import com.github.mikephil.charting.components.Legend;
import com.github.mikephil.charting.components.XAxis;
import com.github.mikephil.charting.components.YAxis;
import com.github.mikephil.charting.data.Entry;
import com.github.mikephil.charting.data.LineData;
import com.github.mikephil.charting.data.LineDataSet;
import com.github.mikephil.charting.formatter.ValueFormatter;
import com.github.mikephil.charting.interfaces.datasets.ILineDataSet;
import com.github.mikephil.charting.renderer.LineChartRenderer;

import java.sql.Date;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class MyLineChart extends LineChart {

    private final List<Entry> entries = new ArrayList<>();
    private long startTime = System.currentTimeMillis();
    private String chartTitle = "实时数据监测";
    private String dataLabel = "数值";
    private int lineColor = Color.rgb(255, 50, 50);
    private int fillColor = Color.rgb(255, 50, 50);
    private int highlightColor = Color.rgb(255, 0, 0);
    private int dataCount = 0; // 添加这个成员变量记录数据点数量
    private SimpleDateFormat timeFormat = new SimpleDateFormat("HH:mm:ss", Locale.getDefault());

    private final List<Long> timestamps = new ArrayList<>(); // 记录每个数据点的时间戳

    public MyLineChart(Context context) {
        super(context);
        init(context);
    }

    public MyLineChart(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    public MyLineChart(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        init(context);
    }

//    private void init(Context context) {
//        // 延迟初始化确保父类完全初始化
//        post(() -> {
//            setupChartStyle();
//            setupAxis();
//            setupRenderer();
//        });
//    }
private void init(Context context) {
    post(() -> {
        setupChartStyle();
        setupAxis();
        setupRenderer();

        // 初始化默认数据
        if (entries.isEmpty()) {
            entries.add(new Entry(0, 0));
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
        setBackgroundColor(ContextCompat.getColor(getContext(), R.color.littleBlack));  // 使用与MainActivity相同的背景色
        setDrawGridBackground(false);
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
        legend.setForm(Legend.LegendForm.LINE);
        legend.setFormSize(12f);

        // 动画效果
        animateX(1500);
        animateY(1500);

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
        xAxis.setGranularity(1f);   //
        xAxis.setValueFormatter(new TimeAxisFormatter());

        // Y轴设置
        YAxis leftAxis = getAxisLeft();
        leftAxis.setTextColor(Color.WHITE);
        leftAxis.setGridColor(Color.argb(50, 0, 255, 255));
        leftAxis.enableGridDashedLine(10f, 10f, 0f);
        leftAxis.setGranularity(1f);

        // 禁用右侧Y轴
        getAxisRight().setEnabled(false);
    }

    private void
    setupRenderer() {
        setRenderer(new LineChartRenderer(this, getAnimator(), getViewPortHandler()) {
            @Override
            protected void drawLinear(Canvas c, ILineDataSet dataSet) {
                // 添加荧光外发光效果
                Paint paint = new Paint();
                paint.setStyle(Paint.Style.STROKE);
                paint.setStrokeWidth(6f);
                paint.setColor(Color.argb(50, 255, 0, 0));
                paint.setMaskFilter(new BlurMaskFilter(15, BlurMaskFilter.Blur.NORMAL));

                super.drawLinear(c, dataSet);
            }
        });
    }

    public void addDataPoint(float value) {
        float xValue = dataCount++;
        long currentTime = System.currentTimeMillis();

        entries.add(new Entry(xValue, value));
        timestamps.add(currentTime); // 记录当前时间戳

        // 限制显示最近20个数据点
        if (entries.size() > 20) {
            entries.remove(0);
            timestamps.remove(0);
            // 重置剩余条目的X值以保持连续
            for (int i = 0; i < entries.size(); i++) {
                entries.get(i).setX(i);
            }
            dataCount = entries.size(); // 重置计数器
        }

        getXAxis().setAxisMinimum(0);
        getXAxis().setAxisMaximum(Math.max(20, xValue + 1));
        moveViewToX(xValue);

        updateChartData();
    }

    private void updateChartData() {
        if (entries.isEmpty()) return;

        LineDataSet dataSet;
        if (getData() == null || getData().getDataSetCount() == 0) {
            dataSet = createDataSet();
            LineData lineData = new LineData(dataSet);
            setData(lineData);
        } else {
            dataSet = (LineDataSet) getData().getDataSetByIndex(0);
            dataSet.setValues(entries);
        }

        getData().notifyDataChanged();
        notifyDataSetChanged();
        invalidate();
    }

    private LineDataSet createDataSet() {
        LineDataSet dataSet = new LineDataSet(entries, dataLabel);

        // 线条样式
        dataSet.setColor(lineColor);
        dataSet.setLineWidth(2f);

        // 填充效果
        dataSet.setDrawFilled(true);
        dataSet.setFillColor(fillColor);
        dataSet.setFillAlpha(50);

        // 高亮效果
        dataSet.setHighLightColor(highlightColor);
        dataSet.setHighlightLineWidth(1f);
        dataSet.setDrawHorizontalHighlightIndicator(false);

        // 数据点样式
        dataSet.setCircleColor(highlightColor);
        dataSet.setCircleRadius(3f);
        dataSet.setCircleHoleRadius(1.5f);
        dataSet.setCircleHoleColor(Color.rgb(255, 100, 100));

        // 数值显示
        dataSet.setValueTextColor(Color.rgb(255, 200, 200));
        dataSet.setValueTextSize(10f);

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

    public void setLineColor(int color) {
        this.lineColor = color;
        this.fillColor = color;
        if (getData() != null && getData().getDataSetCount() > 0) {
            LineDataSet dataSet = (LineDataSet) getData().getDataSetByIndex(0);
            dataSet.setColor(color);
            dataSet.setFillColor(color);
        }
    }

    // 自定义时间轴格式化
    private class TimeAxisFormatter extends ValueFormatter {
        @Override
        public String getFormattedValue(float value) {
            int index = (int) value;
            if (index >= 0 && index < timestamps.size()) {
                return timeFormat.format(new Date(timestamps.get(index)));
            }
            return "";
        }
    }
}