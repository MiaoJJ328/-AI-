package com.example.catvital.fragment;

import android.os.Bundle;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import androidx.core.content.ContextCompat;
import androidx.lifecycle.ViewModelProvider;

import android.os.Handler;
import android.os.Looper;
import android.text.TextUtils;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import com.example.catvital.Chart.BarChartView;
import com.example.catvital.Chart.MyLineChart;
import com.example.catvital.R;
import com.example.catvital.SharedViewModel.SharedViewModel;

import java.util.Random;
import java.util.Timer;
import java.util.TimerTask;

/**
 * A simple {@link Fragment} subclass.
 * Use the {@link healthFragment#newInstance} factory method to
 * create an instance of this fragment.
 */
public class healthFragment extends Fragment {

    // TODO: Rename parameter arguments, choose names that match
    // the fragment initialization parameters, e.g. ARG_ITEM_NUMBER
    private static final String ARG_PARAM1 = "param1";
    private static final String ARG_PARAM2 = "param2";

    // TODO: Rename and change types of parameters
    private String mParam1;
    private String mParam2;
    private TextView heartRateValue;
    private TextView oxygenValue;
    private TextView temperatureValue;
    private BarChartView temp;    //体温
    private MyLineChart heart_rate; //心率
    private MyLineChart oxygen; //心率
    private final Random random = new Random(); //模拟演示

    //模拟数据生成
    private Handler dataUpdateHandler = new Handler(Looper.getMainLooper());
    private Runnable dataUpdateRunnable;
    private static final long UPDATE_INTERVAL = 10000; // 10秒更新间隔
    private SharedViewModel healthDataViewModel;

    public healthFragment() {
        // Required empty public constructor
    }

    /**
     * Use this factory method to create a new instance of
     * this fragment using the provided parameters.
     *
     * @param param1 Parameter 1.
     * @param param2 Parameter 2.
     * @return A new instance of fragment MiddleFragment.
     */
    // TODO: Rename and change types and number of parameters
    public static healthFragment newInstance(String param1, String param2) {
        healthFragment fragment = new healthFragment();
        Bundle args = new Bundle();
        args.putString(ARG_PARAM1, param1);
        args.putString(ARG_PARAM2, param2);
        fragment.setArguments(args);
        return fragment;
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (getArguments() != null) {
            mParam1 = getArguments().getString(ARG_PARAM1);
            mParam2 = getArguments().getString(ARG_PARAM2);
        }
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // Inflate the layout for this fragment
        return inflater.inflate(R.layout.fragment_health, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        //获取ViewModel实例
        healthDataViewModel = new ViewModelProvider(requireActivity()).get(SharedViewModel.class);

        // 1. 先初始化所有视图
        heartRateValue = view.findViewById(R.id.heartRateValue);
        oxygenValue = view.findViewById(R.id.oxygenValue);
        temperatureValue = view.findViewById(R.id.temperatureValue);

        temp = view.findViewById(R.id.temperatureChart);
        heart_rate = view.findViewById(R.id.heartRateChart);
        oxygen = view.findViewById(R.id.oxygenChart);

        // 2. 检查视图是否初始化成功
        if (temp == null || heart_rate == null || oxygen == null) {
            throw new IllegalStateException("Some chart views are not found in layout");
        }

        // 3. 配置图表属性
        temp.setChartTitle("历史体温记录");
        temp.setDataLabel("体温");
        temp.setBarColor(ContextCompat.getColor(requireContext(), R.color.体温));

        heart_rate.setChartTitle("实时心率检测");
        heart_rate.setDataLabel("心率");

        oxygen.setChartTitle("实时血氧检测");
        oxygen.setDataLabel("血氧");
        oxygen.setLineColor(ContextCompat.getColor(requireContext(), R.color.血氧));
        //恢复之前的数据
        restoreDataFromViewModel();
        // 4. 初始化数据更新任务
        initDataUpdateTask();
    }


    private void restoreDataFromViewModel() {
        // 恢复心率数据
        for (Float value : healthDataViewModel.getHeartRateData()) {
            heart_rate.addDataPoint(value);
        }

        // 恢复血氧数据
        for (Float value : healthDataViewModel.getOxygenData()) {
            oxygen.addDataPoint(value);
        }

        // 恢复体温数据
        for (Float value : healthDataViewModel.getTemperatureData()) {
            temp.addDataPoint(value);
        }

        // 更新文本显示
        if (!healthDataViewModel.getHeartRateData().isEmpty()) {
            heartRateValue.setText(String.format("%.1f bpm", healthDataViewModel.getLastHeartRate()));
        }
        if (!healthDataViewModel.getOxygenData().isEmpty()) {
            oxygenValue.setText(String.format("%.1f%%", healthDataViewModel.getLastOxygen()));
        }
        if (!healthDataViewModel.getTemperatureData().isEmpty()) {
            temperatureValue.setText(String.format("%.1f°C", healthDataViewModel.getLastTemperature()));
        }
    }


    private void initDataUpdateTask() {
        dataUpdateRunnable = new Runnable() {
            @Override
            public void run() {
                if (isAdded() && getActivity() != null) {
                    // 生成模拟数据 - 每个指标在不同范围内
                    float heartRate = 60 + random.nextFloat() * 40; // 60-100 bpm
                    float oxygenLevel = 95 + random.nextFloat() * 4; // 94-98%
                    float temperature = 36 + random.nextFloat(); // 37-38°C

                    // 保存到ViewModel
                    healthDataViewModel.addHeartRateData(heartRate);
                    healthDataViewModel.addOxygenData(oxygenLevel);
                    healthDataViewModel.addTemperatureData(temperature);

                    // 更新图表数据
                    heart_rate.addDataPoint(heartRate);
                    oxygen.addDataPoint(oxygenLevel);
                    temp.addDataPoint(temperature);

                    // 更新文本显示
                    heartRateValue.setText(String.format("%.1f bpm", heartRate));
                    oxygenValue.setText(String.format("%.1f%%", oxygenLevel));
                    temperatureValue.setText(String.format("%.1f°C", temperature));

                    // 安排下一次更新
                    dataUpdateHandler.postDelayed(this, UPDATE_INTERVAL);
                }
            }
        };
    }

    @Override
    public void onResume() {
        super.onResume();
        // 开始数据更新
        startDataUpdates();
    }
    @Override
    public void onPause() {
        super.onPause();
        // 停止数据更新
        stopDataUpdates();
    }
    private void startDataUpdates() {
        // 立即执行第一次更新
        dataUpdateHandler.post(dataUpdateRunnable);
    }

    private void stopDataUpdates() {
        // 移除所有待执行的更新任务
        dataUpdateHandler.removeCallbacks(dataUpdateRunnable);
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        // 确保Handler被清理
        stopDataUpdates();
        dataUpdateHandler = null;
    }
}