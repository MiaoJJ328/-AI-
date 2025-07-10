package com.example.catvital.fragment;

import com.example.catvital.data.ExcretionDailyData;
import com.example.catvital.data.FoodDailyData;
import android.annotation.SuppressLint;
import android.icu.util.Calendar;
import android.os.Bundle;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;

import android.os.Handler;
import android.os.Looper;
import android.provider.Settings;
import android.text.TextUtils;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.inputmethod.EditorInfo;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import com.example.catvital.AI.ErnieAgentHelper;
import com.example.catvital.Chart.BarChartView;
import com.example.catvital.Chart.FoodBarChart;
import com.example.catvital.Chart.HeartRateBarChart;
import com.example.catvital.Chart.TempBarChart;
import com.example.catvital.R;
import com.github.mikephil.charting.formatter.ValueFormatter;
import com.example.catvital.Chart.ExcretionBarChart;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Random;

/**
 * A simple {@link Fragment} subclass.
 * Use the {@link aiFragment#newInstance} factory method to
 * create an instance of this fragment.
 */
public class aiFragment extends Fragment {

    // TODO: Rename parameter arguments, choose names that match
    // the fragment initialization parameters, e.g. ARG_ITEM_NUMBER
    public static final String ARG_PARAM1 = "param1";
    public static final String ARG_PARAM2 = "param2";
    private TextView ai_tv;
    // TODO: Rename and change types of parameters
    private String mParam1;
    private String mParam2;

    // 新增AI相关组件
    private EditText etInput;
    private TextView tvResult;
    private ScrollView scrollChat;
    private Button btnSend;

    // AI配置参数
    private static final String APP_ID = "FYBDvoCiUdHaDKNWRNTvYeWhwlqGyAk4";
    private static final String SECRET_KEY = "USBeNIb0PVFOFQANHM3SnB15kGJkRpG1";
    private ErnieAgentHelper agentHelper;
    private String openId;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());

    //七天统计数据而建
    private HeartRateBarChart heartRateChart;
    private TempBarChart tempChart;

    private FoodBarChart foodChart;

    private ExcretionBarChart excretionChart;

    public aiFragment() {
        // Required empty public constructor
    }

    /**
     * Use this factory method to create a new instance of
     * this fragment using the provided parameters.
     *
     * @param param1 Parameter 1.
     * @param param2 Parameter 2.
     * @return A new instance of fragment RightFragment.
     */
    // TODO: Rename and change types and number of parameters
    public static aiFragment newInstance(String param1, String param2) {
        aiFragment fragment = new aiFragment();
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
        return inflater.inflate(R.layout.fragment_ai, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        super.onViewCreated(view, savedInstanceState);

        // 初始化原有AI组件
        initAIViews(view);
        setupAI();

        // 初始化心率图表
        initHeartRateChart();
        // 初始化体温图表
        initTempChart(view);
        // 初始化yinshi图表
        initFoodChart(view);
        // 初始化排泄图表
        initExcretionChart(view);
    }

    private void initAIViews(View view) {
        // 绑定新组件
        etInput = view.findViewById(R.id.etInput);
        tvResult = view.findViewById(R.id.tvResult);
        scrollChat = view.findViewById(R.id.scroll_chat);
        btnSend = view.findViewById(R.id.btnSend);

        // 设置事件监听
        btnSend.setOnClickListener(v -> handleUserInput());
        etInput.setOnEditorActionListener((v, actionId, event) -> {
            if (actionId == EditorInfo.IME_ACTION_SEND) {
                handleUserInput();
                return true;
            }
            return false;
        });
    }

    private void setupAI() {
        // 生成设备ID
        String deviceId = Settings.Secure.getString(requireContext().getContentResolver(),
                Settings.Secure.ANDROID_ID);
        openId = deviceId + "_" + System.currentTimeMillis();
        if (openId.length() > 100) openId = openId.substring(0, 100);

        // 初始化AI助手
        agentHelper = new ErnieAgentHelper(requireContext(), APP_ID, SECRET_KEY);
    }

    private void handleUserInput() {
        String query = etInput.getText().toString().trim();
        if (query.isEmpty()) {
            showToast("请输入问题内容");
            return;
        }

        // 显示用户消息
        etInput.setText("");
        appendMessage("您: " + query);

        // 发送请求
        agentHelper.queryAgent(query, openId, new ErnieAgentHelper.ErnieCallback() {
            @Override
            public void onSuccess(String response, String threadId) {
                appendMessage("助手: " + response);
            }

            @Override
            public void onFailure(int errorCode, String errorMsg) {
                appendMessage("系统: 请求失败 (" + errorCode + ")");
            }
        });
    }

    private void appendMessage(String message) {
        mainHandler.post(() -> {
            String current = tvResult.getText().toString();
            tvResult.setText(current.isEmpty() ? message : current + "\n\n" + message);
            scrollToBottom();
        });
    }

    private void scrollToBottom() {
        scrollChat.post(() -> scrollChat.fullScroll(View.FOCUS_DOWN));
    }

    private void showToast(String msg) {
        mainHandler.post(() ->
                Toast.makeText(getContext(), msg, Toast.LENGTH_SHORT).show());
    }

    @Override
    public void onDestroy() {
        if (agentHelper != null) {
            agentHelper.clearSession();
        }
        super.onDestroy();
    }



    private void initHeartRateChart() {
        heartRateChart = requireView().findViewById(R.id.heartRateChart);

        // 静态测试数据（与日期顺序严格对应）
        List<Float> testData = Arrays.asList(72f, 75f, 80f, 78f, 85f, 82f, 88f);

        // 加载数据
        heartRateChart.loadLast7DaysData(testData);
    }

    //体温
    private void initTempChart(View view) {
        tempChart = view.findViewById(R.id.tempChart);

        // 1. 生成最近7天日期（动态）
        List<String> dates = generateLast7Days();

        // 2. 硬编码测试数据（与日期一一对应）
        List<TempBarChart.TempDailyData> data = Arrays.asList(
                new TempBarChart.TempDailyData(dates.get(0), 37.8f, 37.5f, 38.2f),
                new TempBarChart.TempDailyData(dates.get(1), 38.1f, 37.8f, 38.5f),
                new TempBarChart.TempDailyData(dates.get(2), 38.3f, 38.0f, 38.7f),
                new TempBarChart.TempDailyData(dates.get(3), 38.5f, 38.1f, 39.0f),
                new TempBarChart.TempDailyData(dates.get(4), 38.2f, 37.9f, 38.6f),
                new TempBarChart.TempDailyData(dates.get(5), 37.9f, 37.6f, 38.3f),
                new TempBarChart.TempDailyData(dates.get(6), 38.4f, 38.0f, 38.8f)
        );

        // 3. 加载数据
        tempChart.loadData(data);
    }

    private List<String> generateLast7Days() {
        List<String> dates = new ArrayList<>();
        SimpleDateFormat sdf = new SimpleDateFormat("MM/dd", Locale.CHINA);
        Calendar calendar = Calendar.getInstance();

        // 生成从6天前到今天的日期
        for (int i = 6; i >= 0; i--) {
            calendar.add(Calendar.DAY_OF_YEAR, -i);
            dates.add(sdf.format(calendar.getTime()));
            calendar.add(Calendar.DAY_OF_YEAR, i); // 重置
        }
        return dates; // 示例输出：["03/03", "03/04", ..., "03/09"]
    }



    private void initFoodChart(View view) {
        foodChart = view.findViewById(R.id.foodChart);

        // 1. 生成最近7天日期
        List<String> dates = generateLast7Days();

        // 2. 硬编码测试数据（与日期顺序严格对应）
        List<FoodDailyData> data = Arrays.asList(
                new FoodDailyData(dates.get(0), 15, 5, 20),   // 03/03
                new FoodDailyData(dates.get(1), 18, 7, 20),   // 03/04
                new FoodDailyData(dates.get(2), 12, 3, 20),   // 03/05
                new FoodDailyData(dates.get(3), 16, 4, 20),   // 03/06
                new FoodDailyData(dates.get(4), 14, 2, 20),   // 03/07
                new FoodDailyData(dates.get(5), 10, 4, 16),   // 03/08
                new FoodDailyData(dates.get(6), 13, 5, 20)    // 03/09
        );

        // 3. 加载数据
        foodChart.loadData(data);
    }



    private void initExcretionChart(View view) {
        ExcretionBarChart excretionChart = view.findViewById(R.id.excretionChart);

        // 生成最近7天日期
        List<String> dates = generateLast7Days();

        // 生成随机数据（含一天3次）
        List<Integer> data = generateExcretionData();

        // 加载数据
        excretionChart.loadLast7DaysData(data);
    }

    private List<Integer> generateExcretionData() {
        List<Integer> data = new ArrayList<>();
        Random random = new Random();
        int lowDayIndex = random.nextInt(7);

        for (int i = 0; i < 7; i++) {
            int count = (i == lowDayIndex) ? 3 : (4 + random.nextInt(3));
            data.add(count);
        }
        return data;
    }



    // 修改后的排泄数据生成方法
    private List<ExcretionDailyData> generateExcretionData(List<String> dates) {
        List<ExcretionDailyData> data = new ArrayList<>();
        Random random = new Random();
        int lowDayIndex = random.nextInt(7);

        for (int i = 0; i < dates.size(); i++) {
            int count = (i == lowDayIndex) ? 3 : (4 + random.nextInt(3));
            data.add(new ExcretionDailyData(dates.get(i), count));
        }
        return data;
    }

}


