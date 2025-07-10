package com.example.catvital.fragment;

import android.annotation.SuppressLint;
import android.os.Bundle;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.widget.SwitchCompat;
import androidx.cardview.widget.CardView;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;

import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.SeekBar;
import android.widget.TextView;

import com.example.catvital.ListView.TimeWeightAdapter;
import com.example.catvital.ListView.TimeWeightItem;
import com.example.catvital.MainActivity;
import com.example.catvital.MySQLite.CatVitalApp;
import com.example.catvital.MySQLite.DatabaseHelper;
import com.example.catvital.R;
import com.example.catvital.SharedViewModel.SharedViewModel;
import com.google.gson.Gson;
import com.google.gson.JsonObject;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A simple {@link Fragment} subclass.
 * Use the {@link cleanFragment#newInstance} factory method to
 * create an instance of this fragment.
 */
public class cleanFragment extends Fragment {

    // TODO: Rename parameter arguments, choose names that match
    // the fragment initialization parameters, e.g. ARG_ITEM_NUMBER
    private static final String ARG_PARAM1 = "param1";
    private static final String ARG_PARAM2 = "param2";
    private static final String TAG = "cleanFragment";

    // TODO: Rename and change types of parameters
    private String mParam1;
    private String mParam2;
    private SwitchCompat deodorization;
    private SwitchCompat autoclean;
    private SwitchCompat pushwater;
    private SwitchCompat led;
    private SwitchCompat auto_clean_switch;
    private SwitchCompat room_camera_switch;
    private SwitchCompat litter_box_camera_switch;
    private SwitchCompat remote_control_switch;
    private EditText clean_after_count;
    private Button btn_onekey_clean;
    private Button btn_stop;
    private Button btn_forward;
    private Button btn_backward;
    private Button btn_left;
    private Button btn_right;
    private SeekBar seek_speed;
    private SeekBar seek_turn_speed;
    private ListView listpull;
    private WebView webView;
    public SharedView cleansharedView;
    private CompoundButton.OnCheckedChangeListener switchListener;
    private SharedViewModel shareviewmodel;
    private int camera = 0;   //初始状态0表示没有开启任何一个摄像头，1表示开启房间摄像头， 2表示开启猫砂盆的摄像头
    private TextView camerastatetext;
    private TextView tv_speed;
    private TextView tv_turn_speed;
    private float linespeed = 0;
    private float anglespeed = 0;

    //用于设置按钮长按
    private Handler handler = new Handler(); // 全局一个Handler
    private Map<Button, Runnable> runnableMap = new HashMap<>(); // 存储各按钮的Runnable
    private TimeWeightAdapter cleanadapter;
    private CardView timepullcard;
    String carCamUrl = "http://192.168.232.11/cam1/mjpeg/1";
    String roomCamUrl = "http://192.168.232.13/mjpeg/1";

    public cleanFragment() {
        // Required empty public constructor
    }

    /**
     * Use this factory method to create a new instance of
     * this fragment using the provided parameters.
     *
     * @param param1 Parameter 1.
     * @param param2 Parameter 2.
     * @return A new instance of fragment cleanFragment.
     */
    // TODO: Rename and change types and number of parameters
    public static cleanFragment newInstance(String param1, String param2) {
        cleanFragment fragment = new cleanFragment();
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
        return inflater.inflate(R.layout.fragment_clean, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        cleansharedView = new ViewModelProvider(requireActivity()).get(SharedView.class);
        shareviewmodel = new ViewModelProvider(requireActivity()).get(SharedViewModel.class);

        InitView(view);
        ListViewInit(view);

        // 3. 设置数据观察者（必须先于数据加载）
        setupDataObserver();
        //获取数据库中的数据
        loadInitialData();
        ///设置SeekBar的监听器
        SeekSpeedListener();
        SeekTurnSpeedLinster();

    }

    /*********************************ListView*****************************************/

    // 确保可以外部调用更新
    public void refreshData() {
        if (isAdded() && !isDetached()) {
            onDatabaseUpdated();
        }
    }

    // 当数据库更新时调用
    public void onDatabaseUpdated() {
        loadDataFromDatabase();
    }

    // 当数据库清空时调用
    public void onDatabaseCleared() {
        shareviewmodel.clearDataList();
    }

    private void loadInitialData() {
        new Thread(() -> {
            try {
                List<TimeWeightItem> items = CatVitalApp.getDbHelper().getAllFoodRecords();
                requireActivity().runOnUiThread(() -> {
                    shareviewmodel.setDataList(items);
                });
            } catch (Exception e) {
                Log.e(TAG, "数据加载失败", e);
            }
        }).start();
    }

    private void ListViewInit(@NonNull View view) {
        ListView listpull = view.findViewById(R.id.listpull);
        DatabaseHelper dbHelper = CatVitalApp.getDbHelper();
        // 观察数据变化
        shareviewmodel.getDataList().observe(getViewLifecycleOwner(), items -> {
            if (cleanadapter == null) {
                cleanadapter = new TimeWeightAdapter(requireContext(), items);
                listpull.setAdapter(cleanadapter);
            } else {
                cleanadapter.updateData(items);
            }
        });

        // 添加示例数据（确保ViewModel已初始化）
//        shareviewmodel.addItem(new TimeWeightItem("12:30:00", "30g"));
//        shareviewmodel.addItem(new TimeWeightItem("0:30:45", "15g"));
//        shareviewmodel.addItem(new TimeWeightItem("1:05:23", "30g"));

        listpull.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                cleanadapter.setShowAllItems(!cleanadapter.showAllItems);

                // 计算 ListView 的新高度
                int newHeight = calculateListViewHeight(listpull);

                // 设置 ListView 的新高度
                ViewGroup.LayoutParams params = listpull.getLayoutParams();
                params.height = newHeight;
                listpull.setLayoutParams(params);

                // 请求重新布局
                timepullcard.requestLayout();
            }
        });
    }

    /**
     * 计算 ListView 所需高度的方法
     * @param listView
     * @return
     */
    private int calculateListViewHeight(ListView listView) {
        int totalHeight = 0;
        int desiredWidth = View.MeasureSpec.makeMeasureSpec(listView.getWidth(), View.MeasureSpec.AT_MOST);

        for (int i = 0; i < listView.getAdapter().getCount(); i++) {
            View listItem = listView.getAdapter().getView(i, null, listView);
            listItem.measure(desiredWidth, View.MeasureSpec.UNSPECIFIED);
            totalHeight += listItem.getMeasuredHeight();
        }

        // 加上分割线高度
        totalHeight += listView.getDividerHeight() * (listView.getAdapter().getCount() - 1);

        return totalHeight;
    }

    /**
     *  从SQLite数据库加载数据
     */
    private void loadDataFromDatabase() {
        new Thread(() -> {
            Log.d(TAG, "获取数据库排泄数据");
            List<TimeWeightItem> items = CatVitalApp.getDbHelper().getAllCleanRecords();
            requireActivity().runOnUiThread(() -> {
                shareviewmodel.setDataList(items);
            });
        }).start();
    }

    private void setupDataObserver() {
        shareviewmodel.getDataList().observe(getViewLifecycleOwner(), items -> {
            if (items == null) return;

            if (cleanadapter == null) {
                cleanadapter = new TimeWeightAdapter(requireContext(), items);
                listpull.setAdapter(cleanadapter);
            } else {
                cleanadapter.updateData(items);
            }
        });
    }

    // 当数据库更新时调用
    public void cleanonDatabaseUpdated() {
        loadDataFromDatabase();
    }
    /*********************************ListView*****************************************/

    private void InitView(@NonNull View view) {

        /**********switch************/
        deodorization = view.findViewById(R.id.deodorization);
        autoclean = view.findViewById(R.id.autoclean);
        pushwater = view.findViewById(R.id.pushwater);
        led = view.findViewById(R.id.led);
        auto_clean_switch = view.findViewById(R.id.auto_clean_switch);
        room_camera_switch = view.findViewById(R.id.room_camera_switch);
        litter_box_camera_switch = view.findViewById(R.id.litter_box_camera_switch);
        remote_control_switch = view.findViewById(R.id.remote_control_switch);
        /**********switch************/

        /**********button************/
        btn_onekey_clean = view.findViewById(R.id.btn_onekey_clean);
        btn_stop = view.findViewById(R.id.btn_stop);
        btn_forward = view.findViewById(R.id.btn_forward);
        btn_backward = view.findViewById(R.id.btn_backward);
        btn_left = view.findViewById(R.id.btn_left);
        btn_right = view.findViewById(R.id.btn_right);
        // 为每个Button设置监听器
        setupButtonListeners();
        /**********button************/

        /**********SeekBar************/
        seek_speed = view.findViewById(R.id.seek_speed);
        seek_turn_speed = view.findViewById(R.id.seek_turn_speed);
        seek_speed.setProgress(0);
        seek_speed.setMax(100);
        seek_turn_speed.setProgress(0);
        seek_turn_speed.setMax(200);
        /**********SeekBar************/

        /**********ListView***********/
        listpull = view.findViewById(R.id.listpull);
        /**********ListView***********/

        /**********webView***********/
        webView = view.findViewById(R.id.webView);
        WebSettings webSettings = webView.getSettings();
        webSettings.setJavaScriptEnabled(true);
        webSettings.setLoadWithOverviewMode(true);
        webSettings.setUseWideViewPort(true);
        /**********webView***********/

        /***********恢复SeekBar的值****/
        // 恢复SeekBar的值
        int savedSpeed = cleansharedView.getSeekBarValue(R.id.seek_speed);
        int savedTurnSpeed = cleansharedView.getSeekBarValue(R.id.seek_turn_speed);
        seek_speed.setProgress(savedSpeed);
        seek_turn_speed.setProgress(savedTurnSpeed);
        /***********恢复SeekBar的值****/

        /**********TextView**********/
        camerastatetext = view.findViewById(R.id.camera_status);
        tv_speed = view.findViewById(R.id.tv_speed);
        tv_turn_speed = view.findViewById(R.id.tv_turn_speed);
        linespeed = seek_speed.getProgress();
        String linespeed = String.valueOf(this.linespeed);
        anglespeed = seek_turn_speed.getProgress();
        String angelspeed = String.valueOf(anglespeed);
        tv_speed.setText("线速度：" + linespeed);
        tv_turn_speed.setText("角速度：" + angelspeed);

        /**********TextView**********/

        /**********EditText**********/
        clean_after_count = view.findViewById(R.id.clean_after_count);
        /**********EditText**********/

        switchprocess();

        /**********绑定控件***********/
        checkAndBind(deodorization, "deodorization");
        checkAndBind(autoclean, "autoclean");
        checkAndBind(pushwater, "pushwater");
        checkAndBind(led, "led");
        checkAndBind(auto_clean_switch, "auto_clean_switch");
        checkAndBind(room_camera_switch, "room_camera_switch");
        checkAndBind(litter_box_camera_switch, "litter_box_camera_switch");
        checkAndBind(remote_control_switch, "remote_control_switch");
        /**********绑定控件***********/

    }

    /*********************************WebView*****************************************/
    /**
     * 安全的销毁WebView
     */
    @Override
    public void onDestroyView() {
        super.onDestroyView();
        if (webView != null) {
            webView.stopLoading();          // 停止加载
            webView.clearHistory();         // 清空历史记录
            webView.setWebChromeClient(null);
            webView.setWebViewClient(null); // 移除监听器
            webView.destroy();              // 销毁 WebView
            webView = null;                 // 置空引用（防止内存泄漏）
        }
        // 清理 Handler 防止内存泄漏
        if (handler != null) {
            handler.removeCallbacksAndMessages(null);
        }
    }
    /*********************************WebView*****************************************/

    /*********************************Button*****************************************/

    @SuppressLint("ClickableViewAccessibility")
    private void setupButtonListeners() {
        // 一键清洁按钮
        btn_onekey_clean.setOnClickListener(v -> {
            Log.d(TAG, "一键清洁按钮被点击");
            PublishInt("basin_clean", 1);
        });

        // 停止按钮
        btn_stop.setOnClickListener(v -> {
            Log.d(TAG, "停止按钮被点击");
            // 执行停止逻辑
            if(remote_control_switch.isChecked())
            {
                PublishInt("car_stop", 1);
            }

        });

        // 前进按钮
        setupButton(btn_forward);
        // 后退按钮
        setupButton(btn_backward);
        // 左转按钮
        setupButton(btn_left);
        // 右转按钮
        setupButton(btn_right);

    }

    @SuppressLint("ClickableViewAccessibility")
    private void setupButton(Button button) {
        Runnable runnable = new Runnable() {
            @Override
            public void run() {
                if(remote_control_switch.isChecked())
                {
                    if(button.getId() == R.id.btn_forward)
                    {
                        PublishFloat("car_linespeed", linespeed);
                    }
                    else if(button.getId() == R.id.btn_backward)
                    {
                        PublishFloat("car_linespeed", -linespeed);
                    }
                    else if(button.getId() == R.id.btn_left)
                    {
                        PublishFloat("car_anglespeed", anglespeed);
                    }
                    else if(button.getId() == R.id.btn_right)
                    {
                        PublishFloat("car_anglespeed", -anglespeed);
                    }
                    handler.postDelayed(this, 5000); // 循环执行
                }
            }
        };
        runnableMap.put(button, runnable); // 保存Runnable

        button.setOnTouchListener((v, event) -> {
            switch (event.getAction()) {
                case MotionEvent.ACTION_DOWN:
                    handler.postDelayed(runnable, 1000); // 启动任务
                    return true;
                case MotionEvent.ACTION_UP:
                case MotionEvent.ACTION_CANCEL:
                    handler.removeCallbacks(runnable); // 停止当前按钮的任务
                    if(remote_control_switch.isChecked())
                    {
                        if(button.getId() == R.id.btn_forward)
                        {
                            PublishFloat("car_linespeed", 0);
                        }
                        else if(button.getId() == R.id.btn_backward)
                        {
                            PublishFloat("car_linespeed", 0);
                        }
                        else if(button.getId() == R.id.btn_left)
                        {
                            PublishFloat("car_anglespeed", 0);
                        }
                        else if(button.getId() == R.id.btn_right)
                        {
                            PublishFloat("car_anglespeed", 0);
                        }
                    }
                    return true;
            }
            return false;
        });
    }

    private static void PublishInt(String key, int value) {
        //阿里云发布消息
        JsonObject jsonObject = new JsonObject();
        jsonObject.addProperty(key, value);
        String jsonPayload = new Gson().toJson(jsonObject);
        MainActivity.aliyunClient.publishMessage(jsonPayload);
    }

    private static void PublishFloat(String key, float value) {
        //阿里云发布消息
        JsonObject jsonObject = new JsonObject();
        jsonObject.addProperty(key, value);
        String jsonPayload = new Gson().toJson(jsonObject);
        MainActivity.aliyunClient.publishMessage(jsonPayload);
    }
    /*********************************Button*****************************************/


    /*********************************SeekBar*****************************************/
    private void SeekTurnSpeedLinster() {
        seek_turn_speed.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                // 当进度改变时调用
//                Log.d(TAG, "速度值改变: " + progress);
                anglespeed = (float) progress / 100;
                // 实时保存值
                cleansharedView.saveSeekBarValue(R.id.seek_turn_speed, progress);
                String anglespeedstr = String.valueOf(anglespeed);
                tv_turn_speed.setText("角速度：" + anglespeedstr);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                // 当用户开始拖动时调用
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                // 当用户停止拖动时调用
                int finalValue = seekBar.getProgress();
                Log.d(TAG, "最终角速度值: " + anglespeed);
            }
        });
    }

    private void SeekSpeedListener() {
        // 为seek_speed设置监听器
        seek_speed.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                // 当进度改变时调用
//                Log.d(TAG, "速度值改变: " + progress);
                linespeed = (float) progress  / 100;
                // 实时保存值
                cleansharedView.saveSeekBarValue(R.id.seek_speed, progress);
                String linespeedstr = String.valueOf(linespeed);
                tv_speed.setText("线速度：" + linespeedstr);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                // 当用户开始拖动时调用
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                // 当用户停止拖动时调用
                int finalValue = seekBar.getProgress();
                Log.d(TAG, "最终线速度值: " + linespeed);
            }
        });
    }
    /*********************************SeekBar*****************************************/


    /*********************************switch******************************************/
    private void switchprocess()
    {
        Log.d(TAG, "switchprocess() 方法被调用"); // 确认是否执行
        switchListener = (buttonView, isChecked) -> {

            int switchId = buttonView.getId(); // 获取当前触发 Switch 的 ID
            // 保存状态到 ViewModel
            cleansharedView.saveButtonState(buttonView.getId(), isChecked);
            if (switchId == R.id.deodorization) {
                // 处理 switch1 的逻辑
                Log.d("除臭", "除臭 状态: " + isChecked);
                //阿里云发布消息
                JsonObject jsonObject = new JsonObject();
                if(isChecked)
                {
                    jsonObject.addProperty("Remove_odor", 1);

                }
                else
                {
                    jsonObject.addProperty("Remove_odor", 0);
                }
                String jsonPayload = new Gson().toJson(jsonObject);
                MainActivity.aliyunClient.publishMessage(jsonPayload);

            }
            else if (switchId == R.id.autoclean) {
                // 处理 switch2 的逻辑
                Log.d("手动清理", "手动清理 状态: " + isChecked);
                //阿里云发布消息
                JsonObject jsonObject = new JsonObject();
                if(isChecked)
                {
                    jsonObject.addProperty("Remove_odor", 1);
                    // 延时1秒执行
                    new Handler(Looper.getMainLooper()).postDelayed(new Runnable() {
                        @Override
                        public void run() {
                            setSwitchStateWithoutCallback(autoclean, false);
                        }
                    }, 1000); // 1000毫秒 = 1秒
                }
                String jsonPayload = new Gson().toJson(jsonObject);
                MainActivity.aliyunClient.publishMessage(jsonPayload);
            }
            else if (switchId == R.id.pushwater) {
                // 处理 switch3 的逻辑
                Log.d("冲厕", "冲厕 状态: " + isChecked);
                //阿里云发布消息
                JsonObject jsonObject = new JsonObject();
                if(isChecked)
                {
                    jsonObject.addProperty("pushwater", 1);
                    // 延时1秒执行
                    new Handler(Looper.getMainLooper()).postDelayed(new Runnable() {
                        @Override
                        public void run() {
                            setSwitchStateWithoutCallback(pushwater, false);
                        }
                    }, 1000); // 1000毫秒 = 1秒
                }
                String jsonPayload = new Gson().toJson(jsonObject);
                MainActivity.aliyunClient.publishMessage(jsonPayload);
            }
            else if (switchId == R.id.led) {
                // 处理 switch3 的逻辑
                Log.d("厕所灯", "厕所灯 状态: " + isChecked);
                //阿里云发布消息
                JsonObject jsonObject = new JsonObject();
                if(isChecked)
                {
                    jsonObject.addProperty("led", 1);
                }
                else
                {
                    jsonObject.addProperty("led", 0);
                }
                String jsonPayload = new Gson().toJson(jsonObject);
                MainActivity.aliyunClient.publishMessage(jsonPayload);
            }
            else if (switchId == R.id.auto_clean_switch) {
                // 处理 switch3 的逻辑
                Log.d("自动清洁", "自动清洁 状态: " + isChecked);

                int times = new SafeConverter(clean_after_count.getText().toString()).toInt(1);
                if(isChecked)
                {
                    //阿里云发布消息
                    JsonObject jsonObject = new JsonObject();
                    jsonObject.addProperty("clean_counts", 1);
                    jsonObject.addProperty("counts", times);
                    String jsonPayload = new Gson().toJson(jsonObject);
                    MainActivity.aliyunClient.publishMessage(jsonPayload);
                }
            }
            else if (switchId == R.id.room_camera_switch) {
                // 处理 switch3 的逻辑
                Log.d("室内摄像头", "室内摄像头 状态: " + isChecked);
                if(isChecked)
                {
                    camera = 1;
                    camerastatetext.setText("");
                    setSwitchStateWithoutCallback(litter_box_camera_switch, false);
                    webView.stopLoading();
                    webView.loadUrl(roomCamUrl);
                }
                else if(!room_camera_switch.isChecked() && !litter_box_camera_switch.isChecked())
                {
                    camera = 0;
                    camerastatetext.setText("摄像头已关闭");
                    webView.stopLoading();
                    webView.loadUrl("about:blank");
                }

            }
            else if (switchId == R.id.litter_box_camera_switch) {
                // 处理 switch3 的逻辑
                Log.d("猫砂盆摄像头", "猫砂盆摄像头 状态: " + isChecked);
                if(isChecked)
                {
                    camera = 2;
                    camerastatetext.setText("");
                    setSwitchStateWithoutCallback(room_camera_switch, false);
                    webView.stopLoading();
                    webView.loadUrl(carCamUrl);
                }
                else if(!room_camera_switch.isChecked() && !litter_box_camera_switch.isChecked())
                {
                    camera = 0;
                    camerastatetext.setText("摄像头已关闭");
                    webView.stopLoading();
                    webView.loadUrl("about:blank");
                }
            }
            else if (switchId == R.id.remote_control_switch) {
                // 处理 switch3 的逻辑
                Log.d("远程遥控", "远程遥控 状态: " + isChecked);
                //阿里云发布消息
                JsonObject jsonObject = new JsonObject();
                if(isChecked)
                {
                    jsonObject.addProperty("remotecontrol", 1);
                }
                else
                {
                    jsonObject.addProperty("remotecontrol", 0);
                }
                String jsonPayload = new Gson().toJson(jsonObject);
                MainActivity.aliyunClient.publishMessage(jsonPayload);
            }
        };
    }

    /**
     * 设置Switch状态但不触发回调（统一监听器版本）
     * @param switchView 要设置的Switch控件
     * @param checked 要设置的状态
     */
    private void setSwitchStateWithoutCallback(SwitchCompat switchView, boolean checked) {
        if (switchView == null) return;

        // 1. 临时移除统一监听器
        switchView.setOnCheckedChangeListener(null);

        // 2. 设置状态（不会触发监听器）
        switchView.setChecked(checked);

        // 3. 保存状态到ViewModel
        if (cleansharedView != null) {
            cleansharedView.saveButtonState(switchView.getId(), checked);
        }

        // 4. 恢复统一监听器
        switchView.setOnCheckedChangeListener(switchListener);
    }


    /**
     * 检查控件是否绑定成功
     * @param switchView
     * @param name
     */
    private void checkAndBind(SwitchCompat switchView, String name) {
        if (switchView == null) {
            Log.e(TAG, "❌ " + name + " 绑定失败！请检查布局文件");
        } else {
            Log.d(TAG, "✅ " + name + " 绑定成功");
            // 先设置监听器，再恢复状态
            switchView.setOnCheckedChangeListener(switchListener);
            // 恢复保存的状态
            boolean savedState = cleansharedView.getButtonState(switchView.getId());
            switchView.setChecked(savedState);
            //设置监听器
            switchView.setOnCheckedChangeListener(switchListener);

        }
    }
    /*********************************switch******************************************/
}