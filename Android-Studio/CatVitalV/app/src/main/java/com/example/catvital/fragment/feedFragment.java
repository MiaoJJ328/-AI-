package com.example.catvital.fragment;

import static com.example.catvital.fragment.BeijingTimeUtils.getBeijingTimeComponents;

import android.os.Bundle;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.widget.SwitchCompat;
import androidx.cardview.widget.CardView;
import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentManager;
import androidx.lifecycle.ViewModelProvider;

import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.ProgressBar;
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

import java.util.ArrayList;
import java.util.List;

/**
 * A simple {@link Fragment} subclass.
 * Use the {@link feedFragment#newInstance} factory method to
 * create an instance of this fragment.
 */
public class feedFragment extends Fragment {

    // TODO: Rename parameter arguments, choose names that match
    // the fragment initialization parameters, e.g. ARG_ITEM_NUMBER
    public static final String ARG_PARAM1 = "param1";
    public static final String ARG_PARAM2 = "param2";
    private static final String TAG = "Feed";

    // TODO: Rename and change types of parameters
    /**********ListView***********/
    private TimeWeightAdapter adapter;
    private List<TimeWeightItem> dataList;
    /**********ListView***********/
    private String mParam1;
    private String mParam2;
    private TextView fragfeed_tv;
    private SwitchCompat switch_food, switch_water;
    private SwitchCompat  food_byhand;
    private SwitchCompat  water_byhand;
    private SwitchCompat  food_water_play;
    private SwitchCompat  clockwise;
    private SwitchCompat  anticlockwise;
    private SwitchCompat  auto_food;
    private EditText feedHoursInput;
    private EditText feedMinutesInput;
    private EditText maxFeedAmountInput;
    private ProgressBar foodLevelProgress;
    private ProgressBar waterLevelProgress;
    private CompoundButton.OnCheckedChangeListener switchListener;
    public SharedView sharedView;
    private CardView cardView;
    private SharedViewModel shareviewmodel;
    private ListView listView;


    public feedFragment() {
        // Required empty public constructor
    }

    /**
     * Use this factory method to create a new instance of
     * this fragment using the provided parameters.
     *
     * @param param1 Parameter 1.
     * @param param2 Parameter 2.
     * @return A new instance of fragment LeftFragment.
     */
    // TODO: Rename and change types and number of parameters
    public static feedFragment newInstance(String param1, String param2) {
        feedFragment fragment = new feedFragment();
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
        return inflater.inflate(R.layout.fragment_feed, container, false);
    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);

        // 获取 SharedViewModel 实例
        sharedView = new ViewModelProvider(requireActivity()).get(SharedView.class);
        shareviewmodel = new ViewModelProvider(requireActivity()).get(SharedViewModel.class);
        ViewInitfeed(view);
        /**********ListView***********/
        ListViewInit(view);
        /**********ListView***********/

        // 3. 设置数据观察者（必须先于数据加载）
        setupDataObserver();

        //获取数据库中的数据
        loadInitialData();

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

    private void setupDataObserver() {
        shareviewmodel.getDataList().observe(getViewLifecycleOwner(), items -> {
            if (items == null) return;

            if (adapter == null) {
                adapter = new TimeWeightAdapter(requireContext(), items);
                listView.setAdapter(adapter);
            } else {
                adapter.updateData(items);
            }
        });
    }


    // 添加静态方法获取当前实例,方便外部获取该实例
    public static feedFragment getCurrentInstance(FragmentManager fm) {
        return (feedFragment) fm.findFragmentByTag("feed_fragment_tag");
    }

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
        shareviewmodel.clearDataList(); // 确保SharedViewModel有clearDataList()方法
    }

    /**
     *  从数据库加载数据,这个方法和loadInitialData是一样的
     */
    private void loadDataFromDatabase() {
        new Thread(() -> {
            Log.d(TAG, "获取数据库饮食记录数据数据");
            List<TimeWeightItem> items = CatVitalApp.getDbHelper().getAllFoodRecords();
            requireActivity().runOnUiThread(() -> {
                shareviewmodel.setDataList(items);
            });
        }).start();
    }

    private void ListViewInit(@NonNull View view) {
        ListView listView = view.findViewById(R.id.listfood);
        DatabaseHelper dbHelper = CatVitalApp.getDbHelper();
        // 观察数据变化
        shareviewmodel.getDataList().observe(getViewLifecycleOwner(), items -> {
            if (adapter == null) {
                adapter = new TimeWeightAdapter(requireContext(), items);
                listView.setAdapter(adapter);
            } else {
                adapter.notifyDataSetChanged();
            }
        });

        // 添加示例数据（确保ViewModel已初始化）
//        shareviewmodel.addItem(new TimeWeightItem("12:30:00", "30g"));
//        shareviewmodel.addItem(new TimeWeightItem("0:30:45", "15g"));
//        shareviewmodel.addItem(new TimeWeightItem("1:05:23", "30g"));

        listView.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                adapter.setShowAllItems(!adapter.showAllItems);

                // 计算 ListView 的新高度
                int newHeight = calculateListViewHeight(listView);

                // 设置 ListView 的新高度
                ViewGroup.LayoutParams params = listView.getLayoutParams();
                params.height = newHeight;
                listView.setLayoutParams(params);

                // 请求重新布局
                cardView.requestLayout();
            }
        });
    }

    /**
     * 监听函数
     */
    private void switchprocess() {
        Log.d(TAG, "switchprocess() 方法被调用"); // 确认是否执行
        switchListener = (buttonView, isChecked) -> {

            int switchId = buttonView.getId(); // 获取当前触发 Switch 的 ID
            // 保存状态到 ViewModel
            sharedView.saveButtonState(buttonView.getId(), isChecked);
            if (switchId == R.id.food_byhand) {
                // 处理 switch1 的逻辑
                Log.d("给养粮食", "switch 状态: " + isChecked);
                feed_message_send(isChecked, "food_door");
            }
            else if (switchId == R.id.water_byhand) {
                // 处理 switch2 的逻辑
                Log.d("给养水阀", "switch 状态: " + isChecked);
                feed_message_send(isChecked, "Bump");
            }
            else if (switchId == R.id.food_water_play) {
                // 处理 switch3 的逻辑
                Log.d("玩耍状态", "switch 状态: " + isChecked);
                if(isChecked)
                {
                    // 如果顺时针被激活，强制关闭逆时针
                    setSwitchStateWithoutCallback(clockwise, true);
                    JsonObject jsonObject = new JsonObject();
                    jsonObject.addProperty("clockwise", 1);
                    String jsonPayload = new Gson().toJson(jsonObject);
                    MainActivity.aliyunClient.publishMessage(jsonPayload);
                }
                else{
                    // 如果顺时针未被激活，强制关闭逆时针
                    setSwitchStateWithoutCallback(anticlockwise, false);
                    // 如果顺时针被激活，强制关闭逆时针
                    setSwitchStateWithoutCallback(clockwise, false);
                    JsonObject jsonObject = new JsonObject();
                    jsonObject.addProperty("play", 0);
                    String jsonPayload = new Gson().toJson(jsonObject);
                    MainActivity.aliyunClient.publishMessage(jsonPayload);
                }
            }
            else if (switchId == R.id.clockwise) {
                // 处理 switch3 的逻辑
                Log.d("自动喂食开关", "switch 状态: " + isChecked);
                if (isChecked) {
                    // 如果顺时针被激活，强制关闭逆时针
                    setSwitchStateWithoutCallback(anticlockwise, false);
                    setSwitchStateWithoutCallback(food_water_play, true);
                    JsonObject jsonObject = new JsonObject();
                    jsonObject.addProperty("clockwise", 1);
                    String jsonPayload = new Gson().toJson(jsonObject);
                    MainActivity.aliyunClient.publishMessage(jsonPayload);
                }else{
                    // 只有当两个都关闭时才发送关闭消息
                    if (!clockwise.isChecked() && !anticlockwise.isChecked()) {
                        setSwitchStateWithoutCallback(food_water_play, false);
                        JsonObject jsonObject = new JsonObject();
                        jsonObject.addProperty("play", 0);
                        String jsonPayload = new Gson().toJson(jsonObject);
                        MainActivity.aliyunClient.publishMessage(jsonPayload);
                    }
                }
            }
            else if (switchId == R.id.anticlockwise) {
                // 处理 switch3 的逻辑
                Log.d("顺时针", "switch 状态: " + isChecked);
                if (isChecked) {
                    // 如果逆时针被激活，强制关闭顺时针
                    setSwitchStateWithoutCallback(clockwise, false);
                    setSwitchStateWithoutCallback(food_water_play, true);
                    JsonObject jsonObject = new JsonObject();
                    jsonObject.addProperty("clockwise", 0);
                    String jsonPayload = new Gson().toJson(jsonObject);
                    MainActivity.aliyunClient.publishMessage(jsonPayload);
                }else{
                    // 只有当两个都关闭时才发送关闭消息
                    if (!clockwise.isChecked() && !anticlockwise.isChecked()) {
                        setSwitchStateWithoutCallback(food_water_play, false);
                        JsonObject jsonObject = new JsonObject();
                        jsonObject.addProperty("play", 0);
                        String jsonPayload = new Gson().toJson(jsonObject);
                        MainActivity.aliyunClient.publishMessage(jsonPayload);
                    }
                }
            }
            else if (switchId == R.id.auto_food) {
                // 处理 switch3 的逻辑
                Log.d("逆时针", "switch 状态: " + isChecked);
                if(isChecked)
                {
                    //将输入的值转化为int型(转化不成功默认值就为0)
                    int hours = new SafeConverter(feedHoursInput.getText().toString()).toInt(0);
                    int minutes = new SafeConverter(feedMinutesInput.getText().toString()).toInt(0);
                    int maxweight = new SafeConverter(maxFeedAmountInput.getText().toString()).toInt(0);
                    int minutesintotal = hours * 60 + minutes;
                    //获取当前的时间
                    int[] datetime;
                    datetime = getBeijingTimeComponents();

                    //阿里云发布消息
                    JsonObject jsonObject = new JsonObject();
                    jsonObject.addProperty("auto_feed", 1);
                    jsonObject.addProperty("timeyear", datetime[0]);
                    jsonObject.addProperty("timemonth", datetime[1]);
                    jsonObject.addProperty("timedate", datetime[2]);
                    jsonObject.addProperty("timehours", datetime[3]);
                    jsonObject.addProperty("timeminutes", datetime[4]);
                    jsonObject.addProperty("timeseconds", datetime[5]);
                    jsonObject.addProperty("minutesintotal", minutesintotal);
                    jsonObject.addProperty("maxweight", maxweight);
                    String jsonPayload = new Gson().toJson(jsonObject);
                    MainActivity.aliyunClient.publishMessage(jsonPayload);

                }else{
                    JsonObject jsonObject = new JsonObject();
                    jsonObject.addProperty("auto_feed", 0);
                    String jsonPayload = new Gson().toJson(jsonObject);
                    MainActivity.aliyunClient.publishMessage(jsonPayload);
                }
            }
        };

    }

    /**
     * 封装消息
     * @param isChecked
     * @param string
     */
    private static void feed_message_send(boolean isChecked, String string) {
        if (isChecked) {
            JsonObject jsonObject = new JsonObject();
            jsonObject.addProperty(string, 1);
            String jsonPayload = new Gson().toJson(jsonObject);
            MainActivity.aliyunClient.publishMessage(jsonPayload);
        }
        else{
            JsonObject jsonObject = new JsonObject();
            jsonObject.addProperty(string, 0);
            String jsonPayload = new Gson().toJson(jsonObject);
            MainActivity.aliyunClient.publishMessage(jsonPayload);
        }
    }

    /**
     * 初始化控件
     * @param view
     */
    private void ViewInitfeed(@NonNull View view) {
        /*********card控件***************/
        cardView = view.findViewById(R.id.timefoodcard);
        /*********card控件***************/
        /*********ListView控件***************/
        listView = view.findViewById(R.id.listfood);
        /*********ListView控件***************/
        /*********switch控件*************/
        //手动放粮
        food_byhand = view.findViewById(R.id.food_byhand);

        //手动防水
        water_byhand = view.findViewById(R.id.water_byhand);

        //玩耍装置状态开关
        food_water_play = view.findViewById(R.id.food_water_play);

        //玩耍装置顺时针
        clockwise = view.findViewById(R.id.clockwise);

        //玩耍装置逆时针
        anticlockwise = view.findViewById(R.id.anticlockwise);

        //自动喂食开关
        auto_food = view.findViewById(R.id.auto_food);
        /*********switch控件*************/

        /*********EditText控件*************/
        feedHoursInput = view.findViewById(R.id.feedHoursInput);
        feedMinutesInput = view.findViewById(R.id.feedMinutesInput);
        maxFeedAmountInput = view.findViewById(R.id.maxFeedAmountInput);
        /*********EditText控件*************/

        /*********Progress控件*************/
        foodLevelProgress = view.findViewById(R.id.foodLevelProgress);
        waterLevelProgress = view.findViewById(R.id.waterLevelProgress);
        /*********Progress控件*************/

        switchprocess();

        /*********绑定控件***************/
        checkAndBind(food_byhand, "food_byhand");
        checkAndBind(water_byhand, "water_byhand");
        checkAndBind(food_water_play, "food_water_play");
        checkAndBind(clockwise, "clockwise");
        checkAndBind(anticlockwise, "anticlockwise");
        checkAndBind(auto_food, "auto_food");
        /*********绑定控件***************/

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
            // 恢复保存的状态
            boolean savedState = sharedView.getButtonState(switchView.getId());
            switchView.setChecked(savedState);
            //设置监听器
            switchView.setOnCheckedChangeListener(switchListener);

        }
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
        if (sharedView != null) {
            sharedView.saveButtonState(switchView.getId(), checked);
        }

        // 4. 恢复统一监听器
        switchView.setOnCheckedChangeListener(switchListener);
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
}
