package com.example.catvital;

import android.annotation.SuppressLint;
import android.content.ContentValues;
import android.database.SQLException;
import android.database.sqlite.SQLiteDatabase;
import android.graphics.Color;
import android.icu.util.Calendar;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.View;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import androidx.activity.EdgeToEdge;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;
import androidx.fragment.app.FragmentManager;
import androidx.fragment.app.FragmentTransaction;
import androidx.lifecycle.ViewModelProvider;

import com.example.catvital.MySQLite.CatVitalApp;
import com.example.catvital.MySQLite.DatabaseHelper;
import com.example.catvital.MySQLite.MySQLConnections;
import com.example.catvital.SharedViewModel.SharedViewModel;
import com.example.catvital.aliyun.aliyun;
import com.example.catvital.fragment.BeijingTimeUtils;
import com.example.catvital.fragment.aiFragment;
import com.example.catvital.fragment.cleanFragment;
import com.example.catvital.fragment.feedFragment;
import com.example.catvital.fragment.healthFragment;
import com.github.mikephil.charting.charts.BarChart;
import com.github.mikephil.charting.components.XAxis;
import com.github.mikephil.charting.data.BarData;
import com.github.mikephil.charting.data.BarDataSet;
import com.github.mikephil.charting.data.BarEntry;
import com.github.mikephil.charting.data.Entry;
import com.github.mikephil.charting.formatter.IndexAxisValueFormatter;
import com.github.mikephil.charting.highlight.Highlight;
import com.github.mikephil.charting.listener.OnChartValueSelectedListener;

import java.sql.Connection;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.ResultSetMetaData;
import java.sql.Timestamp;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public class MainActivity extends AppCompatActivity implements View.OnClickListener{

    private LinearLayout llai, llfeed, llhealth, llclean;
    private ImageView ivai, ivfeed, ivhealth, ivclean;
    private TextView tvai, tvfeed, tvhealth, tvclean;
    private FragmentManager fragmentManager;
    private FragmentTransaction fragmentTransaction;
    private SharedViewModel sharedViewModel;
    public static aliyun aliyunClient;
    private static final String TAG = "MainActivity";
    public MySQLConnections dbConnector;
    //定时执行数据库操作
    private Handler handler = new Handler();
    private Runnable mySqlRunnable;
    public int appbehindlast = 0;

    private static PreparedStatement stmt = null; //STATEMENT

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_main);

        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main), (v, insets) -> {
            Insets systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars());
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom);
            return insets;

        });
        // 初始化 ViewModel
        ViewModelInit();
        //初始化阿里云
        aliyunInit();
        InitView();
        InitEvent();
        //初始化阿里云数据库
        alimysqlInit();

        // 数据库的定时更新
        mySqlRunnable = new Runnable() {
            @Override
            public void run() {
                MySqlThread();
                // 每隔2分钟(120000毫秒)执行一次
                handler.postDelayed(this, 120000);
            }
        };

    }


    @Override
    protected void onStart() {
        super.onStart();
        // 应用启动时立即执行一次
        handler.post(mySqlRunnable);
    }


    @Override
    protected void onStop() {
        super.onStop();
        // 避免内存泄漏，移除回调
        handler.removeCallbacks(mySqlRunnable);
    }

    // 当需要更新数据库时调用这个方法
    public void notifyFeedFragmentUpdate() {
        // 获取FragmentManager
        FragmentManager fragmentManager = getSupportFragmentManager();

        // 通过tag找到你的feedFragment实例
        feedFragment fragment = (feedFragment) fragmentManager.findFragmentByTag("Feed");

        // 如果找到了Fragment且不为null，调用更新方法
        if (fragment != null) {
            fragment.onDatabaseUpdated();
        }
    }


    /*********************************************数据库部分*********************************************/

    /**
     * 数据库同步线程
     */
    private void MySqlThread() {
        new Thread(() -> {
            // 每个线程独立管理自己的数据库连接
            SQLiteDatabase db = null;
            Connection conn = null;
            PreparedStatement stmt = null;
            ResultSet rs = null;

            try {
                // 1. 初始化时间数据
                Timestamp appnowstamp = BeijingTimeUtils.getBeijingTimestampToSecond();
                Log.d(TAG, "当前时间" + appnowstamp);

                DatabaseHelper dbHelper = CatVitalApp.getDbHelper();
                Timestamp foodstamp = dbHelper.getMaxTimestampFromTable("cat_food");
                Log.d(TAG, "SQLite最晚时间" + foodstamp);
                // 2. 获取MySQL最后插入时间
                conn = dbConnector.getConnection();
                String sql = "SELECT MAX(loadtime) AS last_insert_time FROM cat_food_record";
                stmt = conn.prepareStatement(sql);
                rs = stmt.executeQuery();

                Timestamp lastInsertTime = rs.next() ? rs.getTimestamp("last_insert_time") : null;
                if (lastInsertTime == null) {
                    Log.d(TAG, "表中没有数据");
                    safeCloseResources(rs, stmt, conn);
                    return;
                }
                Log.d(TAG, "最后插入时间: " + lastInsertTime);

                // 3. 获取SQLite可写数据库（开启事务）
                db = dbHelper.getWritableDatabase();
                db.beginTransaction();

                // 4. 判断同步条件(是不是当天的数据)
                boolean shouldSync = false;
                int selection = 0;
                if(foodstamp != null)
                {
                    int istoday = appnowstamp.compareTo(foodstamp);
                    Log.d(TAG, "SQLite数据库当中数据非空");
                    if(istoday > 0 && ((appnowstamp.getMonth() > foodstamp.getMonth()) || (appnowstamp.getDate() > foodstamp.getDate())))
                    {
                        Log.d(TAG, "SQLite数据库当中存储的NOT当天的数据");
                        //1.清空SQLite数据库
                        Log.d(TAG, "清空SQLite数据库");
                        dbHelper.clearTable(DatabaseHelper.TABLE_CAT_FOOD);
                        runOnUiThread(this::notifyFeedFragmentUpdate); // 清空后立即通知(用于同时FeedFragment当中的ListView)
                        //2.数据库是否有当天数据
                        if(appnowstamp.getDate() == lastInsertTime.getDate())
                        {
                            Log.d(TAG, "数据库有当天的数据");
                            //3.将和appnowstamp所处date相同数据同步进来
                            shouldSync = true;
                            selection = 1;  //syncDataToSQLite执行此种同步1
                        }
                    }
                    else
                    {
                        Log.d(TAG, "SQLite数据库当中存储YES当天的数据");
                        //将从位于foodstamp和lastInsertTime之间的数据插入数据库即可不再进行清空
                        int cmp = foodstamp.compareTo(lastInsertTime);
                        if(cmp == 0)
                        {
                            shouldSync = false;
                            Log.d(TAG, "SQLite数据库已为当天最新数据");
                        }
                        else
                        {
                            shouldSync = true;
                            selection = 2;  //syncDataToSQLite执行此种同步2
                        }

                    }

                }
                else
                {
                    Log.d(TAG, "SQLite数据库为空");
                    //2.数据库是否有当天数据
                    if(appnowstamp.getDate() == lastInsertTime.getDate())
                    {
                        Log.d(TAG, "数据库有当天的数据");
                        //3.将和appnowstamp所处date相同数据同步进来
                        shouldSync = true;
                        selection = 1;  //syncDataToSQLite执行此种同步1
                    }
                }

                // 记录比较结果
                if (shouldSync) {
                    Log.d(TAG, "本地数据比服务器旧");
                } else if (!shouldSync) {
                    Log.d(TAG, "数据已同步");
                }

                // 5. 执行数据同步
                // 修改这部分调用代码
                if (shouldSync) {
                    if (selection == 1) {
                        // 同步当天全部数据
                        syncDataToSQLite(db, conn, appnowstamp, 1);
                    } else {
                        // 同步增量数据（需要传入foodstamp）
                        syncDataToSQLite(db, conn, foodstamp, 2); // 注意这里传入的是foodstamp
                    }
                }

                // 6. 标记事务成功
                db.setTransactionSuccessful();
                Log.d(TAG, "操作完成");

            } catch (Exception e) {
                Log.e(TAG, "数据库操作失败", e);
            } finally {
                // 7. 资源释放（严格按照创建的反序关闭）
                try {
                    if (db != null) {
                        try {
                            db.endTransaction(); // 确保结束事务
                        } finally {
//                            db.close(); // 最后关闭SQLite连接
                        }
                    }
                } catch (Exception e) {
                    Log.e(TAG, "关闭SQLite数据库失败", e);
                }

                try {
                    if (rs != null) rs.close();
                } catch (Exception e) {
                    Log.e(TAG, "关闭ResultSet失败", e);
                }

                try {
                    if (stmt != null) stmt.close();
                } catch (Exception e) {
                    Log.e(TAG, "关闭Statement失败", e);
                }

                try {
                    if (conn != null) conn.close();
                } catch (Exception e) {
                    Log.e(TAG, "关闭MySQL连接失败", e);
                }
            }
        }).start();
    }


    /**
     * 同步数据到SQLite（事务内操作）
     * @param db SQLite数据库实例
     * @param mysqlConn MySQL连接
     * @param date 参考日期
     * @param selection 同步模式：1-同步当天全部数据，2-同步增量数据
     */
    private void syncDataToSQLite(SQLiteDatabase db, Connection mysqlConn,
                                  Timestamp date, int selection) throws SQLException, java.sql.SQLException {
        Log.d(TAG, "开始同步数据，模式：" + selection);

        PreparedStatement stmt = null;
        ResultSet rs = null;
        ContentValues values = new ContentValues();
        int count = 0;

        try {
            // 根据选择模式构建不同的SQL查询
            String sql;
            if (selection == 1) {
                // 模式1：同步当天全部数据
                sql = "SELECT * FROM cat_food_record WHERE DATE(loadtime) = DATE(?)";
                stmt = mysqlConn.prepareStatement(sql);
                stmt.setTimestamp(1, date);
            } else {
                // 模式2：同步增量数据（大于foodstamp的数据）
                sql = "SELECT * FROM cat_food_record WHERE loadtime > ? AND DATE(loadtime) = DATE(?)";
                stmt = mysqlConn.prepareStatement(sql);
                stmt.setTimestamp(1, date); // 这里date实际应该是foodstamp，需要调整调用处
                stmt.setTimestamp(2, date);
            }

            rs = stmt.executeQuery();

            // 批量插入数据
            while (rs.next()) {
                values.clear();
                Timestamp mysqlTime = rs.getTimestamp("loadtime");
                String isoTime = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault())
                        .format(mysqlTime);

                values.put(DatabaseHelper.COL_FOOD_INTAKE, rs.getInt("food_intake"));
                values.put(DatabaseHelper.COL_LOAD_TIME, isoTime);

                db.insert(DatabaseHelper.TABLE_CAT_FOOD, null, values);
                count++;
                // 通知Fragment更新（在主线程执行）
                runOnUiThread(this::notifyFeedFragmentUpdate);  //数据库添加一条数据，就将其更新到ListView
            }

            Log.d(TAG, "同步完成，共插入 " + count + " 条记录");
        } finally {
            // 关闭资源
            if (rs != null) try { rs.close(); } catch (SQLException e) { Log.w(TAG, "关闭ResultSet失败", e); }
            if (stmt != null) try { stmt.close(); } catch (SQLException e) { Log.w(TAG, "关闭Statement失败", e); }
        }
    }

    /*********************************************数据库部分*********************************************/

    /**
     * 初始化阿里云数据库
     */
    private void alimysqlInit() {
        //初始化数据库
        DataBaseInit();
        // 创建数据库连接实例
        dbConnector = MySQLConnections.getInstance();
    }

    /**
     * 用于安全的return，清理掉资源
     * @param rs
     * @param stmt
     * @param conn
     */
    private void safeCloseResources(ResultSet rs, PreparedStatement stmt, Connection conn) {
        try {
            if (rs != null) rs.close();
        } catch (Exception e) {
            Log.e(TAG, "关闭ResultSet失败", e);
        }
        try {
            if (stmt != null) stmt.close();
        } catch (Exception e) {
            Log.e(TAG, "关闭Statement失败", e);
        }
        try {
            if (conn != null) conn.close();
        } catch (Exception e) {
            Log.e(TAG, "关闭Connection失败", e);
        }
    }
    /*********************************************数据库部分*********************************************/

    /**
     * 初始化SQLite数据库
     */
    private static void DataBaseInit() {
        // 获取数据库实例
        DatabaseHelper dbHelper = CatVitalApp.getDbHelper();
        // 插入数据
//        dbHelper.insertFoodRecord(dbHelper, 50, "2025-4-13 12:00:00");
//        dbHelper.insertFoodRecord(dbHelper, 50, "2025-4-13 12:00:04");
        //dbHelper.clearTable(DatabaseHelper.TABLE_CAT_FOOD);
    }


    private void ViewModelInit() {
        sharedViewModel = new ViewModelProvider(this).get(SharedViewModel.class);
        // 监听消息
        sharedViewModel.getMainActivity().observe(this, message -> {
            Log.d("MainActivity", "收到消息: " + message);
            switch (message) {
                case "阿里云连接成功":
                    Toast.makeText(MainActivity.this, "连接成功", Toast.LENGTH_SHORT).show();
                    aliyunClient.subscribeTopic(aliyun.SUB_TOPIC);
                    aliyunClient.publishMessage("hello");
                    break;
                case "阿里云连接失败":
                    Toast.makeText(MainActivity.this, "连接失败", Toast.LENGTH_SHORT).show();
            }
        });
    }


    private void aliyunInit() {
        aliyunClient = new aliyun(this);
        aliyunClient.mqtt_init();
    }

    private void InitEvent() {
        //添加fragment
        fragmentManager = getSupportFragmentManager();
        fragmentTransaction = fragmentManager.beginTransaction();
        aiFragment fragment = aiFragment.newInstance("这是ai页面", "");
        fragmentTransaction.replace(R.id.fragment_container, fragment).commit();

        ivai.setSelected(true);

        llfeed.setOnClickListener(this);
        llai.setOnClickListener(this);
        llhealth.setOnClickListener(this);
        llclean.setOnClickListener(this);

    }

    private void InitView() {
        llai = findViewById(R.id.llai);
        llfeed = findViewById(R.id.llfeed);
        llhealth = findViewById(R.id.llhealth);
        llclean = findViewById(R.id.llclean);

        ivai = findViewById(R.id.iv_ai);
        ivfeed = findViewById(R.id.iv_feed);
        ivhealth = findViewById(R.id.iv_health);
        ivclean = findViewById(R.id.iv_clean);

        tvai = findViewById(R.id.tv_ai);
        tvfeed = findViewById(R.id.tv_feed);
        tvhealth = findViewById(R.id.tv_health);
        tvclean = findViewById(R.id.tv_clean);
    }

    @Override
    public void onClick(View v) {
        int id = v.getId();
        ResetBottomState();
        if(id == R.id.llai)
        {
            //添加fragment
            fragmentManager = getSupportFragmentManager();
            fragmentTransaction = fragmentManager.beginTransaction();
            aiFragment fragment = aiFragment.newInstance("这是ai页面", "");
            fragmentTransaction.replace(R.id.fragment_container, fragment).commit();

            ivai.setSelected(true);
//            tvai.setTextColor(getResources().getColor(R.color.gray));
        }
        else if (id == R.id.llfeed)
        {
            //添加fragment
            fragmentManager = getSupportFragmentManager();
            fragmentTransaction = fragmentManager.beginTransaction();
            feedFragment fragment = feedFragment.newInstance("这是feed界面", "");
            fragmentTransaction.replace(R.id.fragment_container, fragment, "Feed").commit();

            ivfeed.setSelected(true);
        }
        else if (id == R.id.llhealth)
        {
            //添加fragment
            fragmentManager = getSupportFragmentManager();
            fragmentTransaction = fragmentManager.beginTransaction();
            healthFragment fragment = healthFragment.newInstance("这是health界面", "");
            fragmentTransaction.replace(R.id.fragment_container, fragment).commit();

            ivhealth.setSelected(true);
        }
        else if (id == R.id.llclean)
        {
            //添加fragment
            fragmentManager = getSupportFragmentManager();
            fragmentTransaction = fragmentManager.beginTransaction();
            cleanFragment fragment = cleanFragment.newInstance("这是clean界面", "");
            fragmentTransaction.replace(R.id.fragment_container, fragment).commit();

            ivclean.setSelected(true);
        }
    }

    private void ResetBottomState() {
        ivai.setSelected(false);
        ivfeed.setSelected(false);
        ivhealth.setSelected(false);
        ivclean.setSelected(false);
    }


}