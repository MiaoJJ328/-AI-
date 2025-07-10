package com.example.catvital.MySQLite;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;
import android.util.Log;

import androidx.annotation.Nullable;

import com.example.catvital.ListView.TimeWeightItem;

import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.List;

public class DatabaseHelper extends SQLiteOpenHelper {
    private static final String TAG = "DatabaseHelper";

    // 数据库常量
    public static final String DB_NAME = "cat.db";  // 数据库文件名
//    public static final int DB_VERSION = 1;         // 数据库版本
    public static final int DB_VERSION = 2;

    // 表名常量
    public static final String TABLE_CAT_FOOD = "cat_food";
    public static final String TABLE_HEARTRATE = "cat_heartrate";
    public static final String TABLE_RECORD = "cat_record";
    public static final String TABLE_TEMP = "cat_temp";

    // 列名常量（示例）
    public static final String COL_FOOD_INTAKE = "food_intake";
    public static final String COL_LOAD_TIME = "loadtime";

    // 单例实例（volatile保证多线程可见性）
    private static volatile DatabaseHelper instance;

    /**
     * 获取所有food_record的记录
     * @return
     */
    public List<TimeWeightItem> getAllFoodRecords() {
        List<TimeWeightItem> items = new ArrayList<>();
        SQLiteDatabase db = this.getReadableDatabase(); // 不要在这里关闭

        try (Cursor cursor = db.query(TABLE_CAT_FOOD,
                new String[]{COL_FOOD_INTAKE, COL_LOAD_TIME},
                null, null, null, null,
                COL_LOAD_TIME + " DESC")) {

            while (cursor.moveToNext()) {
                String time = cursor.getString(cursor.getColumnIndexOrThrow(COL_LOAD_TIME));
                int intake = cursor.getInt(cursor.getColumnIndexOrThrow(COL_FOOD_INTAKE));
                items.add(new TimeWeightItem(time, intake + "g"));
            }
        } catch (Exception e) {
            Log.e(TAG, "获取食物记录失败", e);
        }
        // 注意：不再调用 db.close()，让调用者管理生命周期
        return items;
    }

    public List<TimeWeightItem> getAllCleanRecords() {
        List<TimeWeightItem> items = new ArrayList<>();
        SQLiteDatabase db = this.getReadableDatabase();

        try (Cursor cursor = db.query(TABLE_RECORD,
                new String[]{COL_LOAD_TIME},
                null, null, null, null,
                COL_LOAD_TIME + " DESC")) {

            while (cursor.moveToNext()) {
                String time = cursor.getString(cursor.getColumnIndexOrThrow(COL_LOAD_TIME));
                items.add(new TimeWeightItem(time));
            }
        } catch (Exception e) {
            Log.e(TAG, "获取排泄记录失败", e);
        }
        return items;
    }

    /**
     * 私有化构造方法（防止外部直接实例化）
     * 使用Application Context避免内存泄漏
     */
    private DatabaseHelper(Context context) {
        super(context, DB_NAME, null, DB_VERSION);
    }

    /**
     * 获取单例实例（双重校验锁实现线程安全）
     */
    public static synchronized DatabaseHelper getInstance(Context context) {
        if (instance == null) {
            instance = new DatabaseHelper(context.getApplicationContext()); // 强制使用ApplicationContext
        }
        return instance;
    }

    @Override
    public void onCreate(SQLiteDatabase db) {
        Log.d(TAG, "创建数据库表结构...");

        // 创建cat_food表（指定load_time为主键）
        String createFoodTable = "CREATE TABLE " + TABLE_CAT_FOOD + " (" +
                COL_FOOD_INTAKE + " INTEGER, " +
                COL_LOAD_TIME + " TEXT PRIMARY KEY)";  // 添加 PRIMARY KEY 约束

        // 创建cat_record表（只有loadtime一列且为主键）
        String createRecordTable = "CREATE TABLE " + TABLE_RECORD + " (" +
                COL_LOAD_TIME + " TEXT PRIMARY KEY)";
        db.execSQL(createRecordTable);
    }

    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
        Log.d(TAG, "升级数据库，旧版本：" + oldVersion + " → 新版本：" + newVersion);
        // 使用事务确保升级操作的原子性
        db.beginTransaction();
        try {
            if (oldVersion < 2) {
                // 版本1到2的升级操作：添加cat_record表
                String createRecordTable = "CREATE TABLE " + TABLE_RECORD + " (" +
                        COL_LOAD_TIME + " TEXT PRIMARY KEY)";
                db.execSQL(createRecordTable);
                Log.d(TAG, "已创建cat_record表");
            }
            db.setTransactionSuccessful();
        } finally {
            db.endTransaction();
        }
    }

    /**
     * 关闭数据库连接（可选）
     */
    public static void closeDatabase() {
        if (instance != null) {
            instance.close();
            instance = null;  // 释放单例实例
        }
    }

    /**
     * 添加食物数据
     * @param dbHelper
     * @param intake
     * @param time
     * @return
     */
    public long insertFoodRecord(DatabaseHelper dbHelper, int intake, String time) {
        SQLiteDatabase db = dbHelper.getWritableDatabase();
        ContentValues values = new ContentValues();
        values.put(DatabaseHelper.COL_FOOD_INTAKE, intake);
        values.put(DatabaseHelper.COL_LOAD_TIME, time);
        return db.insert(DatabaseHelper.TABLE_CAT_FOOD, null, values);
    }

    /**
     * 获取任意表中最大的时间戳
     * @param TABLE_NAME 要查询的表名
     * @return 最大时间戳的Timestamp对象，如果表为空或出错则返回null
     */
    public Timestamp getMaxTimestampFromTable(String TABLE_NAME) {
        SQLiteDatabase db = this.getReadableDatabase();
        Cursor cursor = null;
        Timestamp maxTimestamp = null;

        try {
            // 查询MAX(loadtime)
            String query = "SELECT MAX(" + COL_LOAD_TIME + ") FROM " + TABLE_NAME;
            cursor = db.rawQuery(query, null);

            if (cursor != null && cursor.moveToFirst() && !cursor.isNull(0)) {
                String timeString = cursor.getString(0);
                Log.d(TAG, "获取到的时间字符串: " + timeString);

                // 转换为Timestamp对象
                try {
                    maxTimestamp = Timestamp.valueOf(timeString);
                } catch (IllegalArgumentException e) {
                    Log.e(TAG, "时间格式转换失败: " + timeString, e);
                    // 尝试处理其他时间格式（如毫秒时间戳）
                    try {
                        long millis = Long.parseLong(timeString);
                        maxTimestamp = new Timestamp(millis);
                    } catch (NumberFormatException ex) {
                        Log.e(TAG, "无法解析的时间格式: " + timeString);
                    }
                }
            } else {
                Log.d(TAG, "表中没有数据或查询结果为空");
            }
        } catch (Exception e) {
            Log.e(TAG, "获取最大时间戳失败: ", e);
        } finally {
            if (cursor != null) {
                cursor.close();
            }
//            db.close();
        }
        return maxTimestamp;
    }

    /**
     * 清空某一个表
     * @param tableName
     */
    public void clearTable(String tableName) {
        SQLiteDatabase db = this.getWritableDatabase();
        db.delete(tableName, null, null); // 删除所有行
//        db.close();
    }
}