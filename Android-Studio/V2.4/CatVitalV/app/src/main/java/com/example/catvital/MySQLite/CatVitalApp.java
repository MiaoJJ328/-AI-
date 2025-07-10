package com.example.catvital.MySQLite;

import android.app.Application;
import android.content.Context;

public class CatVitalApp extends Application {
    // 单例实例（volatile保证多线程可见性）
    private static volatile DatabaseHelper dbHelper;
    private static CatVitalApp instance;

    @Override
    public void onCreate() {
        super.onCreate();
        // 使用Application Context初始化
        instance = this;
        initializeIfNeeded(getApplicationContext());
    }


    @Override
    public void onTerminate() {
        if (dbHelper != null) {
            dbHelper.close();   //退出时自动关闭数据库
        }
        super.onTerminate();
    }

    // 添加获取Application实例的方法,方便DatabaseHelper()函数的使用
    public static CatVitalApp getInstance() {
        return instance;
    }

    public static synchronized DatabaseHelper getDbHelper() {
        if (dbHelper == null) {
            dbHelper = DatabaseHelper.getInstance(getInstance().getApplicationContext());
        }
        return dbHelper;
    }


    /**
     * 初始化数据库（线程安全）
     */
    private static void initializeIfNeeded(Context context) {
        if (dbHelper == null) {
            synchronized (CatVitalApp.class) {
                if (dbHelper == null) {
                    // 通过DatabaseHelper的单例方法获取实例
                    dbHelper = DatabaseHelper.getInstance(context.getApplicationContext());
                }
            }
        }
    }

//    /**
//     * 获取数据库帮助类实例（双重校验锁单例）
//     */
//    public static DatabaseHelper getDbHelper() {
//        if (dbHelper == null) {
//            throw new IllegalStateException("DatabaseHelper未初始化！请确保Application已启动");
//        }
//        return dbHelper;
//    }

    /**
     * 释放数据库资源（在应用退出时调用）
     */
    public static void closeDatabase() {
        if (dbHelper != null) {
            synchronized (CatVitalApp.class) {
                if (dbHelper != null) {
                    dbHelper.close();
                    dbHelper = null;
                }
            }
        }
    }
}