package com.example.catvital.SharedPreferences;

import android.content.Context;
import android.content.SharedPreferences;

import android.content.Context;
import android.content.SharedPreferences;


/**
 * 永久存储数据，及时APP退出也不会变为初始值
 * AppPreferences.saveInt(context, "login_count", 5);存储数据
 * int loginCount = AppPreferences.getInt(context, "login_count", 0); 获取数据
 */
public class AppPreferences {
    private static final String PREFS_NAME = "MyAppPrefs";

    // 保存String类型
    public static void saveData(Context context, String key, String value) {
        getSharedPreferences(context).edit().putString(key, value).apply();
    }

    // 获取String类型
    public static String getData(Context context, String key, String defaultValue) {
        return getSharedPreferences(context).getString(key, defaultValue);
    }

    // 保存int类型
    public static void saveInt(Context context, String key, int value) {
        getSharedPreferences(context).edit().putInt(key, value).apply();
    }

    // 获取int类型
    public static int getInt(Context context, String key, int defaultValue) {
        return getSharedPreferences(context).getInt(key, defaultValue);
    }

    // 保存boolean类型
    public static void saveBoolean(Context context, String key, boolean value) {
        getSharedPreferences(context).edit().putBoolean(key, value).apply();
    }

    // 获取boolean类型
    public static boolean getBoolean(Context context, String key, boolean defaultValue) {
        return getSharedPreferences(context).getBoolean(key, defaultValue);
    }

    // 保存float类型
    public static void saveFloat(Context context, String key, float value) {
        getSharedPreferences(context).edit().putFloat(key, value).apply();
    }

    // 获取float类型
    public static float getFloat(Context context, String key, float defaultValue) {
        return getSharedPreferences(context).getFloat(key, defaultValue);
    }

    // 保存long类型
    public static void saveLong(Context context, String key, long value) {
        getSharedPreferences(context).edit().putLong(key, value).apply();
    }

    // 获取long类型
    public static long getLong(Context context, String key, long defaultValue) {
        return getSharedPreferences(context).getLong(key, defaultValue);
    }

    // 移除指定键
    public static void remove(Context context, String key) {
        getSharedPreferences(context).edit().remove(key).apply();
    }

    // 清空所有数据
    public static void clearAll(Context context) {
        getSharedPreferences(context).edit().clear().apply();
    }

    // 检查是否包含某个键
    public static boolean contains(Context context, String key) {
        return getSharedPreferences(context).contains(key);
    }

    private static SharedPreferences getSharedPreferences(Context context) {
        return context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
    }
}
