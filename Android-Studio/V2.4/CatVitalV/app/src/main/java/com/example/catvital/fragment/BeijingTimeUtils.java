package com.example.catvital.fragment;

import java.sql.Date;
import java.sql.Timestamp;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Locale;
import java.util.TimeZone;
/**
 * 获取北京的时间
 */
public class BeijingTimeUtils {

    public static Calendar calendar;

    /**
     * 获取当前北京时间的Calendar实例
     */
    public static Calendar getBeijingCalendar() {
        return Calendar.getInstance(TimeZone.getTimeZone("Asia/Shanghai"));
    }

    /**
     * 获取当前北京时间的各个时间分量（整数形式）
     *
     * @return
     */
    public static int[] getBeijingTimeComponents() {
        // 获取北京时区的Calendar实例
//        Calendar calendar = Calendar.getInstance(TimeZone.getTimeZone("Asia/Shanghai"));
        Calendar calendar = getBeijingCalendar();

        return new int[] {
                calendar.get(Calendar.YEAR),
                calendar.get(Calendar.MONTH) + 1,
                calendar.get(Calendar.DAY_OF_MONTH),
                calendar.get(Calendar.HOUR_OF_DAY),
                calendar.get(Calendar.MINUTE),
                calendar.get(Calendar.SECOND)
        };
    }

    /**
     * 获取当前北京时间的Date对象
     */
    public static Date getBeijingDate() {
        return (Date) getBeijingCalendar().getTime();
    }

    /**
     * 获取当前北京时间的字符串表示（ISO8601格式）
     */
    public static String getBeijingTimeString() {
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.CHINA);
        sdf.setTimeZone(TimeZone.getTimeZone("Asia/Shanghai"));
        return sdf.format(getBeijingDate());
    }

    /**
     * 将数据库时间字符串转换为Date对象
     */
    public static Date parseDatabaseTime(String dbTimeString) throws Exception {
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.CHINA);
        return (Date) sdf.parse(dbTimeString);
    }

    /**
     * 获取当前北京时间的Timestamp对象
     */
    public static Timestamp getBeijingTimestamp() {
        return new Timestamp(getBeijingCalendar().getTimeInMillis());
    }

    /**
     * 获取当前北京时间的Timestamp对象（精确到秒）
     */
    public static Timestamp getBeijingTimestampToSecond() {
        Calendar calendar = getBeijingCalendar();
        // 将毫秒和纳秒部分清零
        Timestamp timestamp = new Timestamp(calendar.getTimeInMillis());
        timestamp.setNanos(0);  // 清除纳秒部分
        return timestamp;
    }

    /**
     * 比较两个时间（系统当前时间与数据库时间）
     * @param dbTimeString 数据库时间字符串
     * @return 比较结果：-1(数据库时间早于当前时间), 0(相同), 1(数据库时间晚于当前时间)
     */
    public static int compareWithCurrentTime(String dbTimeString) throws Exception {
        Date dbTime = parseDatabaseTime(dbTimeString);
        Date currentTime = getBeijingDate();
        return dbTime.compareTo(currentTime);
    }
}
