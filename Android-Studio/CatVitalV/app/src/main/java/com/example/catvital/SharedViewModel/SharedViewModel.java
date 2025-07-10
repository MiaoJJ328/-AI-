package com.example.catvital.SharedViewModel;

import androidx.lifecycle.LiveData;
import androidx.lifecycle.MutableLiveData;
import androidx.lifecycle.ViewModel;
import com.example.catvital.ListView.TimeWeightItem;
import java.util.ArrayList;
import java.util.List;

/**
 * SharedViewModel 用于在多个 Fragment 或 Activity 之间共享数据，
 * 并确保数据在配置更改（如屏幕旋转）时不会丢失。
 */
public class SharedViewModel extends ViewModel {

    /*****************健康数据********************/
    private List<Float> heartRateData = new ArrayList<>();
    private List<Float> oxygenData = new ArrayList<>();
    private List<Float> temperatureData = new ArrayList<>();

    public void addHeartRateData(float value) {
        heartRateData.add(value);
    }

    public void addOxygenData(float value) {
        oxygenData.add(value);
    }

    public void addTemperatureData(float value) {
        temperatureData.add(value);
    }

    public List<Float> getHeartRateData() {
        return heartRateData;
    }

    public List<Float> getOxygenData() {
        return oxygenData;
    }

    public List<Float> getTemperatureData() {
        return temperatureData;
    }

    public float getLastHeartRate() {
        return heartRateData.isEmpty() ? 0 : heartRateData.get(heartRateData.size() - 1);
    }

    public float getLastOxygen() {
        return oxygenData.isEmpty() ? 0 : oxygenData.get(oxygenData.size() - 1);
    }

    public float getLastTemperature() {
        return temperatureData.isEmpty() ? 0 : temperatureData.get(temperatureData.size() - 1);
    }

    /*****************健康数据********************/

    // ---------- 饮食记录相关数据 ----------
    // 使用 MutableLiveData 存储 TimeWeightItem 列表，支持动态更新
    private MutableLiveData<List<TimeWeightItem>> dataList = new MutableLiveData<>();

    // 构造函数：初始化空列表
    public SharedViewModel() {
        dataList.setValue(new ArrayList<>());
    }

    // 获取数据列表的 LiveData（外部只能观察，不能修改）
    public LiveData<List<TimeWeightItem>> getDataList() {
        return dataList;
    }

    // 添加单个条目到列表开头（用于新增数据）
    public void addItem(TimeWeightItem item) {
        List<TimeWeightItem> current = new ArrayList<>(dataList.getValue());
        current.add(0, item); // 新数据添加到开头
        dataList.setValue(current);
    }

    // 设置整个数据列表（用于批量更新）
    public void setDataList(List<TimeWeightItem> items) {
        dataList.setValue(items);
    }

    // 清空数据列表
    public void clearDataList() {
        dataList.setValue(new ArrayList<>());
    }

    // ---------- 跨组件通信的 LiveData ----------
    // 从 Aliyun → MainActivity 的消息通道
    private final MutableLiveData<String> mactivitymain = new MutableLiveData<>();

    // 从 MainActivity → Fragment 的消息通道
    private final MutableLiveData<String> activityToFragmentMsg = new MutableLiveData<>();

    // 为不同 Fragment 独立准备的数据通道
    private final MutableLiveData<String> dataForFeedFragment = new MutableLiveData<>();
    private final MutableLiveData<String> dataForCleanFragment = new MutableLiveData<>();
    private final MutableLiveData<String> dataForHealthFragment = new MutableLiveData<>();
    private final MutableLiveData<String> dataForAIFragment = new MutableLiveData<>();

    // ---------- 对外暴露只读 LiveData ----------
    public LiveData<String> getMainActivity() {
        return mactivitymain;
    }

    public LiveData<String> getActivityToFragmentMsg() {
        return activityToFragmentMsg;
    }

    public LiveData<String> getFeedFragmentMsg() {
        return dataForFeedFragment;
    }

    public LiveData<String> getCleanFragmentMsg() {
        return dataForCleanFragment;
    }

    public LiveData<String> getHealthFragmentMsg() {
        return dataForHealthFragment;
    }

    public LiveData<String> getAIFragmentMsg() {
        return dataForAIFragment;
    }

    // ---------- 更新数据的方法（支持线程安全） ----------
    // 向 MainActivity 发送消息（子线程安全）
    public void sendToActivityMain(String message) {
        mactivitymain.postValue(message); // postValue 可在子线程调用
    }

    // 向通用 Fragment 通道发送消息（主线程调用）
    public void sendToFragment(String message) {
        activityToFragmentMsg.postValue(message);
    }

    // 向各独立 Fragment 发送消息（子线程安全）
    public void sendToCleanFragment(String message) {
        dataForCleanFragment.postValue(message);
    }

    public void sendToFeedFragment(String message) {
        dataForFeedFragment.postValue(message);
    }

    public void sendToHealthFragment(String message) {
        dataForHealthFragment.postValue(message);
    }

    // 向 AI Fragment 发送消息（主线程调用）
    public void sendToAIFragment(String message) {
        dataForAIFragment.postValue(message);
    }
}