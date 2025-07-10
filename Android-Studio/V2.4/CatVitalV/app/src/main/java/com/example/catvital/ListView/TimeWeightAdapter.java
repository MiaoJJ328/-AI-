package com.example.catvital.ListView;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.TextView;

import com.example.catvital.R;

import java.util.List;

public class TimeWeightAdapter extends ArrayAdapter<TimeWeightItem> {
    public boolean showAllItems = false; // 控制是否显示所有项
    private List<TimeWeightItem> originalItems; // 保存原始数据顺序

    /**
     * 为适配器进行初始化设置ArrayAdapter 是 Android 框架中一个非常常用的适配器类，
     * 主要用于将 数据数组/列表 和 UI 列表视图（如 ListView、Spinner）
     * 进行绑定。它是 BaseAdapter 的子类，提供了列表数据管理的基础实现。
     * @param context
     * @param items
     */
    public TimeWeightAdapter(Context context, List<TimeWeightItem> items) {
        super(context, 0, items);   // 调用父类(ArrayAdapter)的构造方法
        this.originalItems = items; // 将传入的数据列表保存到成员变量
    }

    // TimeWeightAdapter.java
    public void updateData(List<TimeWeightItem> newItems) {
        this.originalItems.clear();
        this.originalItems.addAll(newItems);
        notifyDataSetChanged(); // 关键：通知ListView刷新
    }

    // 添加新项目到列表开头（最新数据在最前面）
    public void addNewItem(TimeWeightItem item) {
        originalItems.add(0, item); // 新数据添加到开头
        notifyDataSetChanged();
    }

    // 获取实际数据位置（反转顺序）
    private TimeWeightItem getActualItem(int position) {
        if (showAllItems) {
            return originalItems.get(position);  // 展开模式：返回全部数据
        } else {
            // 只显示最新的一项（列表第一项）
            return originalItems.get(0); // 折叠模式：始终返回最新一项（位置 0）
        }
    }

    @Override
    public int getCount() {
        // 如果不显示所有项，则只返回1个
        return showAllItems ? originalItems.size() : Math.min(originalItems.size(), 1);
    }

    @Override
    public TimeWeightItem getItem(int position) {
        return getActualItem(position);
    }

    /**
     * 负责 将数据绑定到列表项的视图上。
     * @param position The position of the item within the adapter's data set of the item whose view
     *        we want.
     * @param convertView The old view to reuse, if possible. Note: You should check that this view
     *        is non-null and of an appropriate type before using. If it is not possible to convert
     *        this view to display the correct data, this method can create a new view.
     *        Heterogeneous lists can specify their number of view types, so that this View is
     *        always of the right type (see {@link #getViewTypeCount()} and
     *        {@link #getItemViewType(int)}).
     * @param parent The parent that this view will eventually be attached to
     * @return
     */
    @Override
    public View getView(int position, View convertView, ViewGroup parent) {
        TimeWeightItem item = getActualItem(position);

        if (convertView == null) {
            convertView = LayoutInflater.from(getContext())
                    .inflate(R.layout.item_time_weight, parent, false);
        }

        TextView timeView = convertView.findViewById(R.id.timeTextView);
        TextView weightView = convertView.findViewById(R.id.weightTextView);

        timeView.setText(item.getTime());
        weightView.setText(item.getWeight());

        return convertView;
    }

    public void setShowAllItems(boolean showAll) {
        if (this.showAllItems != showAll) {
            this.showAllItems = showAll;
            notifyDataSetChanged();
        }
    }
}