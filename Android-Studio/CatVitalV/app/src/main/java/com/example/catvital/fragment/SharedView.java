package com.example.catvital.fragment;

import androidx.lifecycle.ViewModel;
import java.util.HashMap;
import java.util.Map;

/**
 * 用于切换不同的fragment的之后控件的状态不变
 */
public class SharedView extends ViewModel {
    private final Map<Integer, Boolean> buttonStates = new HashMap<>();
    private final Map<Integer, Integer> seekBarValues = new HashMap<>(); // 新增SeekBar值存储
    public void saveButtonState(int buttonId, boolean isChecked) {
        buttonStates.put(buttonId, isChecked);
    }

    public boolean getButtonState(int buttonId) {
        return buttonStates.containsKey(buttonId) ? buttonStates.get(buttonId) : false;
    }


    // 新增SeekBar相关方法
    public void saveSeekBarValue(int seekBarId, int progress) {
        seekBarValues.put(seekBarId, progress);
    }

    public int getSeekBarValue(int seekBarId) {
        return seekBarValues.containsKey(seekBarId) ? seekBarValues.get(seekBarId) : 0;
    }

}
