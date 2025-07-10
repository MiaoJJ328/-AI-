package com.example.catvital.fragment;

/**
 * 用于不同类型之间的转换
 */
public class SafeConverter {
    private final String source;

    public SafeConverter(String source) {
        this.source = source;
    }

    public int toInt(int defaultValue) {
        try {
            return Integer.parseInt(source.trim());
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }
}
