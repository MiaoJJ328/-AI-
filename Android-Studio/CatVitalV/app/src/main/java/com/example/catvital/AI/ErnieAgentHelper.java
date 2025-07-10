package com.example.catvital.AI;

import android.content.Context;
import android.content.SharedPreferences;

import com.google.gson.Gson;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

import okhttp3.Call;
import okhttp3.Callback;
import okhttp3.MediaType;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.RequestBody;
import okhttp3.Response;

//{
//        "access_token": "24.5bbd347114f250b078aa7a364471514a.2592000.1746199169.282335-118312096",
//        "expires_in": 2592000,
//        "refresh_token": "25.c5b09b51f8c5ac14b619beac3a697f64.315360000.2058967169.282335-118312096"
//        }

public class ErnieAgentHelper {
    private static final String TAG = "ErnieAgentHelper";
    private static final MediaType JSON_TYPE = MediaType.parse("application/json; charset=utf-8");
    private static final String TOKEN_URL = "https://openapi.baidu.com/oauth/2.0/token";
    private static final String API_URL = "https://agentapi.baidu.com/assistant/getAnswer";
    private static final String PREFS_NAME = "ErniePrefs";

    private final OkHttpClient client;
    private final Gson gson;
    private final SharedPreferences prefs;
    private final String appId;
    private final String secretKey;
    private String currentThreadId;

    public ErnieAgentHelper(Context context, String appId, String secretKey) {

        this.client = new OkHttpClient.Builder()
                .connectTimeout(15, TimeUnit.SECONDS)
                .readTimeout(30, TimeUnit.SECONDS)
                .build();

        this.gson = new Gson();
        this.prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        this.appId = appId;
        this.secretKey = secretKey;
    }

    public interface ErnieCallback {
        void onSuccess(String response, String threadId);
        void onFailure(int errorCode, String errorMsg);
    }

    // region 核心方法
    public void queryAgent(String query, String openId, ErnieCallback callback) {
        new Thread(() -> {
            try {
                String accessToken = getValidAccessToken();
                JSONObject requestBody = buildRequestBody(query, openId);

                Request request = new Request.Builder()
                        .url(buildApiUrl(accessToken))
                        .post(RequestBody.create(requestBody.toString(), JSON_TYPE))
                        .build();

                client.newCall(request).enqueue(new Callback() {
                    @Override
                    public void onFailure(Call call, IOException e) {
                        callback.onFailure(1001, "网络请求失败: " + e.getMessage());
                    }

                    @Override
                    public void onResponse(Call call, Response response) throws IOException {
                        handleApiResponse(response, callback);
                    }
                });

            } catch (Exception e) {
                callback.onFailure(1000, "请求构造失败: " + e.getMessage());
            }
        }).start();
    }
    // endregion

    // region 私有方法
    private String getValidAccessToken() throws Exception {
        String cachedToken = prefs.getString("access_token", null);
        long expiresAt = prefs.getLong("expires_at", 0);

        if (cachedToken != null && System.currentTimeMillis() < expiresAt) {
            return cachedToken;
        }
        return refreshAccessToken();
    }

    private String refreshAccessToken() throws Exception {
        String url = TOKEN_URL + "?grant_type=client_credentials"
                + "&client_id=" + appId
                + "&client_secret=" + secretKey
                + "&scope=smartapp_snsapi_base";

        Request request = new Request.Builder().url(url).build();

        try (Response response = client.newCall(request).execute()) {
            String jsonData = response.body().string();
            JSONObject json = new JSONObject(jsonData);

            if (json.has("access_token")) {
                String token = json.getString("access_token");
                long expiresIn = json.getLong("expires_in") * 1000;

                prefs.edit()
                        .putString("access_token", token)
                        .putLong("expires_at", System.currentTimeMillis() + expiresIn - 60000) // 提前1分钟过期
                        .apply();

                return token;
            } else {
                throw new Exception(json.optString("error", "未知错误"));
            }
        }
    }

    private JSONObject buildRequestBody(String query, String openId) throws JSONException {
        JSONObject body = new JSONObject();

        // 会话上下文
        if (currentThreadId != null) {
            body.put("threadId", currentThreadId);
        }

        // 消息结构
        JSONObject message = new JSONObject();
        JSONObject content = new JSONObject();
        content.put("type", "text");

        JSONObject value = new JSONObject();
        value.put("showText", query);
        content.put("value", value);

        message.put("content", content);
        body.put("message", message);

        // 固定参数
        body.put("source", appId);
        body.put("from", "openapi");
        body.put("openId", openId);

        return body;
    }

    private String buildApiUrl(String accessToken) {
        return API_URL
                + "?appId=" + appId
                + "&secretKey=" + secretKey
                + "&access_token=" + accessToken;
    }

    private void handleApiResponse(Response response, ErnieCallback callback) {
        try {
            String jsonData = response.body().string();
            JSONObject json = new JSONObject(jsonData);

            int status = json.getInt("status");
            if (status != 0) {
                callback.onFailure(status, json.optString("message", "未知错误"));
                return;
            }

            JSONObject data = json.getJSONObject("data");
            JSONArray contentArray = data.getJSONArray("content");
            StringBuilder result = new StringBuilder();

            for (int i = 0; i < contentArray.length(); i++) {
                JSONObject item = contentArray.getJSONObject(i);
                String dataType = item.getString("dataType");

                // 关键修改：兼容data字段的两种类型（String 或 JSONObject）
                if ("txt".equals(dataType)) {
                    Object dataContent = item.get("data");
                    if (dataContent instanceof String) {
                        // 直接获取字符串内容
                        result.append((String) dataContent);
                    } else if (dataContent instanceof JSONObject) {
                        // 兼容旧版结构（如存在text字段）
                        result.append(((JSONObject) dataContent).optString("text", ""));
                    }
                }
            }

            if (result.length() > 0) {
                callback.onSuccess(result.toString(), data.getString("threadId"));
            } else {
                callback.onFailure(1003, "响应内容为空");
            }

        } catch (Exception e) {
            callback.onFailure(1002, "响应解析失败: " + e.getMessage());
        }
    }
//    private void handleApiResponse(Response response, ErnieCallback callback) {
//        try {
//            String jsonData = response.body().string();
//            JSONObject json = new JSONObject(jsonData);
//
//            int status = json.getInt("status");
//            if (status != 0) {
//                callback.onFailure(status, json.optString("message", "未知错误"));
//                return;
//            }
//
//            JSONObject data = json.getJSONObject("data");
//            JSONArray contentArray = data.getJSONArray("content");
//            StringBuilder resultBuilder = new StringBuilder();
//
//            // 提取所有内容块
//            for (int i = 0; i < contentArray.length(); i++) {
//                JSONObject contentItem = contentArray.getJSONObject(i);
//                String dataType = contentItem.getString("dataType");
//                JSONObject dataContent = contentItem.getJSONObject("data");
//
//                if ("txt".equals(dataType) && dataContent.has("data")) {
//                    resultBuilder.append(dataContent.getString("data"));
//                }
//            }
//
//            String aiResponse = resultBuilder.toString();
//            if (aiResponse.isEmpty()) {
//                callback.onFailure(1003, "响应内容为空");
//            } else {
//                callback.onSuccess(aiResponse, data.getString("threadId"));
//            }
//
//        } catch (Exception e) {
//            callback.onFailure(1002, "响应解析失败: " + e.getMessage());
//        }
//    }
//    private void handleApiResponse(Response response, ErnieCallback callback) {
//        try {
//            String jsonData = response.body().string();
//            Log.d("API Response", "原始数据: " + jsonData); // 添加此行
//            JSONObject json = new JSONObject(jsonData);
//
//            int status = json.getInt("status");
//            if (status != 0) {
//                callback.onFailure(status, json.getString("message"));
//                return;
//            }
//
//            JSONObject data = json.getJSONObject("data");
//            this.currentThreadId = data.getString("threadId");
//
//            JSONArray contentArray = data.getJSONArray("content");
//            StringBuilder result = new StringBuilder();
//
//            for (int i = 0; i < contentArray.length(); i++) {
//                JSONObject item = contentArray.getJSONObject(i);
//                if ("text".equals(item.getString("dataType"))) {
//                    result.append(item.getJSONObject("data").getString("text"));
//                }
//            }
//
//            callback.onSuccess(result.toString(), currentThreadId);
//
//        } catch (Exception e) {
//            callback.onFailure(1002, "响应解析失败: " + e.getMessage());
//        }
//    }
    // endregion

    // region 工具方法
    public void clearSession() {
        this.currentThreadId = null;
    }

    public String getCurrentThreadId() {
        return currentThreadId;
    }
    // endregion

    // 增强错误处理

}