package com.example.catvital.aliyun;

import android.content.Context;
import android.util.Log;

import androidx.lifecycle.ViewModelProvider;
import androidx.lifecycle.ViewModelStoreOwner;

import com.example.catvital.SharedViewModel.SharedViewModel;

import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;

import java.lang.ref.WeakReference;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


public class aliyun {
    private static final String TAG = "AliyunIoT";
    private final WeakReference<Context> contextRef;
    private SharedViewModel sharedViewModel;

    // MQTT 配置参数（static final）


    private MqttClient client;
    private MqttConnectOptions options;
    private ScheduledExecutorService scheduler;

    // 构造函数
    public aliyun(Context context) {
        this.contextRef = new WeakReference<>(context);
        if (context instanceof ViewModelStoreOwner) {
            this.sharedViewModel = new ViewModelProvider((ViewModelStoreOwner) context)
                    .get(SharedViewModel.class);
        }
    }

    // 初始化 MQTT（非静态）
    public void mqtt_init() {
        try {
            client = new MqttClient(HOST, CLIENT_ID, new MemoryPersistence());
            options = new MqttConnectOptions();
            options.setCleanSession(true);
            options.setUserName(USERNAME);
            options.setPassword(PASSWORD.toCharArray());
            options.setAutomaticReconnect(true);
            options.setConnectionTimeout(10);
            options.setKeepAliveInterval(60);

            //设置回调
            client.setCallback(new MqttCallback() {
                @Override
                public void connectionLost(Throwable cause) {
                    Log.w(TAG, "Connection lost", cause);
                    start_reconnect();  //触发重连机制
                }

                //mqtt发送数据的时候触发
                @Override
                public void deliveryComplete(IMqttDeliveryToken token) {
                    Log.d(TAG, "Message delivered: " + token.isComplete());
                }

                //消息接收到之后触发
                @Override
                public void messageArrived(String topic, MqttMessage message) {
                    String payload = new String(message.getPayload());
                    Log.d(TAG, "Received message: " + payload);
                    if (sharedViewModel != null) {
                        sharedViewModel.sendToActivityMain(payload);    //发送消息到viewmodel
                    }
                }
            });

            mqtt_connect();
        } catch (Exception e) {
            Log.e(TAG, "Initialization failed", e);
        }
    }

    // 非静态方法
    private void mqtt_connect() {
        new Thread(() -> {
            try {
                if (client != null && !client.isConnected()) {
                    client.connect(options);
                    Log.i(TAG, "Connected to Aliyun IoT");
                    if (sharedViewModel != null) {
                        sharedViewModel.sendToActivityMain("阿里云连接成功");    //发送消息到viewmodel
                    }
                }
            } catch (Exception e) {
                Log.e(TAG, "Connect failed", e);
                if (sharedViewModel != null) {
                    sharedViewModel.sendToActivityMain("阿里云连接失败");    //发送消息到viewmodel
                    start_reconnect();
                }
            }
        }).start();
    }

    // 非静态方法
    private void start_reconnect() {
        if (scheduler != null && !scheduler.isShutdown()) {
            scheduler.shutdownNow();
        }
        scheduler = Executors.newSingleThreadScheduledExecutor();
        scheduler.scheduleAtFixedRate(() -> {
            if (client != null && !client.isConnected()) {
                Log.d(TAG, "Attempting reconnect...");
                mqtt_connect();
            }
        }, 5, 7, TimeUnit.SECONDS);
    }

    // 非静态方法
    public void publishMessage(String payload) {
        if (client == null || !client.isConnected()) {
            Log.w(TAG, "Client not connected, skip publishing");
            return;
        }
        new Thread(() -> {
            try {
                MqttMessage message = new MqttMessage(payload.getBytes());
                message.setQos(1);
                client.publish(PUB_TOPIC, message);
                Log.d(TAG, "Published message: " + payload);
            } catch (Exception e) {
                Log.e(TAG, "Publish failed", e);
            }
        }).start();
    }

    // 非静态方法
    public void subscribeTopic(String topic) {
        if (client == null || !client.isConnected()) {
            Log.w(TAG, "Client not connected, skip subscribe");
            return;
        }
        new Thread(() -> {
            try {
                client.subscribe(topic, 1);
                Log.i(TAG, "Subscribed to: " + topic);
            } catch (MqttException e) {
                Log.e(TAG, "Subscribe failed", e);
            }
        }).start();
    }

    //取消订阅的方法
    public void unsubscribeTopic(String topic) {
        if (client == null || !client.isConnected()) {
            Log.w(TAG, "Client not connected, skip unsubscribe");
            return;
        }
        new Thread(() -> {
            try {
                client.unsubscribe(topic);
                Log.i(TAG, "Unsubscribed from: " + topic);
                if (sharedViewModel != null) {
                    sharedViewModel.sendToActivityMain("已取消订阅: " + topic); // 通知 UI
                }
            } catch (MqttException e) {
                Log.e(TAG, "Unsubscribe failed", e);
            }
        }).start();
    }
}