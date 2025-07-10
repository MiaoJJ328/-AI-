plugins {
    alias(libs.plugins.android.application)
}

android {
    namespace = "com.example.catvital"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.example.catvital"
        minSdk = 24
        targetSdk = 35
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
}

dependencies {

    implementation(libs.appcompat)
    implementation(libs.material)
    implementation(libs.activity)
    implementation(libs.constraintlayout)
    testImplementation(libs.junit)
    androidTestImplementation(libs.ext.junit)
    androidTestImplementation(libs.espresso.core)
    implementation("androidx.appcompat:appcompat:1.7.0")
    implementation("com.google.android.material:material:1.12.0")
    implementation("androidx.constraintlayout:constraintlayout:2.1.4")
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test.ext:junit:1.1.5")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.5.1")
    implementation("org.eclipse.paho:org.eclipse.paho.client.mqttv3:1.1.0")
    implementation("org.eclipse.paho:org.eclipse.paho.android.service:1.1.1")
    //implementation("androidx.legacy:legacy-support-v4:1.0.0")
    implementation("androidx.localbroadcastmanager:localbroadcastmanager:1.0.0")
    //LiveData + ViewModel实现数据的通信
    implementation("androidx.lifecycle:lifecycle-viewmodel:2.6.2")
    implementation( "androidx.lifecycle:lifecycle-livedata:2.6.2")
    //Gson
    implementation("com.google.code.gson:gson:2.10.1")
    //控件保存
    implementation("androidx.lifecycle:lifecycle-viewmodel:2.6.2")     // Java 项目
    implementation("androidx.lifecycle:lifecycle-livedata:2.6.2")
    implementation("androidx.lifecycle:lifecycle-runtime:2.6.2")
    implementation(files("libs\\mysql-connector-java.jar"))
    //画图使用
    implementation("com.github.PhilJay:MPAndroidChart:v3.1.0")

    //AI使用
    implementation("com.squareup.okhttp3:okhttp:4.9.3")
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.6.0")

    //图表
    implementation("com.github.PhilJay:MPAndroidChart:v3.1.0")
}