
// ESP32有两个核心：APPlication核心（应用核心）和PROcess核心（运行ESP32 SDK堆栈的核心）。
#define APP_CPU 1
#define PRO_CPU 0

#include "OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

#include <esp_bt.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

// 选择相机型号
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

#include "home_wifi_multi.h"

OV2640 cam;

WebServer server(80);


// 视频流传输是通过3个任务实现的：
TaskHandle_t tMjpeg;   // 处理客户端到服务器的连接
TaskHandle_t tCam;     // 处理从相机获取图片帧并将其本地存储
TaskHandle_t tStream;  // 实际上将帧流传输给所有已连接的客户端
// frameSync信号量用于防止在替换为下一帧时流传输缓冲区
SemaphoreHandle_t frameSync = NULL;

// Queue存储当前已连接的客户端，我们将向他们传输流
QueueHandle_t streamingClients;

// 尝试达到25 FPS的帧率
const int FPS = 100;

// 我们将每50毫秒处理一次Web客户端请求（20 Hz）
const int WSINTERVAL = 100;


// ======== 服务器连接处理任务 ==========================
void mjpegCB(void* pvParameters) {
  TickType_t xLastWakeTime;// 用于记录任务唤醒的时间
  const TickType_t xFrequency = pdMS_TO_TICKS(WSINTERVAL);

  // 创建帧同步信号量并初始化
  frameSync = xSemaphoreCreateBinary();
  xSemaphoreGive( frameSync );// 给出信号量，表示帧同步可用

  // 创建一个队列来跟踪所有已连接的客户端
  streamingClients = xQueueCreate( 10, sizeof(WiFiClient*) );

  //=== 设置部分   ==================

  //  创建RTOS任务，用于从相机抓取帧
  xTaskCreatePinnedToCore(
    camCB,        // 回调函数，用于相机任务
    "cam",        // 任务名称
    4096,         // 任务堆栈大小
    NULL,         // 任务参数
    2,            //  任务优先级
    &tCam,        // RTOS任务句柄
    APP_CPU);     // 指定任务运行在APPlication核心

  //  创建任务，用于将流推送到所有已连接的客户端
  xTaskCreatePinnedToCore(
    streamCB,//回调函数，用于流传输任务
    "strmCB",// 任务名称
    4 * 1024,// 任务堆栈大小
    NULL, //(void*) handler,
    2,// 任务优先级
    &tStream, // RTOS任务句柄
    APP_CPU);// 指定任务运行在APPlication核心

  //  R注册Web服务器处理例程
  server.on("/mjpeg/1", HTTP_GET, handleJPGSstream);// 处理MJPEG流请求
  server.on("/cam1/mjpeg/1", HTTP_GET, handleJPGSstream);
  server.on("/jpg", HTTP_GET, handleJPG);// 处理JPEG图像请求
  server.onNotFound(handleNotFound);// 处理未找到请求的回调

  //   启动Web服务器
  server.begin();

  //=== loop() section  ===================
  xLastWakeTime = xTaskGetTickCount();// 获取当前系统节拍计数
  for (;;) {
    server.handleClient();

    // 在处理每个服务器客户端请求后，让其他任务运行然后暂停
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


// Commonly used variables:
volatile size_t camSize;    // size of the current frame, byte
volatile char* camBuf;      // pointer to the current frame


// ==== RTOS任务，用于从相机抓取帧 =========================
// 相机回调函数，用于从相机获取帧数据
void camCB(void* pvParameters) {

  TickType_t xLastWakeTime;// 用于记录任务唤醒的时间

  // 当前期望的帧率对应的运行间隔
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / FPS);// 将帧率转换为系统节拍

    // 用于保护关键区的互斥量，在切换活动帧时使用
  portMUX_TYPE xSemaphore = portMUX_INITIALIZER_UNLOCKED;// 初始化一个未上锁的互斥量

  //  指向两个帧的指针，它们各自的大小和当前帧的索引
  char* fbs[2] = { NULL, NULL };
  size_t fSize[2] = { 0, 0 };
  int ifb = 0;

  //===  循环部分   ===================
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {// 无限循环

    //  从相机抓取一帧并查询其大小
    cam.run();
    size_t s = cam.getSize();

    //  如果帧大小超过了之前分配的大小 - 请求当前帧空间的125%
    if (s > fSize[ifb]) {
      fSize[ifb] = s * 4 / 3;
      fbs[ifb] = allocateMemory(fbs[ifb], fSize[ifb]);
    }

    //  将当前帧复制到本地缓冲区
    char* b = (char*) cam.getfb();
    memcpy(fbs[ifb], b, s);

    //  让其他任务运行并等待当前帧率间隔结束（如果有剩余时间）
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

   // 只有在没有客户端正在接收帧时才切换帧
    // 等待信号量直到客户端操作完成
    xSemaphoreTake( frameSync, portMAX_DELAY );

    //  在切换当前帧时不允许中断
    portENTER_CRITICAL(&xSemaphore);
    camBuf = fbs[ifb];
    camSize = s;
    ifb++;
    ifb &= 1;  // 通过位运算实现循环切换，结果为1, 0, 1, 0, ...
    portEXIT_CRITICAL(&xSemaphore);

    //  通知等待帧的任务帧已准备好
    xSemaphoreGive( frameSync );

    //  通知流传输任务至少有一个帧可用，可以开始向客户端发送帧（如果有客户端）
    xTaskNotifyGive( tStream );

    //  立即让其他（流传输）任务运行
    taskYIELD();

    // 如果流传输任务已经挂起（没有活跃的客户端要发送帧）
    // 则无需从相机抓取帧。我们可以通过挂起任务来节省电力
    if ( eTaskGetState( tStream ) == eSuspended ) {
      vTaskSuspend(NULL);  // passing NULL means "suspend yourself"
    }
  }
}


// ==== 利用PSRAM（如果存在）的内存分配器 =======================
char* allocateMemory(char* aPtr, size_t aSize) {

  //  由于当前缓冲区太小，释放它。
  if (aPtr != NULL) free(aPtr);


  size_t freeHeap = ESP.getFreeHeap();
  char* ptr = NULL;

  // 如果请求的内存超过当前空闲堆的2/3，立即尝试使用PSRAM。
  if ( aSize > freeHeap * 2 / 3 ) {
    if ( psramFound() && ESP.getFreePsram() > aSize ) {
      ptr = (char*) ps_malloc(aSize);
    }
  }
  else {
    //  有足够的空闲堆空间 - 让我们尝试分配快速RAM作为缓冲区。
    ptr = (char*) malloc(aSize);

    //  如果在堆上分配失败，让我们再给PSRAM一次机会
    if ( ptr == NULL && psramFound() && ESP.getFreePsram() > aSize) {
      ptr = (char*) ps_malloc(aSize);
    }
  }

  // 如果内存指针为NULL，说明我们无法分配任何内存，这是一种终结条件
  if (ptr == NULL) {
    ESP.restart();
  }
  return ptr;
}


// ==== STREAMING ======================================================
const char HEADER[] = "HTTP/1.1 200 OK\r\n" \
                      "Access-Control-Allow-Origin: *\r\n" \
                      "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";
const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
const int hdrLen = strlen(HEADER);
const int bdrLen = strlen(BOUNDARY);
const int cntLen = strlen(CTNTTYPE);


// ==== 处理客户端的连接请求 ===============================
void handleJPGSstream(void)
{
  //  只能容纳10个客户端。这个限制是WiFi连接的默认设置。
  if ( !uxQueueSpacesAvailable(streamingClients) ) return;


  //  创建一个新的WiFi客户端对象以跟踪这个客户端。
  WiFiClient* client = new WiFiClient();
  *client = server.client();

  //  立即向这个客户端发送一个头部信息。
  client->write(HEADER, hdrLen);
  client->write(BOUNDARY, bdrLen);

  // 将客户端推送到流媒体队列中。
  xQueueSend(streamingClients, (void *) &client, 0);

  // 唤醒流媒体任务，如果它们之前被挂起:
  if ( eTaskGetState( tCam ) == eSuspended ) vTaskResume( tCam );
  if ( eTaskGetState( tStream ) == eSuspended ) vTaskResume( tStream );
}


// ==== 向所有连接的客户端流式传输内容 ========================
void streamCB(void * pvParameters) {
  char buf[16];
  TickType_t xLastWakeTime;
  TickType_t xFrequency;

  //  等待直到捕获第一帧，并且有内容可以发送到客户端
  
  ulTaskNotifyTake( pdTRUE,          /* 在退出前清除通知值 */
                    portMAX_DELAY ); /* 阻塞无限期. */

  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    // 假设操作或执行是按照每秒帧数（Frames Per Second，FPS）来进行的
    xFrequency = pdMS_TO_TICKS(1000 / FPS);

    //  有人看时在发送
    UBaseType_t activeClients = uxQueueMessagesWaiting(streamingClients);
    if ( activeClients ) {
      // 调整周期以适应连接的客户端数量
      xFrequency /= activeClients;

      //  由于我们要向所有人发送相同的帧， 从队列前端移除一个客户端
      WiFiClient *client;
      xQueueReceive (streamingClients, (void*) &client, 0);

      //  检测是否还在连接.

      if (!client->connected()) {
        //  如果客户端没连接，就删除
        
        delete client;
      }
      else {

        //  Ok. This is an actively connected client.
        //  获取一个信号量，以防止在我们服务这一帧的同时帧发生变化
        xSemaphoreTake( frameSync, portMAX_DELAY );

        client->write(CTNTTYPE, cntLen);
        sprintf(buf, "%d\r\n\r\n", camSize);
        client->write(buf, strlen(buf));
        client->write((char*) camBuf, (size_t)camSize);
        client->write(BOUNDARY, bdrLen);

        // Since this client is still connected, push it to the end
        // of the queue for further processing
        xQueueSend(streamingClients, (void *) &client, 0);

        //  帧已经发送完毕。释放信号量，让其他任务运行
        //  如果在帧之间已经准备好进行帧切换，那么它将现在发生。
        xSemaphoreGive( frameSync );
        taskYIELD();
      }
    }
    else {
      //  没有连接的客户端，就挂起
      vTaskSuspend(NULL);
    }
    //  在为每个客户提供服务后，让其他任务运行。
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}



const char JHEADER[] = "HTTP/1.1 200 OK\r\n" \
                       "Content-disposition: inline; filename=capture.jpg\r\n" \
                       "Content-type: image/jpeg\r\n\r\n";
const int jhdLen = strlen(JHEADER);

// ==== 提供一帧JPEG图像。 =============================================
void handleJPG(void)
{
  WiFiClient client = server.client();

  if (!client.connected()) return;
  cam.run();
  client.write(JHEADER, jhdLen);
  client.write((char*)cam.getfb(), cam.getSize());
}


// ==== 处理无效的URL请求 ============================================
void handleNotFound()
{
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  server.send(200, "text / plain", message);
}



// ==== SETUP method ==================================================================
void setup()
{

  // 设置串行连接
  Serial.begin(115200);
  delay(1000); // wait for a second to let Serial connect


  // 配置摄像头
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame parameters: pick one
  //  config.frame_size = FRAMESIZE_UXGA;
  //  config.frame_size = FRAMESIZE_SVGA;
  //  config.frame_size = FRAMESIZE_QVGA;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  if (cam.init(config) != ESP_OK) {
    Serial.println("Error initializing the camera");
    delay(10000);
    ESP.restart();
  }


  //  Configure and connect to WiFi
  IPAddress ip;

  WiFi.mode(WIFI_STA);
  WiFi.begin("Miao", "miao0416");//WIFI名称和密码

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }
  ip = WiFi.localIP();
  Serial.println(F("WiFi connected"));
  Serial.println("");
  Serial.print("Stream Link: http://");
  Serial.print(ip);
  Serial.println("/cam1/mjpeg/1");


  // 开始将RTOS任务主流化
  xTaskCreatePinnedToCore(
    mjpegCB,
    "mjpeg",
    4 * 1024,
    NULL,
    2,
    &tMjpeg,
    APP_CPU);
}


void loop() {
  vTaskDelay(1000);
}
