///////////////////////////////////////////////////////////////////////////////

#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <errno.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <dirent.h>
#include <math.h>
#include "esp_system.h"
#include "esp_types.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "lwip/api.h"
#include "lwip/opt.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/netbuf.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_task.h"
#include "neuralite.h"

///////////////////////////////////////////////////////////////////////////////

static const char* TAG = "Headstage";

#define WIFI_MODE_STA 1
#define WIFI_SSID "TP-LINK_34F1"
#define WIFI_PASS "57857043"

#define noVERBOSE 1

///////////////////////////////////////////////////////////////////////////////

struct { 
        u8  step[NumOfChannels];
        u16 channel[NumOfChannels];
        u16 n_selected_channels;
        u16 msg_len;
} lrs_map;

u8  buffer_step[NumOfChannels];

#define FramePoolSize 8
#define FrameBufSize  10
typedef struct { 
        u32* sample_buf[NumOfChannels];
        u32 fid;
} Frame;
Frame frame[FrameBufSize];

esp_err_t initWiFi();
esp_err_t handshake();
void sendMsgToTcp(char*);
void recvMsgFromTcp(char*, u16);
void streamerTask();
void senderTask();
void CpuTask();

TaskHandle_t h_streamer = NULL;
TaskHandle_t h_sender = NULL;
char* tx_buf[2] = { 0 };
char* rx_buf = NULL;
int tcp_sock;
u32 fid = 0;

///////////////////////////////////////////////////////////////////////////////

void app_main(void) {

        // verify config
        ESP_LOGI(TAG, "NumOfChannels  %d", NumOfChannels);
        ESP_LOGI(TAG, "FrameRate      %d", FrameRate);
        ESP_LOGI(TAG, "SampleBitwidth %d", SampleBitwidth);
        ESP_LOGI(TAG, "SampleRate     %d", SampleRate);
        ESP_LOGI(TAG, "SpikeLen       %d", SpikeLen);
        ESP_LOGI(TAG, "TrainingPeriod %f", TrainingPeriod);
        ESP_LOGI(TAG, "SamplesPerGrap %d", SamplesPerGrp);
        ESP_LOGI(TAG, "FrameWidth     %d", FrameWidth);
        ESP_LOGI(TAG, "FrameHeight    %d", FrameHeight);
        ESP_LOGI(TAG, "FrameSize      %d", FrameSize);
        
        assert(NumOfChannels < MaxNumOfChannels);
        assert(FrameRate > MinFrameRate);
        assert(SampleRate % FrameRate == 0);

        // allocate tx and rx buffers
        assert(tx_buf[0] = malloc(HsTxBufSize));
        assert(tx_buf[1] = malloc(HsTxBufSize));
        assert(rx_buf    = malloc(HsRxBufSize));

        // init Wi-Fi
        ESP_ERROR_CHECK(initWiFi());

        // handshake
        while (handshake() != ESP_OK) {
                // close socket and reconnect
                close(tcp_sock);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        assert(xTaskCreatePinnedToCore(streamerTask,
                                       "streamer",
                                       4096,
                                       NULL,
                                       10,
                                       &h_streamer,
                                       1)
        == pdPASS);

        while (true) {
                vTaskDelay(500 / portTICK_PERIOD_MS);
        }
}

//// Streamer Core ////////////////////////////////////////////////////////////

QueueHandle_t q_free_frame_slot;
QueueHandle_t q_full_frame_slot;
int frame_count_free[FrameBufSize+1];
int frame_count_full[FrameBufSize+1];

char* sender_buf = NULL;
char* asm_buf = NULL;

u32 frame_loss = 0;
u32 frame_sent = 0;

inline void swap(char** a, char** b) {
        char* tmp = *a;
        *a = *b;
        *b = tmp;
}


void CpuTask() {
        uint8_t CPU_RunInfo[400];
        vTaskList((char *)&CPU_RunInfo);
        printf("----------------------------------------------------\r\n");
        printf("task_name     task_status     priority stack task_id\r\n");
        printf("%s", CPU_RunInfo);
        printf("----------------------------------------------------\r\n");
        
        memset(CPU_RunInfo, 0, 400);
        vTaskGetRunTimeStats((char *)&CPU_RunInfo);
        printf("task_name      run_cnt                 usage_rate   \r\n");
        printf("%s", CPU_RunInfo);
        printf("----------------------------------------------------\r\n");
    
}

void senderTask() {
        while (true) {
                // wait for the ctrler to finish assembling a msg
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                // ctrler's done, the assembled msg is in sender_buf, send
                sendMsgToTcp(sender_buf);
                // done, report to the ctrler
                xTaskNotifyGive(h_streamer);
        }
}

void lrsAssembler(char* lrs_msg, Frame* f) {
        setMsgHdr(lrs_msg, MSG_LRS, f->fid, lrs_map.msg_len);
        char* lrs_msg_pld = lrs_msg + MsgHdrLen + SamplesHdrLen;
        u16 n_samples_wrote = 0;
        for (u16 i = 0; i < lrs_map.n_selected_channels; i++) {
                u16 channel = lrs_map.channel[i];
                u8  step = lrs_map.step[channel];
                //assert(step % buffer_step[channel] == 0);
                for (u16 t = 0; t < FrameHeight; t += step) {
                        u16 i = t / buffer_step[channel];
                        writeSample((u32*)lrs_msg_pld, n_samples_wrote++,
                             readSample(f->sample_buf[channel], i));
                }
        }
        samplesHdr(lrs_msg)->n_samples = n_samples_wrote;
}

void hdsAssembler(char* req_msg, char* hds_msg, Frame* f) {
        // parse hds req
        u32 req_fid = getMsgFid(req_msg);
        HdsReqHdr* req_hdr = hdsReqHdr(req_msg);
        char* req_ptr = req_msg + MsgHdrLen + HdsReqHdrLen;
        // init hds msg
        u16 hds_msg_len = MsgHdrLen + SamplesHdrLen + SampleGrpSize *
                 ceil((float)req_hdr->n_req_samples / SamplesPerGrp);
        setMsgHdr(hds_msg, MSG_HDS, req_fid, hds_msg_len);
        char* hds_msg_pld = hds_msg + MsgHdrLen + SamplesHdrLen;
        // write samples into pld
        HdsReq r = {0};
        u16 n_samples_wrote = 0;
        for (u16 n = 0; n < req_hdr->n_reqs; n++) {
                decHdsReq(req_ptr, &r);
                req_ptr += HdsReqCodeLen;
                i16 ts = r.sample_idx;
                i16 te = r.sample_idx + SpikeLen;
                if (te > FrameHeight) te = FrameHeight;
                if (ts < 0) ts = 0;
                // assert(r.sample_idx % buffer_step[r.channel] == 0
                //           && r.step % buffer_step[r.channel] == 0);
                u8 lrs_sampling_step = lrs_map.step[r.channel];
                if (lrs_sampling_step == 0) {
                        for (u16 t = ts; t < te; t += r.step) {
                                u16 i = t / buffer_step[r.channel];
                                writeSample((u32*)hds_msg_pld, n_samples_wrote++,
                                    readSample(f->sample_buf[r.channel], i));
                        }
                } else {
                        for (u16 t = ts; t < te; t += r.step) {
                                if (t % lrs_sampling_step == 0) continue;
                                u16 i = t / buffer_step[r.channel];
                                writeSample((u32*)hds_msg_pld, n_samples_wrote++,
                                    readSample(f->sample_buf[r.channel], i));
                        }
                }
        }
        samplesHdr(hds_msg)->n_samples = n_samples_wrote;
}

void frameAlarmCallback(void* arg) {
        fid++;
        Frame* f = NULL;
        //UBaseType_t queueLength = uxQueueMessagesWaiting(q_full_frame_slot);
        //UBaseType_t queueLength_free = uxQueueMessagesWaiting(q_free_frame_slot);
        //frame_count_free[queueLength_free] +=1;
        //frame_count_full[queueLength] +=1;
        /*
        if (fid==10000){
                for(int i=0;i<FrameBufSize+1;i++){
                        ESP_LOGI("QUEUE", "free num: %u", frame_count_free[i]);
                }
                for(int i=0;i<FrameBufSize+1;i++){
                        ESP_LOGI("QUEUE", "full num: %u", frame_count_full[i]);
                }
        }*/
        
        if (xQueueReceive(q_free_frame_slot, &f, 0)!=pdPASS) {
                // no free frame slot is available
                frame_loss++;
        } else {
                f->fid = fid;
                char* buf = malloc(FrameWidth);
                for (u16 t = 0; t < FrameHeight; t++) {
                        memset(buf, 0, FrameWidth);
                }
                free(buf);
                assert(xQueueSendToBack(q_full_frame_slot, &f, 0) == pdPASS);
        }
}

inline Frame* getFrame(u32 fid) {
        for (u8 i = 0; i < FrameBufSize; i++) {
                if (frame[i].fid == fid)
                        return &frame[i];
        }
        return NULL;
}

void streamerTask() 
{
        // init ctrler and sender buffers
        asm_buf = tx_buf[0];
        sender_buf = tx_buf[1];

        // start the sender task
        assert(xTaskCreatePinnedToCore(senderTask,
                                       "sender",
                                       2048,
                                       NULL,
                                       10,
                                       &h_sender,
                                       0)
        == pdPASS);
        // init data pipe
        assert(q_free_frame_slot = xQueueCreate(FrameBufSize, sizeof(Frame*)));
        assert(q_full_frame_slot = xQueueCreate(FrameBufSize, sizeof(Frame*)));
        for (int i = 0; i < FrameBufSize; i++) {
                Frame* f = &frame[i];
                assert(xQueueSendToBack(q_free_frame_slot, &f, 0) == pdPASS);
        }
        const esp_timer_create_args_t frame_alarm_args = {
                .callback = &frameAlarmCallback,
                .name = ""
        };
        esp_timer_handle_t frame_alarm;
        ESP_ERROR_CHECK(esp_timer_create(&frame_alarm_args, &frame_alarm));
        ESP_ERROR_CHECK(esp_timer_start_periodic(frame_alarm, 1000000/FrameRate));

        // prepare a Halo to fill the very first time slot of streaming
        setMsgHdr(asm_buf, MSG_HALO, 0, MsgHdrLen);

        // start stream
        frame_loss = 0;
        frame_sent = 0;
        while (true) {
                for (u8 i = 0; i < FramePoolSize; i++) {
                        /* An assembled msg is in the asm_buf.
                           For the 1st iteration of this for loop, if we just
                           started the streaming, then this msg is the Halo msg
                           assembled before entering this while loop. 
                           Otherwise, the msg is the last hds of the last frame
                           pool.
                           Swap the assembled msg to the sender_buf and notify
                           it to transmit 
                        */
                        //uint64_t startTime = esp_timer_get_time();
                        swap(&asm_buf, &sender_buf);
                        xTaskNotifyGive(h_sender);
                        // get a full frame slot
                        Frame* f = NULL;
                        assert(xQueueReceive(q_full_frame_slot, &f,
                                             portMAX_DELAY) == pdPASS);
                        // assemble the lrs msg for this frame
                        lrsAssembler(asm_buf, f);                        
                        //uint64_t endTime = esp_timer_get_time();
                        //uint64_t elapsedTime = endTime - startTime;
                        //printf("lrs执行时间：%lld 微秒\n", elapsedTime);
                        // wait for the sender to complete
                        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

                }

                /* The lrs of the last frame of the frame pool is now in the
                   asm_buf. The 1st iteration of the for loop below will send
                   this lrs and assemble the first hds of this frame pool
                */

                for (u8 i = 0; i < FramePoolSize; ) {
                        /* An assembled msg is in the asm_buf.
                           If this is the 1st iteration, then this msg is the 
                           last lrs of this frame pool. Otherwise, it is the
                           hds of the (i-1)-th frame of this pool
                           Swap the assembled msg to the sender_buf and notify
                           it to transmit 
                        */
                        //uint64_t startTime = esp_timer_get_time();
                        swap(&asm_buf, &sender_buf);
                        xTaskNotifyGive(h_sender);
                        /* Get thehds req of the i-th frame in this pool from 
                           the tcp recv buf and assemble hds msg accordingly 
                        */
                        recvMsgFromTcp(rx_buf, HsRxBufSize);
                        //assert(getMsgCode(rx_buf) == MSG_HDS_REQ);
                        Frame* f = getFrame(getMsgFid(rx_buf));
                        assert(f);
                        hdsAssembler(rx_buf, asm_buf, f);
                        // Wait for the sender to complete
                        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                        /* Check if the recved hds req is the last req of this
                           frame. If yes, we've finished streaming this frame
                           and can release the slot for the data pipe to write.
                        */
                        if (!hdsReqHdr(rx_buf)->more) {
                                assert(xQueueSendToBack(q_free_frame_slot,
                                                        &f, 0) == pdPASS);
                                frame_sent++;
                                if (frame_sent % 500 == 0) {
                                        if (frame_sent==8000)
                                        CpuTask();
                                        u32 n_frames = frame_sent + frame_loss;
                                        double lr = (float)frame_loss/n_frames;
                                        ESP_LOGI(TAG, "loss rate: %ld/%ld=%f",
                                                frame_loss, n_frames, lr);
                                }
                                i++;
                        }
                        //uint64_t endTime = esp_timer_get_time();
                        //uint64_t elapsedTime = endTime - startTime;
                        //printf("hds执行时间：%lld 微秒\n", elapsedTime);
                }

                /* The hds of the last frame of this frame pool is now in the
                   asm_buf. The 1st iteration of the for loop in the next wihle
                   loop iteration will send this last hds and assemble the
                   first lrs of the next frame pool
                */
        }

        vTaskDelete(h_sender);
        vTaskDelete(NULL);
}

//// Handshake ////////////////////////////////////////////////////////////////

esp_err_t estTcpConn() {
        tcp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (tcp_sock == -1) {
                ESP_LOGE(TAG, "estTcpConn: create socket err: %d", errno);
                return ESP_FAIL;
        }
        ESP_LOGI(TAG, "estTcpConn: socket created");

        int err = 0;
        int val = 1;
        err = setsockopt(tcp_sock, IPPROTO_TCP, TCP_NODELAY, (char*)&val, 
                                                             sizeof(val));
        if (err == -1) {
                ESP_LOGE(TAG, "estTcpConn: failed setting TCP_NODELAY");
                close(tcp_sock);
                return ESP_FAIL;
        }

        ESP_LOGI(TAG, "estTcpConn: connecting to %s:%d", SERVER_IP, TCP_PORT);

        struct sockaddr_in sa = {0};
        sa.sin_family = AF_INET;
        sa.sin_addr.s_addr = inet_addr(SERVER_IP);
        sa.sin_port = htons(TCP_PORT);
        err = connect(tcp_sock, (struct sockaddr*)&sa, sizeof(sa));
        if (err == -1) {
                ESP_LOGE(TAG, "estTcpConn, connect err: %d", errno);
                close(tcp_sock);
                return ESP_FAIL;
        }

        ESP_LOGI(TAG, "estTcpConn: successfully connected");
        return ESP_OK;
}

esp_err_t training() {
        u32 len = MsgHdrLen + FrameSize;
        char* frame_msg = heap_caps_malloc(len, MALLOC_CAP_8BIT);
        assert(frame_msg);
        setMsgHdr(frame_msg, MSG_FRAME, 0, len);
        u32 n_training_frames = TrainingPeriod * FrameRate;
        for (u32 i = 0; i < n_training_frames; i++) {
                sendMsgToTcp(frame_msg);
        }
        ESP_LOGI(TAG, "sent %ld training frames", n_training_frames);
        free(frame_msg);
        return ESP_OK;
}

esp_err_t setupFrameBuffer() {
        recvMsgFromTcp(rx_buf, HsRxBufSize);
        assert(getMsgCode(rx_buf) == MSG_BUF_STEP);
        memcpy(buffer_step, rx_buf + MsgHdrLen, NumOfChannels);

        ESP_LOGI(TAG, "allocating frame bufs ...");

        for (u16 ch = 0; ch < NumOfChannels; ch++) {
                if (buffer_step[ch] == 0) continue;
                u16 n_samples = FrameHeight / buffer_step[ch];
                u16 n_sample_grps = ceil((float)n_samples / SamplesPerGrp);
                u16 buf_size = SampleGrpSize * n_sample_grps;
                for (u8 i = 0; i < FrameBufSize; i++) {
                        frame[i].sample_buf[ch] =
                                heap_caps_malloc(buf_size, MALLOC_CAP_32BIT);
                        if (!frame[i].sample_buf[ch]) {
                                ESP_LOGE(TAG, "heap running out");
                                heap_caps_print_heap_info(MALLOC_CAP_32BIT);
                                return ESP_FAIL;
                        }
                }
        }

        ESP_LOGI(TAG, "done.");
        ESP_LOGI(TAG, "heap remaining");
        heap_caps_print_heap_info(MALLOC_CAP_32BIT);
        return ESP_OK;
}

esp_err_t setupLrsMap() {
        recvMsgFromTcp(rx_buf, HsRxBufSize);
        assert(getMsgCode(rx_buf) == MSG_LRS_MAP);
        memcpy(lrs_map.step, rx_buf + MsgHdrLen, NumOfChannels);
        u16 n_selected_channels = 0;
        u16 n_samples = 0;
        for (u16 ch = 0; ch < NumOfChannels; ch++) {
                if (lrs_map.step[ch] == 0) continue;
                n_samples += (u16)ceil((float)FrameHeight / lrs_map.step[ch]);
                lrs_map.channel[n_selected_channels] = ch;
                n_selected_channels++;
        }
        lrs_map.msg_len = MsgHdrLen + SamplesHdrLen +  
                SampleGrpSize * ceil((float)n_samples / SamplesPerGrp);
        assert(lrs_map.msg_len <= HsTxBufSize);
        lrs_map.n_selected_channels = n_selected_channels;
        ESP_LOGI(TAG, "set lrs map:");
        for (u16 i = 0; i < lrs_map.n_selected_channels; i++) {
                u16 ch = lrs_map.channel[i];
                ESP_LOGI(TAG, "%d:%d", ch, lrs_map.step[ch]);
        }
        ESP_LOGI(TAG, "lrs msg len: %d", lrs_map.msg_len);
        return ESP_OK;
}

esp_err_t handshake() {
        ESP_LOGI(TAG, "start handshake");
        if (estTcpConn() != ESP_OK
         || training() != ESP_OK
         || setupFrameBuffer() != ESP_OK
         || setupLrsMap() != ESP_OK)
                return ESP_FAIL;
        return ESP_OK;
}

//// Send/Recv ////////////////////////////////////////////////////////////////

esp_err_t sendBytesToTcp(char* p, int len) {
        char* p_cur = p;
        char* p_end = p+len;
        while (p_cur != p_end) {
                int n = send(tcp_sock, p_cur, p_end-p_cur, 0);
                if (n <= 0) {
                        ESP_LOGE(TAG, "sendBytesToTcp err: %d", errno);
                        return ESP_FAIL;
                }
                p_cur += n;
        }
        return ESP_OK;
}

void sendMsgToTcp(char* msg) {
        ESP_ERROR_CHECK(sendBytesToTcp(msg, getMsgLen(msg)));
        #ifdef VERBOSE
        ESP_LOGI(TAG, "[%08ld] sent %s of %ld bytes", getMsgFid(msg),
                                                      getMsgCodeStr(msg),
                                                      getMsgLen(msg));
        #endif
}

esp_err_t recvBytesFromTcp(char* p, int len) {
        char* p_cur = p;
        char* p_end = p + len;
        while (p_cur != p_end) {
                int n = recv(tcp_sock, p_cur, p_end-p_cur, 0);
                if (n <= 0) {
                        ESP_LOGE(TAG, "recvBytesFromTcp err: %d", errno);
                        return ESP_FAIL;
                }
                p_cur += n;
        }
        return ESP_OK;
}

void recvMsgFromTcp(char* buf, u16 buf_size) {
        ESP_ERROR_CHECK(recvBytesFromTcp(buf, MsgHdrLen));
        u16 len = getMsgLen(buf);
        assert(len <= buf_size);
        if (len > MsgHdrLen) {
                ESP_ERROR_CHECK(recvBytesFromTcp(buf + MsgHdrLen,
                                                 len - MsgHdrLen));
        }
        #ifdef VERBOSE
        ESP_LOGI(TAG, "[%08ld] recved %s of %ld bytes", getMsgFid(buf),
                                                        getMsgCodeStr(buf),
                                                        getMsgLen(buf));
        #endif
}

//// Wi-Fi ////////////////////////////////////////////////////////////////////

#ifdef WIFI_MODE_STA

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t wifi_event_group;

void wifiStaEventHandler(void* arg,
                         esp_event_base_t event_base,
                         int32_t event_id,
                         void* event_data)
{
        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
                esp_wifi_connect();
        }
        else if (event_base == WIFI_EVENT
                && event_id == WIFI_EVENT_STA_DISCONNECTED) {
                xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGI(TAG, "connect to the AP fail");
        }
        else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
                ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
                ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
                xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        }
}

esp_err_t initWiFi()
{
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES
         || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
                ESP_ERROR_CHECK(nvs_flash_erase());
                ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
        ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
        wifi_event_group = xEventGroupCreate();
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        esp_event_handler_instance_t instance_any_id;
        esp_event_handler_instance_t instance_got_ip;
        ESP_ERROR_CHECK(
                esp_event_handler_instance_register(WIFI_EVENT,
                                                    ESP_EVENT_ANY_ID,
                                                    &wifiStaEventHandler,
                                                    NULL,
                                                    &instance_any_id));
        ESP_ERROR_CHECK(
                esp_event_handler_instance_register(IP_EVENT,
                                                    IP_EVENT_STA_GOT_IP,
                                                    &wifiStaEventHandler,
                                                    NULL,
                                                    &instance_got_ip));
        wifi_config_t wifi_config = {
                .sta = {
                        .ssid = WIFI_SSID,
                        .password = WIFI_PASS,
                        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
                },
        };
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG, "wifi_init_sta finished.");
        EventBits_t bits =
                xEventGroupWaitBits(wifi_event_group,
                                    WIFI_CONNECTED_BIT|WIFI_FAIL_BIT,
                                    pdFALSE, pdFALSE,
                                    portMAX_DELAY);

        if (bits & WIFI_CONNECTED_BIT) {
                ESP_LOGI(TAG, "connected to SSID:%s password:%s", 
                        WIFI_SSID, WIFI_PASS);
                return ESP_OK;
        }
        if (bits & WIFI_FAIL_BIT) {
                ESP_LOGI(TAG, "failed to connect SSID:%s, password:%s", 
                        WIFI_SSID, WIFI_PASS);
        }
        else {
                ESP_LOGE(TAG, "unexpected event");
        }
        return ESP_FAIL;
}

#else

void wifiApEventHandler(void* arg,
                        esp_event_base_t event_base,
                        int32_t event_id,
                        void* event_data)
{
        if (event_id == WIFI_EVENT_AP_STACONNECTED) {
                wifi_event_ap_staconnected_t* event =
                        (wifi_event_ap_staconnected_t*)event_data;
                ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                        MAC2STR(event->mac), event->aid);
        }
        else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
                wifi_event_ap_stadisconnected_t* event =
                        (wifi_event_ap_stadisconnected_t*)event_data;
                ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                        MAC2STR(event->mac), event->aid);
        }
}

bool initWiFi()
{
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES
         || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
                ESP_ERROR_CHECK(nvs_flash_erase());
                ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_ap();
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        ESP_ERROR_CHECK(
                esp_event_handler_instance_register(WIFI_EVENT,
                                                    ESP_EVENT_ANY_ID,
                                                    &wifiApEventHandler,
                                                    NULL, NULL));
        wifi_config_t wifi_config = {
                .ap = {
                        .ssid = WIFI_SSID,
                        .ssid_len = strlen(WIFI_SSID),
                        .channel = WIFI_AP_CHANNEL,
                        .password = WIFI_PASS,
                        .max_connection = 1, // only server
                        .authmode = WIFI_AUTH_WPA2_PSK,
                        .pmf_cfg = {
                                .required = true,
                        },
                },
        };
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG, "wifiInitAp finished. SSID:%s passwd:%s ch:%d",
                WIFI_SSID, WIFI_PASS, WIFI_AP_CHANNEL);
        return true;
}

#endif

/*
bool connectToServer() {
        ip_addr_t server_addr;
        if (ipaddr_aton(SERVER_ADDR, &server_addr) == 0) {
                ESP_LOGE(TAG, "invalid ip addr");
                return false;
        }
        ESP_LOGI(TAG, "connecting to %s:%d", SERVER_ADDR, SERVER_PORT);
        conn = netconn_new(NETCONN_TCP);
        if (!conn) {
                ESP_LOGE(TAG, "netconn_new err");
                return false;
        }
        err_t err = netconn_connect(conn, &server_addr, SERVER_PORT);
        if (err != ERR_OK) {
                ESP_LOGE(TAG, "netconn_connect err, err:%d", err);
                netconn_close(conn);
                netconn_delete(conn);
                conn = NULL;
                return false;
        }
        ESP_LOGI(TAG, "netconn established");
        return true;
}
*/

/*
void sendLrs(void* msg) {
        //Frame* f = findFrame(hdr(msg)->fid);
        //assert(f);
        sendMsg(msg);
        //xSemaphoreTake(f->mutex, portMAX_DELAY);
        //f->state = FRAME_STATE_WAIT_FOR_HDS_REQ;
        //xSemaphoreGive(f->mutex);
        vTaskDelete(NULL);
}

void sendHds(void* msg) {
        Frame* f = findFrame(hdr(msg)->fid);
        assert(f);
        sendMsg(msg);
        xSemaphoreTake(f->mutex, portMAX_DELAY);
        f->state = FRAME_STATE_EMPTY;
        xSemaphoreGive(f->mutex);
        vTaskDelete(NULL);
}
*/

///////////////////////////////////////////////////////////////////////////////

/*
void initFramePool() {
        for (int i = 0; i < FramePoolSize; i++) {
                frame[i].sample = spiram_alloc(FrameSize);
                frame[i].fid = 0;
        }
}

void freeFramePool() {
        for (int i = 0; i < FramePoolSize; i++) {
                free(frame[i].sample);
        }
}

inline Frame* findFrame(u32_t fid) {
        for (int i = 0; i < FramePoolSize; i++) {
                if (frame[i].state != FRAME_STATE_EMPTY
                 && frame[i].fid == fid)
                        return &frame[i];
        }
        return NULL;
}

inline void increaseTxQue() {
        xSemaphoreTake(tx_q_mutex, portMAX_DELAY);
        tx_q++;
        xSemaphoreGive(tx_q_mutex);
}

inline void decreaseTxQue() {
        xSemaphoreTake(tx_q_mutex, portMAX_DELAY);
        tx_q--;
        xSemaphoreGive(tx_q_mutex);
}

inline int txQueSize() {
        xSemaphoreTake(tx_q_mutex, portMAX_DELAY);
        int qs = tx_q;
        xSemaphoreGive(tx_q_mutex);
        return qs;
}

///////////////////////////////////////////////////////////////////////////////

bool sendMsg(char* msg) {
        err_t err = netconn_write(conn, msg, getMsgLen(msg), NETCONN_NOCOPY);
        if (err != ERR_OK) {
                ESP_LOGE(TAG, "send %s failed", getMsgCodeStr(msg));
                conn_ok = false;
                return false;
        }
        ESP_LOGI(TAG, "sent %s of %ld bytes", getMsgCodeStr(msg),
                                              getMsgLen(msg));
        return true;
}

bool recvMsg() {
        char* msg = rx_buf;
        u32 msg_len = 0;
        u32 bytes_recv = 0;
        bool hdr_ok = false;
        struct netbuf* nb = NULL;
        void* data = NULL;
        u16_t len = 0;
        while (!hdr_ok || bytes_recv < msg_len) {
                if (netconn_recv(conn, &nb) != ERR_OK
                 || netbuf_data(nb, &data, &len) != ERR_OK) {
                        ESP_LOGE(TAG, "recvMsg: recv/data err");
                        conn_ok = false;
                        return NULL;
                }
                memcpy(msg + bytes_recv, data, len);
                bytes_recv += len;
                if (!hdr_ok && bytes_recv >= MsgHdrLen) {
                        msg_len = getMsgLen(msg);
                        hdr_ok = true;
                }
                netbuf_delete(nb);
        }
        ESP_LOGI(TAG, "recved %s of %ld bytes", getMsgCodeStr(msg),
                                                getMsgLen(msg));
        return msg;
}

///////////////////////////////////////////////////////////////////////////////

struct sockaddr_in udp_addr_server;
int    udp_sock_rx;
int    udp_sock_tx;

bool sendBytesToUdp(int sock, struct sockaddr_in* ra, char* p, int len)
{
        char* p_cur = p;
        char* p_end = p+len;
        while (p_cur != p_end) {
                int n = sendto(sock, p_cur, p_end-p_cur, 0, (struct sockaddr*)ra,
                                                            sizeof(struct sockaddr));
                if (n <= 0) {
                        ESP_LOGE(TAG, "sendBytesToUdp err: %d", errno);
                        return false;
                }
                p_cur += n;
        }
        return true;
}

bool sendMsgToUdp(int sock, struct sockaddr_in* ra, char* msg)
{
        if (!sendBytesToUdp(sock, ra, msg, getMsgLen(msg))) {
                ESP_LOGE(TAG, "[%08ld] udp sent %s failed", getMsgFid(msg),
                                                            getMsgCodeStr(msg));
                return false;
        }
        #ifdef VERBOSE
        ESP_LOGI(TAG, "[%08ld] udp sent %s of %ld bytes", getMsgFid(msg),
                                                          getMsgCodeStr(msg),
                                                          getMsgLen(msg));
        #endif
        return true;
}

bool recvBytesFromUdp(int sock, char* p, int len)
{
        struct sockaddr sa = {0};
        socklen_t addrlen = sizeof(sa);
        char* p_cur = p;
        char* p_end = p + len;
        while (p_cur != p_end) {
                int n = recvfrom(sock, p_cur, p_end-p_cur, 0, &sa, &addrlen);
                if (n <= 0) {
                        ESP_LOGE(TAG, "recvBytesFromUdp err: %d", errno);
                        return false;
                }
                p_cur += n;
        }
        return true;
}

bool recvMsgFromUdp(int sock, char* buf, int buf_size)
{
        if (!recvBytesFromUdp(sock, buf, MsgHdrLen))
                goto err;
        assert(getMsgLen(buf) <= buf_size);
        if (!recvBytesFromUdp(sock, buf+MsgHdrLen, getMsgLen(buf)-MsgHdrLen))
                goto err;
        #ifdef VERBOSE
        ESP_LOGI(TAG, "[%08ld] udp recved %s of %ld bytes", getMsgFid(buf),
                                                            getMsgCodeStr(buf),
                                                            getMsgLen(buf));
        #endif
        return true;
err:
        ESP_LOGE(TAG, "udp recv failed");
        return false;
}

bool estUdpConn()
{
        udp_sock_tx = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        udp_sock_rx = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (udp_sock_tx < 0 || udp_sock_rx < 0) {
                ESP_LOGE(TAG, "estUdpConn: create socket err: %d", errno);
                return false;
        }

        socklen_t addrlen = sizeof(struct sockaddr_in);
        struct sockaddr_in sa = {0};
        sa.sin_family = AF_INET;
        sa.sin_addr.s_addr = htonl(INADDR_ANY);
        sa.sin_port = htons(UDP_PORT);
        int err = bind(udp_sock_rx, (struct sockaddr*)&sa, addrlen);
        if (err < 0) {
                ESP_LOGE(TAG, "estUdpConn: rx socket bind err: %d", errno);
                return false;
        }

        memset(&udp_addr_server, 0, addrlen);
        udp_addr_server.sin_family = AF_INET;
        udp_addr_server.sin_addr.s_addr = inet_addr(SERVER_IP);
        udp_addr_server.sin_port = htons(UDP_PORT);

        ESP_LOGI(TAG, "estUdpConn: udp sockets created");
        return true;

}
*/

/*
ESP_LOGE(TAG, "hds req: fid %ld, n_req %d, n_req_samples %d",
        req_fid,
        req_hdr->n_reqs,
        req_hdr->n_req_samples);
read_ptr = req_msg + MsgHdrLen + HdsReqHdrLen;
n_samples_wrote = 0;
for (u16 n = 0; n < req_hdr->n_reqs; n++) {
        decHdsReq(read_ptr, &r);
        read_ptr += HdsReqCodeLen;
        i16 ts = r.sample_idx;
        i16 te = r.sample_idx + SpikeLen;
        if (te > FrameHeight) te = FrameHeight;
        if (ts < 0) ts = 0;
        u8 lrs_sampling_step = lrs_map.step[r.channel];
        if (lrs_sampling_step == 0) {
                for (u16 t = ts; t < te; t += r.step)
                        n_samples_wrote++;
        } else {
                for (u16 t = ts; t < te; t += r.step) {
                        if (t % lrs_sampling_step == 0)
                                continue;
                        n_samples_wrote++;
                }
        }
        ESP_LOGE(TAG, "%3d %+4d %3d %2d %+3d %+3d %2d %4d",
                n,
                r.sample_idx, r.channel, r.step,
                ts, te,
                lrs_sampling_step,
                n_samples_wrote);
}
assert(false);
*/

/*
#define DATAPIPE_MOUNT_POINT "/sdcard"
const char* signal_path = DATAPIPE_MOUNT_POINT"/DATA_RAW.DAT";
#define DATAPIPE_BUF_SIZE 4096
char datapipe_buf[DATAPIPE_BUF_SIZE];
FILE* fp_signal = NULL;

void initSD() {
        ESP_LOGI(TAG, "initializing SD card");
        sdmmc_card_t* card = NULL;
        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        host.max_freq_khz = SDMMC_FREQ_52M;
        sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
        slot_config.width = 4;
        slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .max_files = 1,
            .allocation_unit_size = 16384
        };
        ESP_ERROR_CHECK(
                esp_vfs_fat_sdmmc_mount(DATAPIPE_MOUNT_POINT,
                        &host, &slot_config, &mount_config, &card)
        );
        ESP_LOGI(TAG, "filesystem mounted");
        sdmmc_card_print_info(stdout, card);
        ESP_LOGI(TAG, "opening %s", signal_path);
        assert(fp_signal = fopen(signal_path, "rb"));
        fclose(fp_signal);

        ESP_LOGI(TAG, "start reading");
        u64 start = esp_timer_get_time();
        u32 bytes_read = 0;
        u32 i = 0;
        while (true) {
                u32 n = fread(datapipe_buf, 1, DATAPIPE_BUF_SIZE, fp_signal);
                if (n != DATAPIPE_BUF_SIZE)
                        break;
                bytes_read += n;
                if (++i % 100 == 0) {
                        u32 t = (esp_timer_get_time() - start) / 1000; // ms
                        ESP_LOGI(TAG, "speed = %ld KBps", bytes_read / t);
                }
        }
        fclose(fp_signal);
}
*/