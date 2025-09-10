/**
 * @file demo2.cpp
 * @author LDRobot + FARAD
 * @brief  LD LiDAR verilerini alıp ESP32'ye TCP üzerinden gönderen demo
 * @version 0.3
 * @date 2025-09-03 (güncellendi)
 */

#include "ldlidar_driver.h"
#include <netinet/tcp.h>   // TCP_NODELAY için

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <cstring>
#include <vector>
#include <iostream>
#include <chrono>
#include <sstream>
#include <cmath>
#include <cstdio>

// -------------------------------
// Ayarlar (kendi ESP IP/port'unuzu buraya koyun)
// -------------------------------
static const char* ESP_IP   = "10.97.240.202";
static const int   ESP_PORT = 2000;

// Döngü gecikmesi (mikrosaniye). 50ms = 50.000
static const useconds_t LOOP_DELAY_US = 100; // 50 ms

// Açı penceresi (hedef açı ± WINDOW derece içindeki noktaları değerlendir)
static const double ANGLE_WINDOW = 0.8; // derece

// -------------------------------
// Timestamp fonksiyonu
// -------------------------------
uint64_t GetSystemTimeStamp(void) {
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp =
        std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
    return ((uint64_t)tmp.count());
}

// -------------------------------
// Küçük yardımcılar
// -------------------------------
// Dairesel açı farkı (0..180)
static double circularDiffDeg(double a, double b) {
    double d = fabs(a - b);

   // std::cout<< d << std::endl;
    
    if (d > 180.0) d = 360.0 - d;
    return d;
}

// -------------------------------
// TCP client fonksiyonları
// -------------------------------
int connectToESP(const char* esp_ip, int port) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation error");
        return -1;
    }

    // Set a reasonable send timeout (optional)
    struct timeval tv;
    tv.tv_sec = 3; tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof tv);

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, esp_ip, &serv_addr.sin_addr) <= 0) {
        perror("Invalid address / Address not supported");
        close(sock);
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection Failed");
        close(sock);
        return -1;
    }

    // disable Nagle (gönderim gecikmelerini azaltır)
    int flag = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(int));

    return sock;
}

// tüm veriyi gönderene kadar döngüsel gönder (kısmi gönderimler için)
bool sendAll(int sock, const char* data, size_t len) {
    size_t total = 0;
    const char* p = data;
    while (total < len) {
        ssize_t sent = send(sock, p + total, len - total, 0);
        if (sent <= 0) {
            return false;
        }
        total += (size_t)sent;
    }
    return true;
}

bool sendLineToESP(int sock, const std::string& line) {
    if (sock < 0) return false;
    // satır + newline
    std::string out = line;
    if (out.empty()) return true;
    if (out.back() != '\n') out.push_back('\n');
    return sendAll(sock, out.c_str(), out.size());
}

// -------------------------------
// LiDAR başlatma fonksiyonu
// -------------------------------
ldlidar::LDLidarDriver* initLidar(int argc, char **argv) {
    if (argc < 4) {
        LDS_LOG_WARN("Terminal >>: ./ldlidar_stl_node <product_name> serialcom <serial_number>", "");
        LDS_LOG_WARN("or", "");
        LDS_LOG_WARN("Terminal >>: ./ldlidar_stl_node <product_name> networkcom_tcpclient <server_ip> <server_port>", "");
        exit(EXIT_FAILURE);
    }

    std::string product_name(argv[1]);
    std::string communication_mode(argv[2]);
    std::string port_name;
    std::string server_ip;
    std::string server_port;
    uint32_t serial_baudrate = 0;
    ldlidar::LDType type_name;

    if (communication_mode == "serialcom") {
        if (argc != 4) { exit(EXIT_FAILURE); }
        port_name = argv[3];
    } else if (communication_mode == "networkcom_tcpclient") {
        if (argc != 5) { exit(EXIT_FAILURE); }
        server_ip = argv[3];
        server_port = argv[4];
    }

    ldlidar::LDLidarDriver* node = new ldlidar::LDLidarDriver();
    LDS_LOG_INFO("LDLiDAR SDK Pack Version is %s", node->GetLidarSdkVersionNumber().c_str());
    node->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp));
    node->EnableFilterAlgorithnmProcess(true);

    if (product_name == "LD06") {
        serial_baudrate = 230400;
        type_name = ldlidar::LDType::LD_06;
    } else if (product_name == "LD19") {
        serial_baudrate = 230400;
        type_name = ldlidar::LDType::LD_19;
    } else {
        LDS_LOG_ERROR("input <product_name> is error!", "");
        exit(EXIT_FAILURE);
    }

    bool start_ok = false;
    if (communication_mode == "serialcom") {
        start_ok = node->Start(type_name, port_name, serial_baudrate, ldlidar::COMM_SERIAL_MODE);
    } else {
        start_ok = node->Start(type_name, server_ip.c_str(), server_port.c_str(), ldlidar::COMM_TCP_CLIENT_MODE);
    }

    if (!start_ok) {
        LD_LOG_ERROR("ldlidar node start is fail", "");
        exit(EXIT_FAILURE);
    }

    if (node->WaitLidarCommConnect(3500)) {
        LDS_LOG_INFO("ldlidar communication is normal.", "");
    } else {
        LDS_LOG_ERROR("ldlidar communication is abnormal.", "");
        node->Stop();
        exit(EXIT_FAILURE);
    }

    return node;
}

// -------------------------------
// Bir hedef açı için en iyi (geçerli) mesafeyi seç (distance>0 olan, açı farkı en küçük)
// -------------------------------
bool pickBestForAngle(const ldlidar::Points2D& pts, double targetAngle, int& outDistance) {
    double bestDelta = 1e9;
    int bestDist = -1;
    int bestIntensity = -1;

    for (const auto& p : pts) {
        // ignore zero distance (yanlış/boş okumalar)
        if (p.distance == 0) continue;

        double ang = p.angle;
        double delta = circularDiffDeg(ang, (targetAngle == 360.0 ? 0.0 : targetAngle));
        if (delta <= 2) {
            // tercih: daha küçük delta, eğer delta eşitse daha büyük intensity
            if (delta < bestDelta || (fabs(delta - bestDelta) < 1e-6 && p.intensity > bestIntensity)) {
                bestDelta = delta;
                bestDist = p.distance;
                bestIntensity = p.intensity;
            }
        }
    }

    if (bestDist >= 0) {
        outDistance = bestDist;
        return true;
    }
    return false;
}

// -------------------------------
// Bir turu ölç ve "90:..,180:..,270:..,360:..\n" şeklinde satır hazırla.
// Eksik açılar varsa o çiftleri atlar.
// -------------------------------
std::string buildLine(ldlidar::LDLidarDriver* node) {
    ldlidar::Points2D pts;
    if (node->GetLaserScanData(pts, 1500) != ldlidar::LidarStatus::NORMAL) {
        return "";
    }

    int d90=-1, d180=-1, d270=-1, d360=-1;
    pickBestForAngle(pts, 90.0,  d90);
    pickBestForAngle(pts, 180.0, d180);
    pickBestForAngle(pts, 270.0, d270);
    pickBestForAngle(pts, 360.0, d360);

    std::ostringstream oss;
    bool first = true;
    auto addKV = [&](int angle, int dist) {
        if (dist >= 0) {
            if (!first) oss << ",";
            oss << angle << ":" << dist;
            first = false;
        }
    };

    addKV(90, d90);
    addKV(180, d180);
    addKV(270, d270);
    addKV(360, d360);

    if (first) return ""; // hiçbir veri yok
    std::string s = oss.str();
    s.push_back('\n');
    return s;
}

// -------------------------------
// Ana program
// -------------------------------
int main(int argc, char **argv) {
    ldlidar::LDLidarDriver* node = initLidar(argc, argv);

    // ESP'ye bağlanmayı dene
    int esp_sock = connectToESP(ESP_IP, ESP_PORT);
    if (esp_sock < 0) {
        LDS_LOG_WARN("ESP32'e bağlanılamadı; döngü devam ederken periyodik olarak yeniden denenecek.", "");
    } else {
        LDS_LOG_INFO("ESP32'ye TCP ile bağlandı.", "");
    }

    while (ldlidar::LDLidarDriver::IsOk()) {
        std::string line = buildLine(node);
        if (!line.empty()) {
            // eğer socket yok ise yeniden bağlanmayı dene
            if (esp_sock < 0) {
                esp_sock = connectToESP(ESP_IP, ESP_PORT);
                if (esp_sock >= 0) LDS_LOG_INFO("ESP32 yeniden bağlandı.", "");
            }

            if (esp_sock >= 0) {
                bool ok = sendLineToESP(esp_sock, line);
                if (!ok) {
                    LDS_LOG_WARN("ESP32'ye gönderilemedi, bağlantı kapatılıyor ve tekrar denenecek.", "");
                    close(esp_sock);
                    esp_sock = -1;
                } else {
                    // send başarılı ise log (isteğe bağlı)
                    LDS_LOG_INFO("Sent to ESP: %s", line.c_str());
                }
            } else {
                // eğer bağlanamamışsak sadece loglayalım
                LDS_LOG_INFO("Line hazır (ESP bağlı değil): %s", line.c_str());
            }
        }

        usleep(LOOP_DELAY_US);
    }

    if (esp_sock >= 0) close(esp_sock);
    node->Stop();
    delete node;
    return 0;
}
