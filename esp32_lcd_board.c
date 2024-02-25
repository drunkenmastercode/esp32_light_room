#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include <DNSServer.h>
#include <AsyncElegantOTA.h>
#include <esp_now.h>
#include "AsyncTCP.h"
#include <TickerScheduler.h>  


#define DEBUG_UART  1   //  <DEBUG_UART> чтобы отключить вывод отладочных сообщений в UART
#define SET   1
#define RESET 0

uint8_t broadcastAddress[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };   // BOARD_SLAVE - МАС релейной платы 

#define nSSID_AP "OTAupdDesk"          // SSID и пароль для точки доступа которую создает ESP-плата
#define pwd_ap "light@desk"
const char *server_name = "desk.local";  //  адрес http://light.local/ по которому внутренний DNS сервер будет откличкаться при открытии главной страницы
const byte DNS_PORT = 53;                 // порт DNS
IPAddress apIP(192, 168, 4, 1);           // IP платы по-умолчанию при подключении к ее точке доступа

// переменные для объявления SSID и пароля точки доступа, настраиваются выше в #DEFINE
const char* ssid_ap = nSSID_AP;
const char* password_ap = pwd_ap;

//  Логин и пароль для открытия страницы /update и обновления платы "по воздуху"
const char* www_username = "admin";
const char* www_password = "otaadmin";

AsyncWebServer server(80);
DNSServer dnsServer;
IPAddress ipBoard;


typedef enum {
  TM,
  DATA,
  CMD,
} eIdMsg;

/*
* структура передаваемого сообщения 
* [0] байт - id сообщения: 0 - обычное сообщение; 1 - сообщение с указанием IP адреса платы 
* [1] байт - идентификатор светильника
* [2] байт - состояние в которое необходимо перевести GPIO вывод отвечающий id
*/
typedef struct {
  uint8_t idMsg;      // id сообщения: 0 - обычное сообщение; 1 - сообщение с указанием IP адреса платы 
  uint8_t idLight;    // 
  uint8_t DataMsg;    //
  uint8_t BoardIP[4];
} esp_msg;

esp_msg EspDataMsg;       // объявление структуры принимаемых сообщений
esp_msg outgoingMessage;  // объявление структуры отправляемых сообщений

// массив для опеределения статуса светильника
uint8_t LampStatus[8] = { RESET, RESET, RESET, RESET, RESET, RESET, RESET, RESET };

// Объявляем прототипы используемых функций
void InitESP_NOW(void); 
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void InitWiFi(void);
void SendCmd (uint8_t idLamp, uint8_t StatusLamp);
// end


/*
* Функция инициализации протокола ESP-NOW
*/
void InitESP_NOW(void) {
    if (esp_now_init() != RESET) {
#if DEBUG_UART
      Serial.printf("Error init ESP-NOW \n");
#endif
      return;
    }
    else {
#if DEBUG_UART
      Serial.printf("Init ESP-NOW \n");
#endif
    }
    esp_now_register_send_cb(onDataSent); // событие отправкии сообщения и вызываемая им функция
    esp_now_register_recv_cb(onDataRecv); // событие получения сообщения и вызываемая им функция
}

//-----------------callback когда сообщение отправлено платой-----------------------
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#if DEBUG_UART
    Serial.print("Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success " : "Fail ");
#endif
    /*
          сюда можно дописать вывод на дисплей сообщения:
          - если пакет с командой от платы не был доставлен на релейную плату
          - выводить статус "ОК" при успешной отправке 
    */
}


//---------------------------- Вызываемая функция когда принято сообщение платой------
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    memcpy(&EspDataMsg, incomingData, sizeof(EspDataMsg));                // копируем принятое сообщение в переменную <EspDataMsg> типа структура
    uint8_t id = EspDataMsg.idMsg;
    switch (id) {
      case TM:         // принято сообщение с данными телеметрии
        LampStatus[EspDataMsg.idLight] = EspDataMsg.DataMsg;
        break;

      case DATA:         // принято сообщение с данными IP адреса релейной платы (ipBoard[0].ipBoard[1].ipBoard[2].ipBoard[3])
        ipBoard = EspDataMsg.BoardIP;
        Serial.printf("%d.%d.%d.%d", ipBoard[0],ipBoard[1],ipBoard[2],ipBoard[3]);
        break;

      default:
        break;
    }
}


void SendCmd(uint8_t idLampGroup, uint8_t StatusLamp) {
    outgoingMessage.idMsg = CMD;
    outgoingMessage.idLight = idLampGroup;
    outgoingMessage.DataMsg = StatusLamp;
    esp_now_send(broadcastAddress, (uint8_t *) &outgoingMessage, sizeof(outgoingMessage));
}

void InitWiFi(void) {
  WiFi.mode(WIFI_AP);                                                       // режим - точка доступа 
  WiFi.softAP(ssid_ap, password_ap, 7);                                     // Точка доступа на канале 7 
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));               // настройка точки доступа платы
  dnsServer.start(DNS_PORT, server_name, apIP);                             // запускаем DNS-server
}

void setup() {
  Serial.begin(115200);
  InitWiFi();
  InitESP_NOW();                                                  //  инициализаруем ESP-NOW
  AsyncElegantOTA.begin(&server, www_username, www_password);     // запуск обновления по воздуху с заданными логином и паролем
  server.begin();
}

void loop() {
  dnsServer.processNextRequest();     
}
