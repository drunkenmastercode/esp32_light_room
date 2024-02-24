#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include <DNSServer.h>
#include <AsyncElegantOTA.h>
#include <esp_now.h>
#include "AsyncTCP.h"


#define DEBUG_UART  1   //  <DEBUG_UART> чтобы отключить вывод отладочных сообщений в UART
#define NUM_RELAYS  8   //  Количество подключенных реле
#define SET   1
#define RESET 0

/* 
* Массив Реле № {0, 1, 2, 3, 4, 5, 6, 7 }
* в фигурных скобках № GPIO к которому подключен вывод управления реле
*/
int relayGPIOs[NUM_RELAYS] = { 23, 22, 21, 19, 18, 5, 17, 16 };

uint8_t macAddSlave[6];                   //  массив МАС адреса slave платы
#define nSSID_AP "HomeLightServ"          // SSID и пароль для точки доступа которую создает ESP-плата
#define pwd_ap "light@server"
const char *server_name = "light.local";  //  адрес http://light.local/ по которому внутренний DNS сервер будет откличкаться при открытии главной страницы
const byte DNS_PORT = 53;                 // порт DNS
IPAddress apIP(192, 168, 4, 1);           // IP платы по-умолчанию при подключении к ее точке доступа

// переменные для объявления SSID и пароля точки доступа, настраиваются выше в #DEFINE
const char* ssid_ap = nSSID_AP;
const char* password_ap = pwd_ap;
// логин и пароль домашней сети WiFi
const char* ssid = "Pentagon";
const char* password = "inetpass00";
// переменные передаваемые с web страницы
const char* PARAM_INPUT_1 = "output";
const char* PARAM_INPUT_2 = "state";
//  Логин и пароль для открытия страницы /update и обновления платы "по воздуху"
const char* www_username = "admin";
const char* www_password = "otaadmin";

AsyncWebServer server(80);
DNSServer dnsServer;

/*
* структура передаваемого сообщения 
* [0] байт - идентификатор светильника
* [1] байт - состояние в которое необходимо перевести GPIO вывод отвечающий id
*/
typedef struct {
  uint8_t idLight;    // 
  uint8_t DataMsg;    //
} esp_msg;

esp_msg EspDataMsg;   // объявление структуры


// Объявляем прототипы используемых функций
String processor(const String& var);
String outputState(int gpio_id);
void gpio_init(void);
void notFound(AsyncWebServerRequest *request);
void InitESP_NOW(void); 
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void printMAC(const uint8_t * mac_addr);
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
// end


const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 3.0rem;}
    p {font-size: 3.0rem;}
    body {max-width: 600px; margin:0px auto; padding-bottom: 25px;}
    .switch {position: relative; display: inline-block; width: 120px; height: 68px} 
    .switch input {display: none}
    .slider {position: absolute; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; border-radius: 34px}
    .slider:before {position: absolute; content: ""; height: 52px; width: 52px; left: 8px; bottom: 8px; background-color: #fff; -webkit-transition: .4s; transition: .4s; border-radius: 68px}
    input:checked+.slider {background-color: #2196F3}
    input:checked+.slider:before {-webkit-transform: translateX(52px); -ms-transform: translateX(52px); transform: translateX(52px)}
  </style>
</head>

<body>
  <h2>ESP32 Сервер освещения</h2>
  <span id="upd_status">
  %BUTTONPLACEHOLDER%
  </span>
<script>
function toggleCheckbox(element) {
  var xhr = new XMLHttpRequest();
  if(element.checked) { 
    xhr.open("GET", "/relay?output="+element.id+"&state=1", true); 
    }
  else { 
    xhr.open("GET", "/relay?output="+element.id+"&state=0", true); 
    }
  xhr.send();
}

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
      document.getElementById("upd_status").innerHTML = this.responseText;
  };
  location.reload();
}, 30000 ) ;

</script>
</body>
</html>
)rawliteral";

/*
* Функция создания переключателей на web-странице 
*  id=\"0\" - идентификатор переключателя
*  outputState - вызов функции для получения состояния
*/
String processor(const String& var){
  if(var == "BUTTONPLACEHOLDER"){
    String buttons ="";
      buttons += "<h4>Светильник 1 </h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"0\" " + outputState(0) + "><span class=\"slider\"></span></label>";
      buttons += "<h4>Светильник 2 </h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"1\" " + outputState(1) + "><span class=\"slider\"></span></label>";
      buttons += "<h4>Светильник 3 </h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"2\" " + outputState(2) + "><span class=\"slider\"></span></label>";
      buttons += "<h4>Светильник 4 </h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"3\" " + outputState(3) + "><span class=\"slider\"></span></label>";
      buttons += "<h4>Светильник 5 </h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"4\" " + outputState(4) + "><span class=\"slider\"></span></label>";
      buttons += "<h4>Светильник 6 </h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"5\" " + outputState(5) + "><span class=\"slider\"></span></label>";
      buttons += "<h4>Светильник 7 </h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"6\" " + outputState(6) + "><span class=\"slider\"></span></label>";
      buttons += "<h4>Светильник 8 </h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"7\" " + outputState(7) + "><span class=\"slider\"></span></label>";
      return buttons;
  }
  return String();
}

/* 
* Функция получения состояния конкретного GPIO-вывода и возврат в функцию <processor>
*/
String outputState(int gpio_id) {
    if(digitalRead(relayGPIOs[gpio_id])){
      return "checked";
    }
    else {
      return "";
    }
}

/*
* Инициализация GPIO- выводов
* Изначальное состояние "0"(LOW) 
* Для смены состояния сменить в digitalWrite LOW на HIGH
*/
void gpio_init(void) {
  for(int i = 0; i < NUM_RELAYS; i++){
    pinMode(relayGPIOs[i], OUTPUT);
    digitalWrite(relayGPIOs[i], LOW);
    }
}
/*
* Ответ при обращение к странице которой не существует
*/
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "404 page not found");
}

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
    Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Success to: " : "Fail to: ");
    printMAC(mac_addr);
    Serial.println();
#endif
}


// ---------------------------- вызываемая функция печати MAC адреса ----------------
void printMAC(const uint8_t * mac_addr) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  #if DEBUG_UART
    Serial.print(macStr);
  #endif
}

//---------------------------- Вызываемая функция когда принято сообщение платой------
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    memcpy(&EspDataMsg, incomingData, sizeof(EspDataMsg));              // копируем принятое сообщение в переменную <EspDataMsg> типа структура
    switch (EspDataMsg.idLight) {                                         //  выбираем из id светильника полученного в сообщении 
      case 0:
        digitalWrite(relayGPIOs[EspDataMsg.idLight], EspDataMsg.DataMsg); // переводим GPIO вывод в соответсвующее состояние 
        break;
      case 1:
        digitalWrite(relayGPIOs[EspDataMsg.idLight], EspDataMsg.DataMsg); // переводим GPIO вывод в соответсвующее состояние
        break;
      case 2:
        digitalWrite(relayGPIOs[EspDataMsg.idLight], EspDataMsg.DataMsg); // переводим GPIO вывод в соответсвующее состояние
        break;
      case 3:
        digitalWrite(relayGPIOs[EspDataMsg.idLight], EspDataMsg.DataMsg); // переводим GPIO вывод в соответсвующее состояние
        break;
      case 4:
        digitalWrite(relayGPIOs[EspDataMsg.idLight], EspDataMsg.DataMsg); // переводим GPIO вывод в соответсвующее состояние
        break;
      case 5:
        digitalWrite(relayGPIOs[EspDataMsg.idLight], EspDataMsg.DataMsg); // переводим GPIO вывод в соответсвующее состояние
        break;
      case 6:
        digitalWrite(relayGPIOs[EspDataMsg.idLight], EspDataMsg.DataMsg); // переводим GPIO вывод в соответсвующее состояние
        break;
      case 7:
        digitalWrite(relayGPIOs[EspDataMsg.idLight], EspDataMsg.DataMsg); // переводим GPIO вывод в соответсвующее состояние
        break;
      case 8:
        digitalWrite(relayGPIOs[EspDataMsg.idLight], EspDataMsg.DataMsg); // переводим GPIO вывод в соответсвующее состояние
        break;
      default:
        // сюда никогда не должно зайти
        break;
    }
}


void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP_STA);                                                   // смешанный режим - точка доступа и подключаемся к Wi-Fi с указанными настройками SSID 
  WiFi.softAP(ssid_ap, password_ap, 7);                                     // Точка доступа на канале 7 
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));               // настройка точки доступа платы

  WiFi.begin(ssid, password);                                               // Подключаемся к своему WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  dnsServer.start(DNS_PORT, server_name, apIP);                             // запускаем DNS-server
  Serial.println(WiFi.localIP());
  gpio_init();                                                              //  инициализаруем GPIO для управления реле

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){              //  обработка события "/" ассинхронного веб-сервера, т.е. открыть главную страницу
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/relay", HTTP_GET, [] (AsyncWebServerRequest *request) {       //  обработка события "/relay" ассинхронного веб-сервера 
    String inputMessage;
    String inputMessage2;
    if (request->hasParam(PARAM_INPUT_1) & request->hasParam(PARAM_INPUT_2)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();                    // получаем <element.id>
      inputMessage2 = request->getParam(PARAM_INPUT_2)->value();                   // получаем <state>
        digitalWrite(relayGPIOs[inputMessage.toInt()], inputMessage2.toInt());     // устанавливаем на вывод GPIO уровень в зависимости от полученного сообщения - digitalWrite(GPIO, LOW/HIGH)
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });

  server.onNotFound(notFound);
  AsyncElegantOTA.begin(&server, www_username, www_password);     // запуск обновления по воздуху с заданными логином и паролем
  server.begin();
}

void loop() {
  dnsServer.processNextRequest();     
}
