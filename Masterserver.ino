  
/**  versión 2.7   Pulling RF24  **/co
/**  usando EEPROM 24LC512  **/

#define miversion 2
#define misubver 7
#define VERMEM 1
#define ARDUINO 103

#define EN        // idioma EN:english,  ES: español
#define LOG 0

//#define SERIAL_DEB
#define WEBDUINO_SERIAL_DEBUGGING 0
#define WEBDUINO_FAIL_MESSAGE "<h1>FATAL ERROR</h1>"
//#define WEBDUINO_FAVICON_DATA { /////0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10,\                   0x10, 0x02, 0x00, 0x01, 0x00, 0x01, 0x00,\
//                                0xb0, 0x00, 0x00, 0x00, 0x16, 0x00, 0x00,\
//                                0x00, 0x28, 0x00, 0x00, 0x00, 0x10, 0x00,\
//                                0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x01,\
//                                0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,\
//                                0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,\
//                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
//                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
//                                0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,\
//                                0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0x00,\
//                                0x00, 0xff, 0xff, 0x00, 0x00, 0xcf, 0xbf,\
//                                0x00, 0x00, 0xc7, 0xbf, 0x00, 0x00, 0xc3,\
//                                0xbf, 0x00, 0x00, 0xc1, 0xbf, 0x00, 0x00,\
//                                0xc0, 0xbf, 0x00, 0x00, 0x00, 0x00, 0x00,\
//                                0x00, 0xc0, 0xbf, 0x00, 0x00, 0xc1, 0xbf,\
//                                0x00, 0x00, 0xc3, 0xbf, 0x00, 0x00, 0xc7,\
//                                0xbf, 0x00, 0x00, 0xcf, 0xbf, 0x00, 0x00,\
//                                0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0x00,\
//                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
//                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
//                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
//                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
//                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
//                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
//                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
//                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
//                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
//                                0x00, 0x00}

#include "SPI.h"
#include "Ethernet.h"
#include "WebServer.h"
#include "EEPROM.h"
#include "EEPROMAnything.h"
#include "LiquidCrystal.h"
//#include "LiquidCrystal440.h"
#include "Wire.h"

#include "DS1307RTC.h"
#include "Time.h"
#include "Base64.h"
#include "OneWire.h"
#include "RF24.h"
#include "RF24Network.h"
#include "RF24NetworkRL.h"
#include <avr/pgmspace.h>
#include "Mudbus.h"
#include "DallasTemperature.h"
#include "RemoteReceiver.h"
#include "RemoteTransmitter.h"
#include "NewRemoteReceiver.h"
#include "NewRemoteTransmitter.h"
#include "idiomas.h"
#include "SdFat.h"
#include "InterruptChain.h"
#include <DHT.h>
#include <avr/wdt.h>

volatile byte modo433Rec;
volatile long pendiente433Rec;
volatile long address433Rec;
volatile long unit433Rec;
volatile long switchType433Rec;
volatile int ninter=0;
long pendiente433RecMostrar;


#define inact 255          // valor para programación, significa inactivo
#define despMail 251       // revalor para enviar mails, de 90 a 92
#define nretryrem 20       // reintentos onoff remoto
#define delayretryrem 10   // retardo entre reintentos en ms
#define delayACKrem 50    // espera despuÃ©s de onrCmd/offrCmd para recibir el estado
#define delayACKMail 2000   // espera a respuesta de servidor smtp despuÃ©s de cada comando
/* NO USAR LOS PINES: 53, 52, 51, 50, 10,4   Ethernet shield  */
#define spiMISO 50
#define spiMOSI 51
#define spiSCK 52
#define ethSEL 10
#define sdSEL 4
#define rstFab 42    //  Pin para reset fábrica

/* 49, 47, 45, 35, 33, 31, 29   LCD  */
#define lcdRS 49
#define lcdRW 47
#define lcdEN 45
#define lcdEN2 18    // sólo para Powertip PC4004, se quita una Ent. digital
#define lcdD0 35
#define lcdD1 33
#define lcdD2 31
#define lcdD3 29
// LiquidCrystal lcd(lcdRS,lcdRW,lcdEN,lcdD0,lcdD1,lcdD2,lcdD3); 

/*   Módulo radio NRF24L01  */
#define radSEL 48
#define radCE 53
/*   Módulos radio 433 */
#define radRX 19    // este pin activa la interrupción 4
#define radTX 44

#define owPin1 43    // pin para 1-wire, puerto 1
#define owPin2 46    // pin para 1-wire, puerto 2

#define maxNodos 60  // número máximo de nodos remotos 2,4 Ghz
#define lTAB 8       // en 8 bytes se guardan variables de 64 bits
#define lTABr 24     // en 24 bytes se guardan variables de 192 bits
#define nPINES 54    // varía según la placa, 54 para Arduino MEGA
#define nANALO 16    // número máximo de entradas analÃ³gicas
#define nANALOr 20   // número máximo de entradas analÃ³gicas
#define nDIG 34      // número máximo de entradas/salidas digitales
#define nPRG 32      // número de programaciones
#define nEsc 5       // número de escenas
#define nsalporesc 86  // número de salidas por escena
#define nPRGsem 5    // programas a lso que se pueden asociar los programas semanales
#define nPRGeve 5    // programas a lso que se pueden asociar los programas de eventos
#define LEN5 5
#define LEN20 20
#define LEN40 40
#define LEN80 80
#define maxTemp 20     // máximo de sondas 1-wire
#define baseTempr 20
#define baseAna 80
#define baseAnar 96
#define maxTempr 60  
#define maxReler 60  
#define max433 30

// definición posiciones EEPROM
// hueco desde 0 hasta 50
#define dirEEmodo 50       // 1 byte, modo de presentación web, 0:Avanzado, 1:Básico, 2:Local, 3:Domótica,4:Riego
#define dirEEevenTemp 51   // 40
#define dirEEprectemp 91   // 1 byte, precisión sondas ds18b20: 9,10,11,12
#define dirEEradNOD 92     // 2
#define dirEEradACT 94     // 1
#define dirEEradFUN 95     // 1
#define dirEEreactRad 96   // 1
#define dirEEreact433 97   // 1
#define dirEEradVEL 98     // 1
#define dirEEsdHay 99      // 1
#define dirEEmac 100       // 6 MAC
#define dirEEip 106        // 4 IP
#define dirEEmask 110      // 4 Mask
#define dirEEgw 114        // Gateway
#define dirEEdns 118       // DNS
#define dirEEwebport 122   // 2 Port web
#define dirEEpass 124      // 20 password
#define dirEEactprg 144    // 5 bytes guarda que programas están activos
#define dirEEuser 149      // 20 usuario
#define dirEEpmactRem 169    // 4 bytes
// hueco desde 173 hasta 183  // 15 bytes
#define dirEEusepass 184   // 1 activar password
#define dirEEshowTemp 185  // 3 bytes, 20 temperaturas (cabrían 24=8x3)
#define dirEEfecval 188     // 8
#define dirEEpmactVar 196    // 4 bytes
#define dirEEnumIA 200     // 1   
#define dirEEnumID 201     // 1
#define dirEEnumOD 202     // 1
#define dirEEshown 203     // 1 mostrar número de pin entre parÃ©ntesis
#define dirEEperact 204    // 1 período de actualizaciÃ³n en segundos de pantalla 
#define dirEEradCan 213    // 9 tabla de red-canal, 9 posibles redes
#define dirEErshow433 222  // 8 tabla de 8 bytes para 64 dispositivos de 433, mostrar sí/no
#define dirEEradACT433 230  // 1
#define dirEEremPin 231    // 120 (MaxRem) pines de la E/S remotas (20 ANA, 20 EDIG, 20 SDIG, 60 RelÃ©s)
#define dirEEestado 351     // 32 bytes, se guarda el estado de 64 pines locales y 192 remotos
#define dirEEbshowLink 383  // 1 byte
// hueco desde 384 hasta 385 (2 bytes)
#define dirEEdescpinD 410   // descripción de pines locales 34=680
#define dirEEdescpinA 1090   // descripción de pines locales 16x20=320
#define dirEEremTpin 1410  // orden sonda remota en un nodo remoto 60
#define dirEEfecdia 1470     // 32
// hueco desde 1502 hasta 1505  4 bytes
#define dirEEremNodo 1506   // 240 tabla de nodos remotos, maxRem nodos (120)
//hueco desde 1746 hasta 1809  // 
#define dirEEevenvalD 1810   // 64 bytes
#define dirEEtipo433 1842    // 1 x 40 bytes=40 bytes
#define dirEEbprgtipo 1882   // 4 bytes, tipo señal salida en prg semana
#define dirEEbeveacttipo 1886   // 4 bytes, tipo señal activadora en prg evento
#define dirEEbevesaltipo 1890   // 4 bytes, tipo señal salida en prg evento
#define dirEEtipoLCD 1894       // 1 byte
#define dirEEtipoED 1895        // 16 bytes tipo de entrada digital
// hueco desde 1911 hasta 1958, 49 bytes
#define dirEEfecmin 1959    // 32  fecha, minuto   
#define dirEEevenact 1991   // 32 señales activadoras del evento
#define dirEEevensal 2023   // 32 bytes, pin sobre el que se actÃºa en eventos
#define dirEEfecsal 2055    // 32
#define dirEEbfectipo 2087   // 4 bytes, tipo señal salida en prg fecha
// hueco desde 2091 hasta 2092, 2 bytes
#define dirEEevenniv 2093   // 8 bytes, se guarda bevenniv, nivel de activaciÃ³n del evento
#define dirEEest433 2101    // 4 bytes estado dispositivos 433 Mhz
// hueco desde 2105 hasta 2108, 4 bytes
#define dirEEredNodo 2109   // 120, red a la que pertenece cada nodo
#define dirEEdescTemp 2229  // 400, 20x20 descripción de las sondas 1-wire
#define dirEEbshowpin 2629  // 8 bytes, Panel Principal locales
// hueco desde 2637 hasta 2639
#define dirEEremshowpin 2640 // 24 bytes, Panel Principal, pines remotos
#define dirEEbactsem 2664      // 4 bytes programa semanal activo
#define dirEEbacteve 2668      // 4 bytes programa eventos activo
#define dirEEbactfec 2672      // 4 bytes programa fechas activo
#define dirEEbshowEsc 2676       // 1 bytes
// hueco desde 2677 hasta 2682 (6 bytes)
#define dirEEprgsal 2683     // 32   
#define dirEEfecano 2715     // 32
#define dirEEprgval 2747     // 64  
#define dirEEprgdia 2811     // 32 
#define dirEEprghor 2843     // 32
#define dirEEprgmin 2875     // 32
#define dirEEprgcon 2907     // 32
#define dirEEprgconv 2939    // 64 bytes
#define dirEEprgcomp 3003    // 32
#define dirEEnodotempr 3035  // nodos de las sondas remotas, 60x2=120
#define dirEEaddr433 3155    // 4 x 30 = 120
#define dirEEunit433 3315    // 1  30 = 30 bytes
#define dirEEfechor 3355     // 32
#define dirEEmailACT 3387    // 1 activar envío de correo
#define dirEEmailSVR 3388    // 40 servidor de correo
#define dirEEmailPER 3428    // 1 período de envío de correo en minutos
#define dirEEmailPORT 3429   // 2 puerto del servidor de correo
#define dirEEmailUSER 3431   // 40 usuario correo
#define dirEEmailPASS 3471   // 40 password correo
#define dirEEmailSENDER 3511 // 40 remitente correo
#define dirEEmailDEST 3551    // 120: 3 x 40, destinatarios correo
#define dirEEevencomp 3671   // 32 comparador eventos analógicos
#define dirEEcontadores 3703  // 32 8 contadores de 4 bytes(long) entradas digitales
#define dirEEfecmes 3735     // 32
#define dirEEcode433Off 3767  // 4 x 30=120 byter
// hueco desde 3887 hasta 4066, 180 bytes.
// hueco desde 4067 hasta 4095, 28 bytes
#define dirEEdescTempr 4096    // 60 x 20=1200 bytes, 19 caracteres +1
#define dirEEdescReler 5296    // 60 x 20=1200 bytes, 19 caracteres +1
#define dirEEdescAnalr 6496    // 20 x 20=400 bytes, 19 caracteres +1
#define dirEEdescEDigr 6896    // 20 x 20=400 bytes, 19 caracteres +1
#define dirEEdescSDigr 7296    // 20 x 20=400 bytes, 19 caracteres +1
#define dirEEfactorArem 7696   // 20 x 4 = 80 bytes, factor conversion analógicas remotas
#define dirEEfactorA 7776      // 16 x 4 = 64 bytes, factor conversion analógicas locales
#define dirEEdescEsc 7840      // 5 x 20 = 100, descripciones escenas
#define dirEEesc 7940          // 5 escenas x 11 = 55 bytes, escenas 
#define dirEEPRGsem 7999    // 20 bytes
#define dirEEPRGeve 8019    // 20 bytes
#define dirEEdyndnsHost 8039   // 40 bytes
#define dirEEdyndnsUser 8079   // 40 bytes
#define dirEEdyndnsPass 8119   // 40 bytes
// hueco desde 8119 hasta 8191, 73 bytes
#define dirEEevenvalA 10000  // 128, 32x4, valor de activación para analógicas
// hueco desde 10128 hasta 10255, 128 bytes  // Reservados ampliación evenval 
#define dirEEevenhis 10256   // 128  32x4, valor de histéresis de activación del evento
// hueco desde 10384 hasta 10511, 128 bytes  // Reservados ampliación evenhis 
#define dirEEdescPrg 10512      // 5 x 20 = 100, descripciones escenas
// hueco desde 10612 hasta 10711, 128 bytes  // Reservados ampliación evenhis 
#define dirEEdesc433 10712    // 20 x 30=600  para nombres de dispositivos 433 (19 +1)
#define dirEEurlLink 11312   // 5x80=400 5 posibles enlaces de usuario
#define dirEEdescLink 11712  // 5x20=100 descripciones Links
#define dirEEoffsetArem 11812   // 20 x 4 = 80 bytes, offset conversion analógicas remotas
#define dirEEoffsetA 11892      // 16 x 4 = 64 bytes, offset conversion analógicas locales
#define dirEEperActRem 11956    // 60 x2, período de actualización de unidades remotas
#define dirEEsumatA 12706      // 20 x 1 = 20 bytes, sumatorio conversion analógicas locales
#define dirEEsumatArem 12726   // 20 x 1 = 20 bytes, sumatorio conversion analógicas remotas
// hueco desde 12746  hasta final

byte modo=0;
/********* IMPORTANTE  *****************/
//const byte valorpin[2] = {0,1};    // directo 0=LOW,  1=HIGH    depende de laplaca de relÃ©s: placa estrecha
const byte valorpin[2] = {1,0};    // 2  inverso 1=LOW,  0=HIGH    depende de laplaca de relÃ©s: placa ancha
const byte reversepin[2] = {1,0};  // 2  inverso 1=LOW,  0=HIGH    
const int buffLEN = 128;   // 2 tamaño buffer para webserver.processConnection

int minRam = 8000;        // 2

boolean arrancando = true;    // 1
byte showN   =  1;    // 1 indica si se muestra el número de pin en la lista de entradas/salidas
int peract  =  60;     // 2 período de actualización automática de la página principal 
int peractRem  =  60;  // 2 período de actualización de datos de remotos 
byte debugserial = 0;

byte usepass =  0;    // 1, usar password:1,  sin password:0
char user1[LEN20];    // 20 bytes
char pass1[LEN20];    // 20 bytes
tmElements_t tm;       // estructura para RTC
tmElements_t tmInicio; // estructura para RTC

const byte freepines[]={        // 54  pin libre=0; pin reservado=1
          0,0,0,0,1, 0,0,0,0,0,  1,0,0,1,0, 0,0,0,0,1,  // pines  0 a  19
          1,1,0,0,0, 0,0,0,0,1,  0,1,0,1,0, 1,0,0,0,0,  // pines 20 a 39
          0,0,1,1,1, 1,1,1,1,1,  1,1,1,1};              // pines 40 a 53
const byte tabpin[34]={0,1,2,3,5,6,7,8,9,11,12,14,15,16,17,18,19,22,23,  // tabla de pines útiles
                       24,25,26,27,28,30,32,34,36,37,38,39,40,41,42};        
  byte EEmac[6];                // 6  100-105   dirección MAC
  byte EEip[4];                 // 4   106-109   dirección IP
  byte EEgw[4];                 // 4   110-113   puerta de enlace 
  byte EEmask[4];               // 4   114-117   máscara de subred
  byte EEdns[4];                // 4   118-121   servidor DNS
  unsigned int EEwebPort=82;    // 2   122-123   puerto servidor web 
  // variables SD
  byte sdOK = 0;                // 1
  byte sdHay = 0;               // 1   indica si existe tarjeta SD
  SdFat SD;
  SdFile file;
  
  byte numIA = 16;              // 1   200       número de entradas anlÃ³gicas mÃ¡ximo 16
  byte numID = 16;              // 1    201       número de entradas digitales
  byte numOD = 22;              // 1    202       número de E/S utilizadas, la suma de las 3 anteriores mÃ¡ximo 54
  byte numPines;                // 1    número total de pines usados, se calcula y no se guarda
  byte bshowpin[lTAB];          // 8    bytes,   0:visible, 1:oculto, en Panel Principal
  byte numNodos=0;              // número de nodos diferentes, hasta 60, seguardan en tnodosact
  unsigned int tnodosact[maxNodos];     // 120 bytes, 60, nodos activos, no se guarda, se crea cada reset
  byte tlastnodo[maxNodos];     // 60 bytes, 60 nodos  minutos desde última escucha del nodo
  byte bshowEsc;                // 1 byte, por bits uno para cadaescena 
  byte bshowTemp[3];            // 3    bytes,   0:visible, 1:oculto, en Panel Principal
  byte bshowLink;               // 1    bytes,   0:visible, 1:oculto, en Panel Principal
  byte bprgtipo[4];             // tipo de salida en prg semanal: local/remota
  byte beveacttipo[4];          // tipo de activadora en prg evento: local/remota
  byte bevesaltipo[4];          // tipo de salida en prg evento: local/remota
  byte bfectipo[4];             // tipo de salida en prg fecha: local/remota
  byte bactsem[4];              // programa semanal activo, 4 bytes = 32 programas
  byte bacteve[4];              // programa eventos activo, 4 bytes = 32 programas
  byte bactfec[4];              // programa fechas activo, 4 bytes = 32 programas
  byte evenact[nPRG];           // 32   1 por cada evento, señal que activa el evento
  byte bevenENABLE[2][lTAB];    // 16   2x8 bytes, define si el evento ha activado ya algo
  byte evencomp[nPRG];          // 32   32 bytes, comparador: =, <>, >=, .... 
  byte evenvalD[nPRG];          // 32   32 bytes, valor de activación del evento para digitales
  byte evensal[nPRG];           // 68   define la señal sobre la que se actua
  byte bevenniv[lTAB];          // 8    8 bytes, define nivel a activar. 

  byte evenTemp[nPRG];          // 32   define la señal sobre la que se actua
  long evenvalA[nPRG];          // 128  32x4, valor de activación del evento para analógicas
  long evenhis[nPRG];           // 128  32x4, valor de histéresis de activación del evento
  byte bescena[nEsc][11];       // 55   5 x 11 = 55, definición de escenas   
  byte bPRGsem[nPRGsem][4];     // 20 bytes, para guardar 5x32 bits de asociación PRG-PRGsem
  byte bPRGeve[nPRGeve][4];     // 20 bytes, para guardar 5x32 bits de asociación PRG-PRGeve
  byte actPrg[5];               // 5 bytes guarda que programas están activos: 0/1 
  byte tipoED[16];              // tipo de la entrada digital: 0=ON/OFF, 1=DHT11
  float vanaTED[16];              // valores anal. de entradas digitales para sensores DHT11-Temp
  float vanaHED[16];              // valores anal. de entradas digitales para sensores DHT11-Hum
  
  uint16_t nodotempr[maxTempr];   // 120  60 x 2

  // variables para programación SEMANAL
  byte bprgval[lTAB];         // 8 bytes, valor a poner en cada programación, on/off
  byte prgsal[nPRG];          // 32 bytes, salida a actuar en cada programación
  byte prgdia[nPRG];          // 32 bytes, días de la semana, bits 0 a 6
  byte prghor[nPRG];          // 32 bytes, hora
  byte prgmin[nPRG];          // 32 bytes, minuto
  byte prgcon[nPRG];          // 32 bytes, señal para activación condicional
  byte prgcomp[nPRG];         // 32 bytes, tipo de comparación
//  int prgconv[nPRG];          // 64 bytes, valor de la señal para activación condicional

  // variables para programación por FECHAS
  byte bfecval[lTAB];         // 8 bytes, valor a poner en cada programación, on/off
  byte fecsal[nPRG];          // 32 bytes, salida a actuar en cada programación por fechas
  byte fecano[nPRG];          // 32 bytes, año menos 2000 para poder usar un byte
  byte fecmes[nPRG];          // 32 bytes, mes
  byte fecdia[nPRG];          // 32 bytes, día del mes 
  byte fechor[nPRG];          // 32 bytes, hora
  byte fecmin[nPRG];          // 32 bytes, minuto

  #define maxRem 120            // número máximo de e/s remotas
  byte nTemp1=0;                 // número temperaturas 1-wire, puerto 1
  byte nTemp2=0;                 // número temperaturas 1-wire, puerto 2
  byte nTempr=0;                // número temperaturas remotas de un nodo
  unsigned int remNodo[maxRem]; // nodo e/s remota
//  byte antigNod[maxRem];        // tiempo en períodos de 10 segundos desde última recepción de cada nodo
  byte remPin[maxRem];          // 120  Pin e/s remota
  byte remTpin[maxTempr];       // 60  Sondas 1-wire remotas
  byte rbshowpin[lTABr];        // 24   bytes, hasta 192 bits, en panel general
  byte rbshow433[lTAB];          // 8    bytes,   0:visible, 1:oculto, en lista general
  boolean TX433activo=false;
  int nodact, nodnue, canact, cannue;
  uint16_t Nodoraiz=0;
  byte addr1Wire[maxTemp][8];      // ver de eliminar
  unsigned long contadores[8];
  byte prectemp=12;
  int contador=0; 
  int velocidad=0; 

  long mact1;
  long mact10;
  long mact60;
  long mactVar;
  long mactRem;
  long pmactVar=60;
  long pmactRem=60;
  int opcode=99;
  
  // variables radios 433
  unsigned long addr433[max433];    // dirección del dispositivo 433 VER de GUARDAR EN EEPROM o CodeON
  unsigned long code433Off[max433];    // CodeOFF para genéricos
  byte  unit433[max433];  // VER de GUARDAR EN EEPROM
//  byte dim433[max433];  // VER de GUARDAR EN EEPROM
  byte tipo433[max433];    // clase/tipo de dispositivo: 0=receptor, 1=emisor, 2= ambos
  byte react433=0;    // reenviar estado cada ract433 minutos, radio 433
  byte reactRad=0;    // reenviar estado cada ract433 minutos, radio 2,4 Ghz
  float factorA[nANALO];  // 16x4 factor conversión analógicas locales
  float offsetA[nANALO];   // 16x4 offset conversión analógicas locales
  float factorArem[nANALOr];  // 20x4 factor conversión analógicas remotas
  float offsetArem[nANALOr];  // 20x4 offset conversión analógicas remotas
  byte bsumatA[2];  
  byte bsumatArem[3];  
  byte tipoLCD=0;
  boolean tareason=false;
  long auxrnd433=0;      // número aleatorio para seguridad de mensajes 433
  
  NewRemoteCode msgRecibido;

  //File milog;
  char namelogEve[]="eventos.log";  // tipo 2  
  char namelogAcc[]="acciones.log";  // tipo 1, 5 y 6    
  char namelogAna[]="analog.log";  // tipo 3
  char namelogSys[]="system.log";  // tipo 4    
  char namelogIP[]="ipaccess.log";  // tipo 7
  char namelogTem[]="temperat.log";  // 
 // file backup
  char namefilebackup[]={"backup.bin"};
  char namefile[13];
  byte resetfab=0;

Mudbus Mb;
DHT dht(6);

#define PREFIX ""
WebServer webserver(PREFIX, EEwebPort);
LiquidCrystal lcd(lcdRS,lcdRW,lcdEN,lcdD0,lcdD1,lcdD2,lcdD3); 
EthernetClient EthClient;

//P(htmlHead_i0) = "<!DOCTYPE html><html><head><title>";
P(htmlHead_i0) = "<!DOCTYPE html PUBLIC \"-//W3C//DTD XHTML 1.0 Transitional//EN\" "
                 "\"http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd\">"
                 "<html xmlns=\"http://www.w3.org/1999/xhtml\"><head><title>";
P(htmlHead_ix) = "Conuco Web server";
P(htmlHead_i1) = "</title>"
    "<meta name=\"viewport\" content=\"width=device-width; initial-scale=2.5\"/>"
    "<meta http-equiv=\"Content-Type\" content=\"text/html;charset=utf-8\"/>"
    
    "<style type=\"text/css\">"
    
    "table.tmenu {font-family:Verdana; font-size:11pt;color:#333333;border-width:2px;border-color:#666666;"
                 "border-collapse:collapse; background-color:#dedede;}"     
    "table.tmenu th {color:#000000;border-width:2px;padding:8px;border-style:solid;border-color:#666666;"
                    "background-color:#FDFF9A;}"
    "table.tmenu td {color:#FFFFFF;border-width:2px;padding:8px;border-style:solid;border-color:#666666;"
                    "background-color:#dedede;}"

    "table.tpanel {font-family:Verdana; font-size:9pt;border-color:#666666;border-width:2px;"
                    "border-collapse:collapse;color:#FFFFFF;background-color:#515772;text-align:left;}"      
    "table.tpanel th {border-color:#666666;border-width:2px;padding:8px;border-style:solid;color:#000000; background-color:#FFFF00;text-align:center;}"
    "table.tpanel td {border-color:#666666;border-width:2px;padding:8px; border-style:solid; background-color:#515772;}"

    // usado en tablas de configuración, programación etc CENTRADO
    "table.tconf {text-align:center;font-family:Verdana; font-color:#000000;font-size:9pt; border-color:#666666;border-width:2px;border-collapse:collapse;color:#FFFFFF;background-color:#515772;text-align:left;}"      
    "table.tconf th {color:#000000;border-color:#666666;border-width:2px;padding:4px;border-style:solid;color:#000000; background-color:#FFFF00;text-align:center;}"
    "table.tconf td {text-align:center;color:#000000;border-color:#666666;border-width:2px;padding:4px; border-style:solid; background-color:#DEDEDE;}"

    // usado en tablas de configuración, programación etc NO CENTRADO
    "table.tnormal {font-family:Verdana; font-color:#000000;font-size:9pt; border-color:#666666;border-width:2px;border-collapse:collapse;color:#FFFFFF;background-color:#515772;text-align:left;}"      
    "table.tnormal th {color:#000000;border-color:#666666;border-width:2px;padding:4px;border-style:solid;color:#000000; background-color:#FFFF00;}"
    "table.tnormal td {color:#000000;border-color:#666666;border-width:2px;padding:4px; border-style:solid; background-color:#DEDEDE;}"
    "</style>";

/////////////    CÓDIGO DE SEGUIMIENTO Google Analytics

P(seguimiento)="<script type=\"text/javascript\">"
               "(function(i,s,o,g,r,a,m){i['GoogleAnalyticsObject']=r;i[r]=i[r]||function(){"
               "(i[r].q=i[r].q||[]).push(arguments)},i[r].l=1*new Date();a=s.createElement(o),"
               "m=s.getElementsByTagName(o)[0];a.async=1;a.src=g;m.parentNode.insertBefore(a,m)"
               "})(window,document,'script','//www.google-analytics.com/analytics.js','ga');"
               "ga('create', 'UA-1404365-7', 'zapto.org');"
               "ga('send', 'pageview');"
               "</script>";
//P(seguimiento1)="<script type=\"text/javascript\">"
//               "(function(i,s,o,g,r,a,m){i['GoogleAnalyticsObject']=r;i[r]=i[r]||function(){"
//               "(i[r].q=i[r].q||[]).push(arguments)},i[r].l=1*new Date();a=s.createElement(o),"
//               "m=s.getElementsByTagName(o)[0];a.async=1;a.src=g;m.parentNode.insertBefore(a,m)"
//               "})(window,document,'script','//www.google-analytics.com/analytics.js','ga');"
//               "ga('create', '";
//P(seguimientox)="UA-1404365-7";
//P(seguimiento2)="', 'elconucom.zapto.org');ga('send', 'pageview');</script>";
/////////////

P(head_f) = "</head>";
P(body_i) = "<body>";  P(body_f) = "</body>";
P(html_i) = "<html>";  P(html_f) = "</html>";
P(htmlRefresh_i) = "<meta http-equiv=\"refresh\" content=\"";
P(Http400) = "HTTP 400 - BAD REQUEST";
P(Form_get) = "\" method=\"get\">";
P(Form_post) = "\" method=\"post\">";
P(Form_action) = "<FORM action=\"";
P(value1)="\" value=\"1\"";
P(checked)=" checked";
P(Form_seg) = "ss.html";
P(Form_rel) = "srl.html";
P(Form_eve) = "sv.html";
P(Form_clima) = "sclima.html";
P(setupDev) = "sd.html";
P(setupNod) = "sn.html";
P(setupSys) = "rst.html";
P(resetNod) = "rn.html";
P(setupNod2) = "sn2.html";
P(setupSond) = "st.html";
P(setupEAnag) = "sea.html";
P(setupEDig) = "sed.html";
P(setupSDig) = "ssd.html";
P(remEAnaSetup) = "sr.html";
P(remEDigSetup) = "sred.html";
P(remSDigSetup) = "srsd.html";
P(remSRelSetup) = "srsr.html";
P(rem4Setup) = "sr4.html";
P(rem4codeSetup) = "sr4code.html";
P(rTempSetup) = "srt.html";
P(radSetup) = "sRa.html";
P(rad4Setup) = "sRa4.html";
P(radSetup4) = "sRa4.html";
P(netSetup) = "sNe.html";
P(webclient) = "wc.html";
P(Form_f) = "</FORM>";
P(Form_input_text_start) = "<input type=\"text\" name=\"";
P(Form_input_text_start_invis) = "<input type=\"text\" style=\"visibility: hidden\" name=\"";
P(Form_input_pass_start) = "<input type=\"password\" name=\"";
P(Form_input_value) = "\" value=\"";
P(Form_input_size) = "\" maxlength=\"1\" size=\"";
P(Form_input_end) = "\">\n";
P(Select_name) = "<SELECT name=\"";
P(Select_name_invis) = "<SELECT style=\"visibility: hidden\" name=\"";
P(Select_f) = "</SELECT>";
P(Max_length) = "\" maxlength=\"";
P(b_i) = "<b>";    P(b_f) = "</b>";
P(h1_i) = "<h1>";  P(h1_f) = "</h1>"; 
P(h2_i) = "<h2>";  P(h2_f) = "</h2>";
P(h3_i) = "<h3>";  P(h3_f) = "</h3>"; 
P(h4_i) = "<h4>";  P(h4_f) = "</h4>";
P(cierre) = ">";
P(comillascierre) = "\"/>";
P(tablamenu) = "<table class=\"tmenu\">";
P(tablapanel) = "<table class=\"tpanel\">";
P(tablaconf) = "<table class=\"tconf\">";
P(tablanormal) = "<table class=\"tnormal\">";
P(br) = "<br />";    P(brn) = "<br />\n";
P(td) = "<td>";  P(td_f) = "</td>";  P(td_if) = "<td></td>";
P(th) = "<th>";  P(th_f) = "</th>";  
P(tr) = "<tr>";  P(tr_f) = "</tr>";  P(tr_if) = "<tr></tr>";
P(trd_i) = "<tr><td>";
P(size_i) = "\" size=\"";
P(trcolor1_i) = "<tr>";
P(trcolor2_i) = "<tr bgcolor=#FFFF00 valign=\"center\">"; // amarillo
P(tdcolor2_i) = "<td bgcolor=#FFFF00 valign=\"center\">"; // amarillo
P(tdcolor2center_i) = "<td bgcolor=#FFFF00 valign=\"center\" align=\"center\">"; // amarillo
P(table_f) = "</table>";
P(colspan) = "<td colspan=\"";
P(rad250kbps) = "250 Kbps";
P(rad1mbps) = "1 Mbps";
P(rad2mbps) = "2 Mbps";
P(mayoroigual) = ">=";          
P(menoroigual) = "<=";          
P(on) = "On";    P(off) = "Off";     P(onoff) = "On/Off";
P(ON) = "ON";    P(OFF) = "OFF";     P(ONOFF) = "ON/OFF";
P(b) = " ";
P(selected) = " selected";
P(optionvalue) = "<OPTION value=\"";
P(option_f) = "</OPTION>";
P(barramayor)="\">";
P(tdbgcolorFFFF00)="<td bgcolor=#FFFF00>";         
P(tdbgcolor8E8E8E)="<td bgcolor=#8E8E8E>";         
P(tdbgcolorAAAAAA)="<td bgcolor=#AAAAAA>";         
P(tdcentercol)= "<td align=\"center\">";
P(tdcentercolspan2)= "<td align=\"center\" colspan=\"2\">";
P(tdcentercolspan3)= "<td align=\"center\" colspan=\"3\">";
P(tdrightcolspan2)= "<td align=\"right\" colspan=\"2\">";
P(centercolspan2)= "align=\"center\" colspan=\"2\">";
P(centercolspan3)= "align=\"center\" colspan=\"3\">";
P(colspan4)="colspan=\"4\">";
P(tdwidthbgcolorFFFF00)="<td width=\"40\" bgcolor=#FFFF00 ";

P(aligncenteron)="align=\"center\">ON";
P(aligncenteroff)="align=\"center\">OFF";
P(hrefon)="<a href=\"on?";
P(hrefoff)="<a href=\"off?";
P(hrefonr)="<a href=\"onr?";
P(hrefoffr)="<a href=\"offr?";
P(hrefon4)="<a href=\"on4?";
P(hrefoff4)="<a href=\"off4?";
P(ona_f)="\">ON</a>";
P(offa_f)="\">OFF</a>";
P(celsius)=" ºC";          
P(hr)="<hr>";  
P(dospuntos)=":";
P(resetcontl)="<a href=\"sy.html?act=rl";     
P(resetcontp)="<a href=\"sy.html?act=rp";     
P(a_f)="</a>";          
P(blancos4)="    ";
P(punto)=".";
P(coma)=",";
P(puntoycoma)=";";
P(porciento)=" %";
P(letraa)="a";
P(letrad)="d";
P(letrae)="e";
P(letraf)="f";
P(letraI)="I";
P(letrak)="k";
P(letran)="n";
P(letraN)="N";
P(letraP)="P";
P(letrar)="r";
P(letras)="s";
P(letrav)="v";
P(letrasred)="red";
P(letrasrsd)="rsd";
P(ed)="ed";
P(sd)="sd";
P(rs)="rs";
P(ra)="ra";
P(rk)="rk";
P(rr)="rr";
P(N2puntos)="N: "; 
P(C2puntos)="C: "; 
P(paren_i)="(";
P(paren_f1b)=") ";
P(paren_f2b)=")  ";
P(corchete_i)=" [";
P(corchete_f)="]";
P(parenguion)=")-";
P(mayorparen)=">(";
P(optionvalue0)="<OPTION value=\"0\"";  
P(optionvalue1)="<OPTION value=\"1\"";  
P(optionvalue2)="<OPTION value=\"2\"";  
P(optionvalue4)="<OPTION value=\"2\"";  
P(optionvalue5)="<OPTION value=\"2\"";  
P(optionvalue255)="<OPTION value=\"255\"";
P(barraa)="a";
P(barradi)="di";
P(barrads)="ds";
P(barrared)="red";
P(barrars)="rs";
P(barrarsd)="rsd";
P(barrara)="ra";
P(barrarr)="rr";
P(barrado)="do";
P(barras)="s";
P(llave_i)="{ ";
P(llave_f)=" }";
P(comablanco)=", ";
P(tdalignright)="<td align=\"right\">";            
P(tralignright)="<tr align=\"right\">";            
P(guion)="-";            
P(setupsem)="sse.html";  
P(tdcolspan2)="<td colspan=\"2\">";
P(tdcolspan3)="<td colspan=\"3\">";
P(tdcolspan4)="<td colspan=\"4\">";        
P(tdcolspan5)="<td colspan=\"5\">";        
P(tdcolspan7)="<td colspan=\"7\">";
P(inputcheckbox)="<input type=\"checkbox\" name=\""; 
P(setupfec)="sf.html";
P(setupesc)="sesc.html";
P(setupprg)="sprg.html";
P(setuplinks)="slinks.html";
P(dema)="DEMA";
P(chacon)="CHACON D-IO";
P(conuco)="CONUCO";
//P(kaku)="KAKU";
//P(blokker)="BLOKKER";
//P(elro)="ELRO";
P(n) = "n";
P(barra) = "/";
P(barraatras)="\"";
P(cero) ="0";
P(uno) = "1";
P(dos) = "2";
P(tres) = "3";
P(cuatro) = "4";
P(cinco) = "5";
P(seis) = "6";
P(bytes) = "bytes";            
P(Bytes) = "Bytes";            
P(elconuco) = "El Conuco Labs";
P(dyndnsNOIP)="https://dynupdate.no-ip.com";
P(noip)="NO-IP";
P(dyndns)="DynDNS";
P(blancoC)=" C";
P(blancoIP)=" IP:";
P(menor)="<";
P(mayor)=">";
P(tRTC)="RTC ";
P(mdospuntosp)=" M:P";
P(mailHELO)="EHLO 1";
P(mailAUTH)="AUTH LOGIN";
P(mailFrom)="MAIL From: ";
P(mailRCPT)="RCPT To: ";
P(mailDATA)="DATA";
P(mailFROM)="From: Conuco Controller <";
P(mailTO)="To: ";
P(mailSubject)="Subject: ";
P(mailQUIT)="QUIT";
P(mailQuitrin)="QUIT\ri\n";  
P(mailelconuco)="Conuco Controller.";    
P(ttestEEPROM)="EEPROM";
P(escribirceros)="Writing/Reading zero";
P(escribir015)="Writing/Reading values 0-15";
P(href_i)="<a href=";
P(href_cierre)=">";
P(href_f)="</a>";
P(chdir01)="<a href=\"cd?";
P(chdir02)="\">";
P(chdir03)="</a>";
P(showfile01)="<a href=\"sf?";
P(showfile02)="\">";
P(showfile03)="</a>";
P(deletefile01)="<a href=\"df?";
P(deletefile02)="\">";
P(deletefile03)="</a>";
P(ready)="*****  Ready  *****";
P(avisoINI)="=== INI ===";
P(avisoFIN)="=== FIN ===";
P(dht11)="DHT11";
P(dht21)="DHT21";
P(dht22)="DHT22";
P(blancoERROR)=" ERROR";
P(blancoOK)=" OK";
P(Panel)="Panel";
P(sprghtml)="sprg.html";
P(seschtml)="sesc.html";
P(svhtml)="sv.html";
P(sclimahtml)="sclima.html";
P(sNehtml)="sNe.html";
P(sRahtml)="sRa.html";
P(sRa4html)="sRa4.html";
P(sdhtml)="sd.html";
P(slogshtml)="slogs.html";
P(sfileshtml)="sfiles.html";
P(snhtml)="sn.html";
P(rnhtml)="rn.html";
P(rsthtml)="rst.html";
P(sn2html)="sn2.html";
P(sthtml)="st.html";
P(seahtml)="sea.html";
P(sedhtml)="sed.html";
P(ssdhtml)="ssd.html";
P(srhtml)="sr.html";
P(sredhtml)="sred.html";
P(srsdhtml)="srsd.html";
P(srsrhtml)="srsr.html";
P(sr4html)="sr4.html";
P(sr4codehtml)="sr4code.html";
P(srthtml)="srt.html";
P(sahtml)="sa.html";
P(sshtml)="ss.html";
P(srlhtml)="srl.html";
P(act)="act";
P(rsteex)="rsteex";
P(shee)="shee";
P(backup)="backup";
P(restore)="restore";
P(deletefile)="deletefile";
P(ssehtml)="sse.html";
P(sfhtml)="sf.html";
P(json)="json";
P(puntoflecha)=".  -->";

//////  tratamiento de bits /////////////////////
const byte tab[8] = {1,2,4,8,16,32,64,128};    // 8

byte mailACT = 0;          //  1
byte mailPER = 5;          //  2
char mailSVR[LEN40] = "";  //  40
int mailPORT = 25;         //  2
char mailUSER[LEN40] = ""; //  40
char mailPASS[LEN40] = "";   //  40
char mailSENDER[LEN40] = ""; //  40
char mailDEST[3][LEN40] = {"","",""};   //  120
byte dyndnsServ=0;
char dyndnsHost[LEN40]="";
char dyndnsUser[LEN40]="";
char dyndnsPass[LEN40]="";

#define maxVALr 22

// Configuración radio NRF24L01
// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
RF24 radio(radCE,radSEL);  
#define radnRed 9        // nÃºmero posible de redes
byte radCan[radnRed];    // tabla del canal asignado a cada red
byte redNodo[maxRem];    // tabla de la red a que pertenece cada nodo (120 mÃ¡ximo)
byte radACT = 1;   // 1   radio activa sí/no
byte radACT433 = 0;   // 1   radio activa sí/no 433
byte radFUN = 0;   // 1   funciÃ³n master=0, slave=1
byte radPAQ = 32;  // 1   tamaÃ±o paquete
byte radVEL = 1;   // 1   1 Mbps por defecto
byte radRED = 0;   // 1   Red a la que pertenece en modo slave
uint16_t radNOD = 0;   // 2   nÃºmero de nodo de RF24Network
RF24Network network(radio);

//
byte transByte(byte orig)
  {
  return orig;  
  }
  
//
byte getbit(byte mibyte, byte pin)
 {if ((mibyte & tab[(pin % 8)]) > 0) 
    return 1; 
  else 
    return 0;  }

void setbit(byte *mibyte, byte pin, byte value)  { 
  if (value == 0)
    *mibyte = *mibyte & (255 ^ tab[(pin % 8)]);  
  else
    *mibyte = *mibyte | tab[(pin % 8)]; }
 
byte getbit8(byte tabla[], byte pin)
 {if ((tabla[pin/8] & tab[(pin % 8)]) > 0) 
    return 1; 
  else 
    return 0;  }

void setbit8(byte tabla[], byte pin, byte value)  { 
  if (value == 0)
    tabla[pin/8] = tabla[pin/8] & (255 ^ tab[(pin % 8)]);  
  else
    tabla[pin/8] = tabla[pin/8] | tab[(pin % 8)]; }
    
byte EEread(unsigned int dir)  {
  return readEEPROM(EEPROM2dir, dir); }   // está en EEPROMAnything  }

void EEwrite(int dir, byte valor)  {
   writeEEPROM(EEPROM2dir,dir,valor);  }  // está en EEPROMAnything  

void printPiP(WebServer &server, const prog_uchar *texto1,int num,const prog_uchar *texto2)
  {
    server.printP(texto1);
    server.print(num);
    server.printP(texto2);
  }

void printPlP(WebServer &server, const prog_uchar *texto1,long num,const prog_uchar *texto2)
  {
    server.printP(texto1);
    server.print(num);
    server.printP(texto2);
  }

void printPcP(WebServer &server, const prog_uchar *texto1,char car,const prog_uchar *texto2)
  {
    server.printP(texto1);
    server.print(car);
    server.printP(texto2);
  }

void printP1(WebServer &server, const prog_uchar *texto1)
  {
    server.printP(texto1); 
  }

void printP2(WebServer &server, const prog_uchar *texto1, const prog_uchar *texto2)
  {
    server.printP(texto1); server.printP(texto2);
  }

void printP3(WebServer &server, const prog_uchar *texto1, const prog_uchar *texto2, const prog_uchar *texto3)
  {
    server.printP(texto1); server.printP(texto2); server.printP(texto3);
  }

void printP4(WebServer &server, const prog_uchar *texto1, const prog_uchar *texto2, 
                                const prog_uchar *texto3, const prog_uchar *texto4)
  {
    printP2(server, texto1, texto2); printP2(server, texto3, texto4);
  }

void printP5(WebServer &server, const prog_uchar *texto1,  const prog_uchar *texto2, 
                                const prog_uchar *texto3,  const prog_uchar *texto4,
                                const prog_uchar *texto5)
  {
    printP3(server, texto1, texto2, texto3);
    printP2(server, texto4, texto5);
  }

void printP6(WebServer &server, const prog_uchar *texto1,  const prog_uchar *texto2, 
                                const prog_uchar *texto3,  const prog_uchar *texto4,
                                const prog_uchar *texto5,  const prog_uchar *texto6)
  {
    printP3(server, texto1, texto2, texto3);
    printP3(server, texto4, texto5, texto6);
  }
  
void printP7(WebServer &server, const prog_uchar *texto1,  const prog_uchar *texto2, 
                                const prog_uchar *texto3,  const prog_uchar *texto4,
                                const prog_uchar *texto5,  const prog_uchar *texto6,
                                const prog_uchar *texto7)
  {
    printP3(server, texto1, texto2, texto3);
    printP4(server, texto4, texto5, texto6, texto7);
  }
  
void printP8(WebServer &server, const prog_uchar *texto1,  const prog_uchar *texto2, 
                                const prog_uchar *texto3,  const prog_uchar *texto4,
                                const prog_uchar *texto5,  const prog_uchar *texto6,
                                const prog_uchar *texto7,  const prog_uchar *texto8)
  {
    printP4(server, texto1, texto2, texto3, texto4);
    printP4(server, texto5, texto6, texto7, texto8);
  }

void printS(const prog_uchar *str)
  {
  char c;
  while((c = pgm_read_byte(str++)))
    Serial.write(c);
  }

void printlnS(const prog_uchar *str)
  {
  char c;
  while((c = pgm_read_byte(str++)))
    Serial.write(c);
  Serial.println();
  }

void serialmem(char texto[])
  {
  printS(libre); Serial.print(mifreeRam()); 
  }
    
void printTDTR(WebServer &server, byte posicion, boolean trb, boolean tdb)
  {
    if (posicion==0)    // inicio
      {
      if (trb) printP1(server,tr);
      if (tdb) printP1(server,td);
      }
    else
      {
      if (tdb) printP1(server,td_f);
      if (trb) printP1(server,tr_f);
      }
  }
  
void printTD(WebServer &server, char *texto, boolean tr, boolean td)
  {
  printTDTR(server, 0, tr, td);
  server.print(texto);
  printTDTR(server, 1, tr, td);
  }
 
void printTDP(WebServer &server, const prog_uchar *texto, boolean tr, boolean td)
  {
  printTDTR(server, 0, tr, td);
  server.printP(texto);
  printTDTR(server, 1, tr, td);
  }
 
void printIP(WebServer &server, int valor, byte base, const prog_uchar *texto)
  {
    if (base==10) server.print(valor,DEC);
    else if (base==8) server.print(valor,OCT);
      else if (base==16) server.print(valor,HEX);
        else if (base==2) server.print(valor,BIN);
          else server.print(valor);
    server.printP(texto);
  }
  
void printLP(WebServer &server, long valor, byte base, const prog_uchar *texto)
  {
    if (base==10) server.print(valor,DEC);
    else if (base==8) server.print(valor,OCT);
      else if (base==16) server.print(valor,HEX);
        else if (base==2) server.print(valor,BIN);
          else server.print(valor);
    server.printP(texto);
  }
  
void printcampoC(WebServer &server, int numpar, char *valactual, byte tam, boolean visible, boolean td, boolean pass)
  {
  printTDTR(server, 0, false, td);
  if (visible)
    if (pass)
      server.printP(Form_input_pass_start);
    else
      server.printP(Form_input_text_start);
  else
    server.printP(Form_input_text_start_invis);
  printIP(server, numpar, 10, Form_input_value);
  server.print(valactual);
  server.printP(Max_length);
  printIP(server, tam, 10, size_i);
  printIP(server, tam, 10, Form_input_end);
  printTDTR(server, 1, false, td);
  }
  
void printcampoL(WebServer &server, int numpar, long valactual, byte tam, boolean td)
  {
  printTDTR(server, 0, false, td);
  server.printP(Form_input_text_start);
  printIP(server, numpar, 10, Form_input_value);
  printLP(server, valactual, 10, Max_length);
  printIP(server, tam, 10, size_i);
  printIP(server, tam, 10, Form_input_end);
  printTDTR(server, 1, false, td);
  }
  
void printcampoChr(WebServer &server, int numpar, int valactual, byte tam, boolean td)
  {
  printTDTR(server, 0, false, td);
  server.printP(Form_input_text_start);
  printIP(server, numpar, 10, Form_input_value);
  server.write(valactual);
  printP1(server, Max_length);
  printIP(server, tam, 10, size_i);
  printIP(server, tam, 10, Form_input_end);
  printTDTR(server, 1, false, td);
  }
  
void printcampoI(WebServer &server, int numpar, int valactual, byte tam, boolean td)
  {
  printTDTR(server, 0, false, td);
  server.printP(Form_input_text_start);
  printIP(server, numpar, 10, Form_input_value);
  printIP(server, valactual, 10, Max_length);
  printIP(server, tam, 10, size_i);
  printIP(server, tam, 10, Form_input_end);
  printTDTR(server, 1, false, td);
  }
  
void printcampoIOCT(WebServer &server, int numpar, int valactual, byte tam,boolean td)
  {
  printTDTR(server, 0, false, td);
  server.printP(Form_input_text_start);
  printIP(server, numpar, 10, Form_input_value);
  printIP(server, valactual, 8, Max_length);
  printIP(server, tam, 10, size_i);
  printIP(server, tam, 10, Form_input_end);
  printTDTR(server, 1, false, td);
  }
  
void printcampouIOCT(WebServer &server, int numpar, unsigned int valactual, byte tam,boolean td)
  {
  printTDTR(server, 0, false, td);
  server.printP(Form_input_text_start);
  printIP(server, numpar, 10, Form_input_value);
  printIP(server, valactual, 8, Max_length);
  printIP(server, tam, 10, size_i);
  printIP(server, tam, 10, Form_input_end);
  printTDTR(server, 1, false, td);
  }
  
void printcampoF(WebServer &server, int numpar, float valactual, byte tam, byte deci, boolean td)
  {
  printTDTR(server, 0, false, td);
  server.printP(Form_input_text_start);
  printIP(server, numpar, 10, Form_input_value);
  server.print(valactual,deci);
  server.printP(Max_length);
  printIP(server, tam, 10, size_i);
  printIP(server, tam, 10, Form_input_end);
  printTDTR(server, 1, false, td);
  }
  
void printcampoH(WebServer &server, byte numpar, int valactual, byte tam,boolean td)
  {
  printTDTR(server, 0, false, td);
  server.printP(Form_input_text_start);
  
  printIP(server, numpar, 10, Form_input_value);
  printIP(server, valactual, 16, Max_length);
  printIP(server, tam, 10, size_i);
  printIP(server, tam, 10, Form_input_end);
  printTDTR(server, 1, false, td);
  }
  
void printcampoSiNo(WebServer &server, int numpar, int valactual, const prog_uchar *uno, const prog_uchar *cero, boolean visible,boolean td)
  {
  printTDTR(server, 0, false, td);
  server.printP(visible ? Select_name:Select_name_invis);
  server.print(numpar); server.printP(barramayor);
  server.printP(optionvalue1);
  if (valactual == 1) server.printP(selected);
  printP3(server, cierre, uno, option_f);
  server.printP(optionvalue0);
  if (valactual == 0) server.printP(selected); 
  printP3(server, cierre, cero, option_f);
  server.printP(Select_f);  
  printTDTR(server, 1, false, td);
  }         
  
void printcampoCB(WebServer &server, int numpar, int valactual, int pri, int ult, boolean visible, boolean td)
  {
  printTDTR(server, 0, false, td);
  server.printP(Select_name);
  server.print(numpar);   server.printP(barramayor);
  for (int j=pri; j<=ult; j++)  {
    server.printP(optionvalue); server.print(j); server.printP(barraatras);
    if (j == valactual) server.printP(selected);
    server.printP(cierre);
    server.print(j);     
    server.printP(option_f);  }
  server.printP(Select_f);  
  printTDTR(server, 1, false, td);
  }         
  
void printparC(WebServer &server, char *titulo, int numpar, char valactual[], byte tam, boolean pass)
  {
  printTDTR(server, 0, true, true);
  server.print(titulo);
  server.printP(td_f);  
  printcampoC(server, numpar, valactual, tam, true, true, pass);
  printTDTR(server, 1, true, false);
  }
  
void printparCP(WebServer &server, const prog_uchar *titulo, int numpar, char valactual[], byte tam, boolean pass)
  {
  printTDTR(server, 0, true, true);
  printP2(server, titulo, td_f);
  printcampoC(server, numpar, valactual, tam, true, true, pass);
  printTDTR(server, 1, true, false);
  }
  
void printparI(WebServer &server, char *titulo, int numpar, int valactual, byte tam)
  {
  printTDTR(server, 0, true, true);
  server.print(titulo);
  server.printP(td_f);
  printcampoI(server, numpar, valactual, tam, true);
  printTDTR(server, 1, true, false);
  }  
  
void printparIP(WebServer &server, const prog_uchar *titulo, int numpar, int valactual, byte tam)
  {
  printTDTR(server, 0, true, true);
  printP2(server, titulo, td_f);
  printcampoI(server, numpar, valactual, tam, true);
  printTDTR(server, 1, true, false);
  }  
  
void printparLP(WebServer &server, const prog_uchar *titulo, int numpar, long valactual, byte tam)
  {
  printTDTR(server, 0, true, true);
  printP2(server, titulo, td_f);
  printcampoL(server, numpar, valactual, tam, true);
  printTDTR(server, 1, true, false);
  }  
  
void printparIPOCT(WebServer &server, const prog_uchar *titulo, int numpar, int valactual, byte tam)
  {
  printTDTR(server, 0, true, true);
  printP2(server, titulo, td_f);
  printcampoIOCT(server, numpar, valactual, tam, true);
  server.printP(tr_f);
  }  
  
void printparuIPOCT(WebServer &server, const prog_uchar *titulo, int numpar, unsigned int valactual, byte tam)
  {
  printTDTR(server, 0, true, true);
  printP2(server, titulo, td_f);
  printcampouIOCT(server, numpar, valactual, tam, true);
  printTDTR(server, 1, true, false);
  }  
  
void printColspan(WebServer &server, int ancho, boolean cerrado)
  {
  server.printP(colspan);
  server.print(ancho); 
  if (cerrado) server.printP(barramayor);
  }

void lineaH(WebServer &server, byte ancho)
  {
  printTDTR(server, 0, true, false);
  printColspan(server,ancho,true);
  server.printP(hr);
  printTDTR(server, 1, true, true);
  }

void printEE(WebServer &server, int dir, byte len)
  {
  int i=0;
  char mitext[80]; mitext[0]='\0';
  boolean repeat;
  do {
    mitext[i] = EEread(dir+i);
    repeat = (mitext[i] != '\0');
    i = i++;
    } while (repeat);
  server.print(mitext);
  }
  
void printEEmail(EthernetClient &client, int dir, byte len)
  {
  int i=0;
  char mitext[20]; mitext[0]='\0';
  boolean repeat;
  do {
    mitext[i] = EEread(dir+i);
    repeat = (mitext[i] != '\0');
    i = i++;
    } while (repeat);
  client.print(mitext);
  }
  
//******************************************************************************************************

void printEth(const prog_uchar *str)
  {
  char c;
  while((c = pgm_read_byte(str++)))
    EthClient.write(c);
  }

//Función que envia el email
byte sendEmail(byte pinx, byte even, byte tipo)
{
  boolean debugmail=true;
   char user64[40];
   char pass64[40];
    if (mailACT == 0) return(0);
    if (!EthClient.connect(mailSVR,mailPORT)) return 0; 
    if(!eRcv()) return 0;
    
    printEth(mailHELO);  
    if(!eRcv()) return 0; 
    
    printEth(mailAUTH);    //Le decimos que queremos autenticarnos
    if(!eRcv()) return 0;
    byte largo=strlen(mailUSER);
    base64_encode(user64,mailUSER,largo);

    EthClient.println(user64);   //Enviamos el usuario en base64
    if(!eRcv()) return 0;
    
    largo=strlen(mailPASS);
    base64_encode(pass64,mailPASS,largo);
    EthClient.println(pass64);   //Enviamos el password en base64    
    if(!eRcv()) return 0;
    
    printEth(mailFROM); 
    printEth(menor); EthClient.print(mailSENDER); printEth(mayor);     //especificamos el sender
    if (!eRcv()) return 0;

    printEth(mailRCPT); printEth(menor); 
    if (tipo==0)
      {
      EthClient.print(mailDEST[evensal[pinx]-despMail]);      //especificamos el destinatario
      }
    else
      EthClient.print(mailDEST[evenTemp[pinx]-despMail]);     //especificamos el destinatario
    printEth(mayor); 
    if(!eRcv()) return 0;
    
    printEth(mailDATA);   //indicamos que vamos a empezar a escribir el correo

    printEth(mailFROM); EthClient.print(mailSENDER); printEth(mayor); 
    printEth(mailTO); 
    if (tipo==0)
      EthClient.println(mailDEST[evensal[pinx]-despMail]);
    else
      EthClient.println(mailDEST[evenTemp[pinx]-despMail]);
    printEth(mailSubject); printEth(mailALARMA);
    if (tm.Day < 10)  EthClient.print(0); EthClient.print (tm.Day);      printEth(barra);
    if (tm.Month < 9) EthClient.print(0); EthClient.print (tm.Month+1);  printEth(barra);
    EthClient.print (tmYearToCalendar(tm.Year)); printEth(b);
    if (tm.Hour < 10) EthClient.print(0);   EthClient.print (tm.Hour);   printEth(dospuntos);
    if (tm.Minute < 10) EthClient.print(0); EthClient.print (tm.Minute); printEth(dospuntos);
    if (tm.Second < 10) EthClient.print(0); EthClient.print (tm.Second); printEth(b);
    if (tipo==0)
      printEEmail(EthClient, dirEEdescpinD+(evenact[pinx]*LEN20), LEN20);
    else
      printEEmail(EthClient, dirEEdescTemp+(evenact[pinx]*LEN20), LEN20);
    printEth(b); 
    if (tipo==0) 
      if (evenvalD[pinx-numIA] == 1) printEth(ON); else printEth(OFF);
    else
      {
      EthClient.print(float(Mb.R[pinx])/100); 
      printEth(blancoC); 
      }
    EthClient.println("");   
    if (tm.Day < 10)  EthClient.print(0); EthClient.print (tm.Day);     printEth(barra);
    if (tm.Month < 9) EthClient.print(0); EthClient.print (tm.Month+1); printEth(barra);
    EthClient.print (tmYearToCalendar(tm.Year)); printEth(b);
    if (tm.Hour < 10)   EthClient.print(0); EthClient.print (tm.Hour);   
    printEth(dospuntos);
    if (tm.Minute < 10) EthClient.print(0); EthClient.print (tm.Minute); 
    printEth(dospuntos);
    if (tm.Second < 10) EthClient.print(0);  EthClient.println (tm.Second);
    if (tipo==0) 
      {
      printEEmail(EthClient, dirEEdescpinD+(evenact[pinx]*LEN20), LEN20);
      printEth(mailHaPasadoa); 
      if (evenvalD[pinx] == 1) printEth(ON); else printEth(OFF);
      }
    else
      {
      printEth(mailsonda); 
      printEEmail(EthClient, dirEEdescTemp+(evenact[pinx]*LEN20), LEN20);
      printEth(mailhasuperado); 
      EthClient.print(float(evenvalA[pinx])/100);
      printEth(mailvaloract); 
      EthClient.print(float(Mb.R[pinx])/100);
      printEth(blancoC);
      }
    EthClient.println(""); 
    printEth(mailelconuco);
    EthClient.println(); 
    EthClient.println(""); 
    printEth(punto);        //Cuando enviamos un punto indica el fin del mensaje
    if(!eRcv()) return 0;
    
    printEth(mailQUIT);
    EthClient.println();   // Cerramos la conexion telnet
    if(!eRcv()) return 0;
    EthClient.flush();
    EthClient.stop();      //Paramos el cliente
    return 1; 
}

//

byte eRcv()
{
  byte respCode;
  byte thisByte;
  while(!EthClient.available()) delay(delayACKMail);
  respCode = EthClient.peek();
  while(EthClient.available())
    thisByte = EthClient.read();    
  if(respCode >= '4')
    {
    efail();
    return 0;  
    }  
  return 1;
}
//

void efail()
{
  byte thisByte = 0;
  printEth(mailQuitrin);
  while(!EthClient.available()) delay(1);
  while(EthClient.available())
    thisByte = EthClient.read();    
  EthClient.stop();
}

void printparentesis(WebServer &server, char letra, int numero, byte base)
  {
  server.printP(paren_i);
  server.print(letra); 
  if (base==10) server.print(numero);
  else
    if (base==8) server.print(numero,OCT);
  server.printP(paren_f2b);
  }

void printcorchete(WebServer &server, int numero, byte base)
  {
  server.printP(corchete_i);
  if (base==10) server.print(numero);
  else
    if (base==8) server.print(numero,OCT);
  server.printP(corchete_f);
  }

void printDayName(WebServer &server, byte dia)
  {
    if (dia==0) server.printP(domingo);
    else if (dia==1) server.printP(lunes);
      else if (dia==2) server.printP(martes);
        else if (dia==3) server.printP(miercoles);
          else if (dia==4) server.printP(jueves);
            else if (dia==5) server.printP(viernes);
              else if (dia==6) server.printP(sabado);
  }

void printMes(WebServer &server, byte mes)
  {
    if (mes==0) server.printP(enero);
      else if (mes==1) server.printP(febrero);
        else if (mes==2) server.printP(marzo);
          else if (mes==3) server.printP(abril);
            else if (mes==4) server.printP(mayo);
              else if (mes==5) server.printP(junio);
                else if (mes==6) server.printP(julio);
                  else if (mes==7) server.printP(agosto);
                    else if (mes==8) server.printP(septiembre);
                      else if (mes==9) server.printP(octubre);
                        else if (mes==10) server.printP(noviembre);
                          else if (mes==11) server.printP(diciembre);
  }
  
boolean autOK(WebServer &server)
 {
    char codeori[41]="";
    char code64[41]="";
    strcpy(codeori,user1);
    strcat(codeori,":");
    strcat(codeori,pass1);
    int largo=strlen(codeori);  
    base64_encode(code64,codeori,largo);
    if ((usepass) && (!server.checkCredentials(code64)))
      return false;
    else
      return true;
 }
 
void logAcc(char *mensaje, int npin, char tipo, boolean activa)
 {
 if (!sdHay) return;
  ofstream sdlog(namelogAcc, ios::out | ios::app);  
  if (sdlog)
    {
      char buf[3];
       sdlog << tmYearToCalendar(tm.Year);
       if (tm.Month < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Month+1,buf,10);
       if (tm.Day < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Day,buf,10);
       if (tm.Hour < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Hour,buf,10); 
       if (tm.Minute < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Minute,buf,10); 
       if (tm.Second < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Second,buf,10); 
       sdlog << tipo; 
       sdlog << itoa(npin,buf,10);
       printSD(sdlog, guion); 
       printSD(sdlog,activa?uno:cero);
       sdlog << char(10); 
       sdlog.close();
    }
 } 
 
void logEve(char *mensaje, int npin, char tipo, boolean activa)
 {
 if (!sdHay) return;
  ofstream sdlog(namelogEve, ios::out | ios::app);  
  if (sdlog)
    {
      char buf[3];
       sdlog << tmYearToCalendar(tm.Year);
       if (tm.Month < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Month+1,buf,10);
       if (tm.Day < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Day,buf,10);
       if (tm.Hour < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Hour,buf,10); 
       if (tm.Minute < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Minute,buf,10); 
       if (tm.Second < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Second,buf,10); 
       sdlog << tipo; 
       sdlog << itoa(npin,buf,10);
       printSD(sdlog, guion);
       sdlog << (activa?"ON":"OFF");
       sdlog << char(10); 
       sdlog.close();
    }
 } 

void logAna(char *mensaje, int npin, char tipo, boolean activa)
 {
 if (!sdHay) return;
  ofstream sdlog(namelogAna, ios::out | ios::app);  
  if (sdlog)
    {
      char buf[3];
       sdlog << tmYearToCalendar(tm.Year);
       if (tm.Month < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Month+1,buf,10);
       if (tm.Day < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Day,buf,10);
       if (tm.Hour < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Hour,buf,10); 
       if (tm.Minute < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Minute,buf,10); 
       if (tm.Second < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Second,buf,10); 
       sdlog << tipo; 
       sdlog << itoa(npin,buf,10);
       printSD(sdlog, guion); 
       printSD(sdlog,activa?uno:cero);
       sdlog << itoa(npin,buf,10);
       printSD(sdlog, guion);
       sdlog << (activa?"ON":"OFF");
////////////// aqui va lo de ana
       sdlog << char(10); 
       sdlog.close();
    }
 } 

void logSys(char *mensaje, int npin, char tipo, boolean activa)
 {
 if (!sdHay) return;
  ofstream sdlog(namelogSys, ios::out | ios::app);  
  if (sdlog)
    {
      char buf[3];
       sdlog << tmYearToCalendar(tm.Year);
       if (tm.Month < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Month+1,buf,10);
       if (tm.Day < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Day,buf,10);
       if (tm.Hour < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Hour,buf,10); 
       if (tm.Minute < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Minute,buf,10); 
       if (tm.Second < 10) printSD(sdlog,cero); 
       sdlog << itoa(tm.Second,buf,10); 
       sdlog << tipo; 
       sdlog << mensaje; 
       sdlog << char(10); 
       sdlog.close();
    }
 } 

void logIP(char *mensaje, int npin, char tipo, boolean activa)
 {
 if (!sdHay) return;
  byte rip[] = {0,0,0,0 };
  EthernetClient client = webserver.available();
  client.getRemoteIP(rip); // where rip is defined as byte rip[] = {0,0,0,0 };
  if (rip[0] != 192)
    {
    ofstream sdlog(namelogIP, ios::out | ios::app);  
    if (sdlog)
      {
        char buf[3];
         sdlog << tmYearToCalendar(tm.Year);
         if (tm.Month < 10) printSD(sdlog,cero); 
         sdlog << itoa(tm.Month+1,buf,10);
         if (tm.Day < 10) printSD(sdlog,cero); 
         sdlog << itoa(tm.Day,buf,10);
         if (tm.Hour < 10) printSD(sdlog,cero); 
         sdlog << itoa(tm.Hour,buf,10); 
         if (tm.Minute < 10) printSD(sdlog,cero); 
         sdlog << itoa(tm.Minute,buf,10); 
         if (tm.Second < 10) printSD(sdlog,cero); 
         sdlog << itoa(tm.Second,buf,10); 
         sdlog << tipo; 
         printSD(sdlog, guion); 
         sdlog << mensaje;
         printSD(sdlog, guion); 
         for (int i= 0; i < 4; i++)
           { 
            sdlog << itoa(rip[i],buf,10);
            if (i<3) printSD(sdlog,punto);
           }  
         sdlog << char(10); 
         sdlog.close();
      }
    }
 } 
 
void logIPP(const prog_uchar *str, int npin, char tipo, boolean activa)
  {
    
  char mensaje[20]="";
  char c;
  byte longi=0;
  while((c = pgm_read_byte(str++)))
    {
      mensaje[longi]=c;
      longi++;
    }
  mensaje[longi]=0;
  logIP(mensaje, npin, tipo, activa);
  }

//******************************************************************************************************  
void outputPins(WebServer &server, WebServer::ConnectionType type)
{ 
//    
  if (LOG==1) logIPP(Panel, 0, 'I', false);
  
  if (!autOK(server))  {server.httpUnauthorized(); return; }
      
  if (type == WebServer::HEAD) return;
  server.httpSuccess();

  printP3(server,htmlHead_i0, htmlHead_ix, htmlHead_i1);
  printP1(server,seguimiento);
//  printP3(server,seguimiento1,seguimientox,seguimiento2);
  if (peract != 0)   {
    server.printP(htmlRefresh_i);
    printIP(server, peract,10,comillascierre);
    }
  printP4(server, head_f, body_i,tablamenu, trcolor1_i);
  printP5(server, opc01,  opc13, tr_f, table_f,brn);
  
  printP1(server,tablapanel);
// ENLACES
    for (byte i = 0; i < 5; i++)
      if (getbit(bshowLink,i)==1)
       { 
        printP2(server,tr,tdcentercolspan3);
        
        printP1(server,href_i);
        printEE (server, dirEEurlLink+(i*LEN80), LEN80);
        printP1(server,href_cierre);
        printEE (server, dirEEdescLink+(i*LEN20), LEN20);
        printP1(server,href_f);
//        printP2(server,td_f,td);    
//        printEE (server, dirEEurlLink+(i*LEN80), LEN80);

        printP1(server,td_f);    
        printP1(server,tr_f);
        }
// ESCENAS
    for (byte i = 0; i < nEsc; i++)
      if (getbit(bshowEsc,i)==1)
       { 
        printP2(server,tr,td);
        printEE (server, dirEEdescEsc+(i*LEN20), LEN20);
        printP1(server,td_f);    
        
        printP1(server,td);
        printP1(server,hrefon);server.print(i+100); server.printP(ona_f);
        printP2(server,td_f,td);
        printP2(server,td_f,tr_f);
        }
// TEMPERATURAS LOCALES
  byte val;
  if (nTemp1 > 0) {
    for (int i=0; i<nTemp1; ++i)
      if (getbit8(bshowTemp,i) == 1)
      {
        printTDTR(server, 0, true, true);
        if (showN == 1) 
         {
          printparentesis(server,'s',i,10);
          for(int j = 0; j < 8; j++) {
            server.printP(b);
            server.print(addr1Wire[i][j], HEX);
            }
         }
        server.printP(b);
        printEE (server, dirEEdescTemp+(i*LEN20), LEN20);
        printTDTR(server, 1, false, true);
        printColspan(server,2,true);
        server.print(float(Mb.R[i])/100);
        printP1(server, celsius);     
        printTDTR(server, 1, true, true);
      }
  }
  
// ENTRADAS ANALÓGICAS
  for (byte i=0; i<16; ++i)    
    if (getbit8(bshowpin,i+41) == 1)
     {  
      printTDTR(server, 0, true, true);
//      if (showN == 1) printparentesis(server,'a',tabpin[i],10);
      if (showN == 1) printparentesis(server,'a',i,10);
      printEE (server, dirEEdescpinA+(i*LEN20), LEN20);
      printTDTR(server, 1, false, true);
      printColspan(server,2,true);
      server.print(float(Mb.R[i+baseAna])*factorA[i]); 
      printP1(server, b);
      printTDTR(server, 1, true, true);
     }

// ENTRADAS DIGITALES
    for (byte i = 0; i<16; ++i)
      if (getbit8(bshowpin,i) == 1)
        { 
        printP2(server,tr,td);
        if (showN == 1) printparentesis(server,'d',tabpin[i],10);
        printEE (server, dirEEdescpinD+(i*LEN20), LEN20);
        printP1(server, td_f);

        if (tipoED[i]==0)      // entrada digital ON/OFF
          {
          val = digitalRead(tabpin[i]);
          printP3(server,(val==0) ? td : th, (val==0) ? OFF : ON, (val==0) ? td_f : th_f);
          if (i<=7)    // contadores
            {
            printP1(server,td);
            printPiP(server,resetcontp,i,barramayor);
            server.print(contadores[i]);
            printP2(server,a_f,td_f);
            }
          }
        if (tipoED[i]>0)      // entrada digital DHT11, DHT21, DHT22
          {
          printP1(server,td);
          server.print(vanaTED[i]); 
          printP1(server, celsius);
          printP1(server,td_f);
          printP1(server,td);
          server.print(vanaHED[i]); 
          printP1(server, porciento);
          printP1(server,td_f);
          }
       }

// SALIDAS DIGITALES
  if (numOD > 0)   {
    for (byte i = 17; i < nDIG-1; ++i)
      if (getbit8(bshowpin,i) == 1)
       { 
          char buf[3];
          itoa(tabpin[i],buf,10);
          val = valorpin[digitalRead(tabpin[i])];
          
          printP2(server,tr, td);
          if (showN == 1) printparentesis(server,'d',tabpin[i],10);
          printEE (server, dirEEdescpinD+(i*LEN20), LEN20);
          printP1(server,td_f);

          printP1(server,(val==0) ? td : th);
          server.printP(hrefon);  server.print(buf); server.printP(ona_f);
          printP1(server,(val==0) ? td_f : th_f);
          
          printP1(server,td);
          server.printP(hrefoff);  server.print(buf); server.printP(offa_f);
          printP2(server,td_f,tr_f);
        }
      }
      
//  PUNTOS REMOTOS
  if (modo!=2)
    {
    // TEMPERATURAS REMOTAS
    for (byte i =0; i < maxTempr; i++) 
      {
        if (getbit8(rbshowpin,i+120)==1) 
        {   // temperaturas remotas
          char buf[3];
          itoa(i,buf,10);
          printTDTR(server, 0, true, true);
          if (showN == 1) 
            {
             server.printP(paren_i);
             printPiP(server,rs,i,paren_f1b);
            }
          printEE (server, dirEEdescTempr+(i*LEN20), LEN20);
          if (showN == 1) {
            server.printP(b);
            printparentesis(server,'n',nodotempr[i],8); 
            printparentesis(server,'p',remTpin[i],10); 
            }
          byte n=buscaNodoenTab(nodotempr[i]);
          if ((n!=99) && (tlastnodo[n]>1))
            printcorchete(server,tlastnodo[n],10);
          printP1(server, td_f);
          printColspan(server,2,true);
          if (tlastnodo[n]<60)
            {
            server.print(float(Mb.R[i+baseTempr])/100); server.printP(celsius);
            }
          else
            printP1(server,nodisponible);
          printTDTR(server, 1, true, true);
          }
      }
     
    // SEÑALES REMOTAS 
    for (byte i =0; i < maxRem; i++) 
      {
      if (getbit8(rbshowpin,i) == 1)  
        { 
        char buf[3];
        itoa(i,buf,10);
        printTDTR(server, 0, true, false);
//        
        if (i>=0 && i<20) {    // entradas ANALÓGICAS remotas
          printP1(server, td);
          if (showN == 1)   {
             server.printP(paren_i);
             printPiP(server,ra,i,paren_f1b);     }
            
          printEE (server, dirEEdescAnalr+(i*LEN20), LEN20);
            
          if (showN == 1) 
            {
              server.printP(b);
              printparentesis(server,'n',remNodo[i],8);
              printparentesis(server,'p',remPin[i],10);
            }
          byte n=buscaNodoenTab(remNodo[i]);
          if ((n!=99) && (tlastnodo[n]>1))
            printcorchete(server,tlastnodo[n],10);
          printP1(server,td_f);
          printColspan(server,2,true);
          server.print(float(Mb.R[i+baseAnar])*factorArem[i]); 
          printP1(server, td_f);
          }
//          
        if (i>=20 && i<40) {    // entradas digitales remotas
          val = getbit8(Mb.C8,i+44);  // AQUÍ SE LEE EL VALOR DEL PIN REMOTO, a partir pos 64
          printP1(server, td);
          if (showN == 1) 
            {
             server.printP(paren_i);
             printPiP(server,letrasred,i,paren_f1b);
            }
          printEE (server, dirEEdescEDigr+((i-20)*LEN20), LEN20);
          if (showN == 1) {
            server.printP(b);
            printparentesis(server,'n',remNodo[i],8);
            printparentesis(server,'p',remPin[i],10); }
          byte n=buscaNodoenTab(remNodo[i]);
          if ((n!=99) && (tlastnodo[n]>1))
            printcorchete(server,tlastnodo[n],10);
          printP1(server,td_f);
          printP1(server,(val==0) ? td : th);
          server.printP(val ? ON : OFF);
          printP1(server,(val==0) ? td_f : th_f);
          }
//          
        if (i>=40 && i<60) {  // Salidas digitales remotas
          if ((getbit8(rbshowpin,i) ==1))
            {
            val = getbit8(Mb.C8,i+44);  // AQUÍ SE LEE EL VALOR DEL PIN REMOTO, a partir pos 84
            printP1(server, td);
            if (showN == 1) 
              {
               server.printP(paren_i);
               printPiP(server,letrasrsd,i,paren_f1b);
              }
            printEE (server, dirEEdescSDigr+((i-40)*LEN20), LEN20);
            if (showN == 1) {
              server.printP(b);
              printparentesis(server,'n',remNodo[i],8);
              printparentesis(server,'p',remPin[i],10); }
            byte n=buscaNodoenTab(remNodo[i]);
           if ((n!=99) && (tlastnodo[n]>1))
              printcorchete(server,tlastnodo[n],10);
            printP1(server,td_f);
            
            printP1(server,(val==0) ? td : th);
            printPiP(server, hrefonr, i, ona_f);
            printP1(server,(val==0) ? td_f : th_f);
            
            printP1(server,td);
            printPiP(server, hrefoffr, i, offa_f);
            printP1(server,td_f);
            }
          }
        if (i>=60 && i<maxRem) {  // Salidas relé remotos
          if ((getbit8(rbshowpin,i) ==1))
            {
            val = getbit8(Mb.C8,i+44);  // AQUÍ SE LEE EL VALOR DEL PIN REMOTO, a partir pos 104
            printP1(server, td);
            if (showN == 1) 
              {
               server.printP(paren_i);
               printPiP(server,rr,i,paren_f1b);
              }
           printEE (server, dirEEdescReler+((i-60)*LEN20), LEN20);
           if (showN == 1) {
              server.printP(b);
              printparentesis(server,'n',remNodo[i],8);
              printparentesis(server,'p',remPin[i],10); 
              }
            byte n=buscaNodoenTab(remNodo[i]);
           if ((n!=99) && (tlastnodo[n]>1))
              printcorchete(server,tlastnodo[n],10);
            printP1(server,td_f);
            printP1(server,(val==0) ? td : th);
            printPiP(server, hrefonr, i, ona_f);  
            printP1(server,(val==0) ? td_f : th_f);
            
            printP1(server,td);
            printPiP(server, hrefoffr, i, offa_f);  
            printP1(server,td_f);
            }
          }
        server.printP(tr_f);
        }
      }
  
    // SEÑALES REMOTAS 433
    for (byte i =0; i < max433; i++) 
      {
      if (getbit8(rbshow433,i) == 1)  
        { 
        char buf[3];
        itoa(i,buf,10);
        printTDTR(server, 0, true, false);
        val = getbit8(Mb.C8,i+168);  // AQUÍ SE LEE EL VALOR almacenado del disp. 433
        printP1(server, td);
        if (showN == 1) 
          {
           server.printP(paren_i);
           printPiP(server,rk,i,paren_f1b);
          }
        printEE (server, dirEEdesc433+(i*LEN20), LEN20);
        if (showN == 1) {
          server.printP(b);
          printparentesis(server,'X',433,10); 
          }
        printP1(server, td_f);

        printP1(server,(val==0) ? td : th);
        printPiP(server, hrefon4, i, ona_f);  
        printP1(server,(val==0) ? td_f : th_f);
        
        printP1(server,td);
        printPiP(server, hrefoff4, i, offa_f);  
        printP1(server,td_f);
        server.printP(tr_f);
        }
      } 
    } 
//      
  
  printP2(server, tr, tdcolspan3);   

  if (RTC.read(tm)) 
    { 
    printDayName(server, tm.Wday);
    server.printP(comablanco);      
    if (tm.Day < 10) server.printP(cero); server.print (tm.Day); server.printP(barra);
    printMes(server,tm.Month);
    server.printP(barra);
    server.print (tmYearToCalendar(tm.Year)); server.printP(blancos4);
    if (tm.Hour < 10) server.printP(cero); server.print (tm.Hour); server.printP(dospuntos);
    if (tm.Minute < 10) server.printP(cero); server.print (tm.Minute); server.printP(dospuntos);
    if (tm.Second < 10) server.printP(cero); server.print (tm.Second);
    }
  else    
    if (RTC.chipPresent())      
      server.printP(rtcparado);
    else 
      server.printP(rtcnodisponible);
  printP2(server,td_f,tr_f);

  printP2(server, trcolor1_i,tdcolspan3);
  printP2(server, letrav, b);
  server.print(miversion);
  printP1(server,punto);
  server.print(misubver);
  printP3(server, b,elconuco,b);
  server.print(mifreeRam());
  printP1(server,barra);
  server.print(velocidad);
  printP2(server, td_f, tr_f);
  printP3(server,table_f, body_f, html_f);
}

void writeMenu(WebServer &server, byte opcprin, byte opcsec)
  {
  server.printP(htmlHead_i0);
  server.printP(htmlHead_ix);
  server.printP(htmlHead_i1);
  printP4(server,head_f,body_i,tablamenu,trcolor1_i);    // formato menú
  if (opcprin==1) printP1(server,opc01); else printP1(server,opc11);  // PANEL    
  if (opcprin==3) printP1(server,opc03); else printP1(server,opc13);  // CONFIGURACIÓN
  if (opcprin==2) printP1(server,opc02); else printP1(server,opc12);  // PROGRAMACIÓN
  if (modo!=2)
    if (opcprin==5) printP1(server,opc05); else printP1(server,opc15);  // RADIO 2.4 Ghz
  if (modo!=2)
    if (opcprin==7) printP1(server,opc07); else printP1(server,opc17);  // RADIO 433 mHZ
//  if (modo==0)  // Avanzado
    if (opcprin==4) printP1(server,opc04); else printP1(server,opc14);  // AVANZADO
  printP2(server,tr_f,table_f);
  
  printP2(server,tablamenu,trcolor1_i);    // formato menú
  if (opcprin==1)    // PANEL
    {
    }
  if (opcprin==2)    // PROGRAMACIÓN
    {
    if (opcsec==5) printP1(server,opc025); else printP1(server,opc125);  // Semanal
    if (opcsec==1) printP1(server,opc021); else printP1(server,opc121);  // Semanal
    if (opcsec==2) 
      if (modo==5)    // modo clima
        printP1(server,opc022clima); 
      else
        printP1(server,opc022); 
    else 
       if (modo==5)    // modo clima
         printP1(server,opc122clima); 
       else
          printP1(server,opc122);  // Eventos
    if (opcsec==3) printP1(server,opc023); else printP1(server,opc123);  // Fechas
    if (opcsec==4) printP1(server,opc024); else printP1(server,opc124);  // Escenas
    }
  if (opcprin==3)    // CONFIGURACIÓN
    {
    if (opcsec==0) printP1(server,opc030); else printP1(server,opc130);  // Dispositivo
    if (opcsec==10) printP1(server,opc03a); else printP1(server,opc13a);  // Mis enlaces
    if (opcsec==1) printP1(server,opc031); else printP1(server,opc131);  // Sondas Temp.
    if (opcsec==7) printP1(server,opc037); else printP1(server,opc137);  // E/S Analog
    if (opcsec==8) if (modo==5) printP1(server,opc038clima); else printP1(server,opc038); 
    else  if (modo==5) printP1(server,opc138clima); else printP1(server,opc138); 
    if (opcsec==9) printP1(server,opc039); else printP1(server,opc139);  // Sal. Digitales
    if (opcsec==3) printP1(server,opc033); else printP1(server,opc133);  // Red  
    if (opcsec==5) printP1(server,opc035); else printP1(server,opc135);  // Seguridad  
    if (opcsec==6) printP1(server,opc036); else printP1(server,opc136);  // Fecha/Hora
    }
  if (modo==0)  // Avanzado
    if (opcprin==4)    // AVANZADO
      {
      if (opcsec==1) printP1(server,opc041); else printP1(server,opc141);  // logs
      if (opcsec==2) printP1(server,opc042); else printP1(server,opc142);  // files
      if (opcsec==5) printP1(server,opc045); else printP1(server,opc145);  // logs
      if (opcsec==6) printP1(server,opc046); else printP1(server,opc146);  // files
      if (opcsec==3) printP1(server,opc043); else printP1(server,opc143);  // show EEPROM
      if (opcsec==4) printP1(server,opc044); else printP1(server,opc144);  // reinicia EEPROM
      }
  if (modo !=2)    // MODO NO LOCAL
    if (opcprin==5)    // Radio 2.4 Ghz
      {
      if (opcsec==1) printP1(server,opc051); else printP1(server,opc151);  // RADIO
      if (opcsec==5) printP1(server,opc055); else printP1(server,opc155);  // SONDAS REMOTAS
      if (opcsec==2) printP1(server,opc052); else printP1(server,opc152);  // E/S ANAL REMOTAS
      if (opcsec==6) printP1(server,opc056); else printP1(server,opc156);  // ENT. DIG. REMOTAS
      if (opcsec==7) printP1(server,opc057); else printP1(server,opc157);  // SAL. DIG. REMOTAS
      if (opcsec==8) printP1(server,opc058); else printP1(server,opc158);  // RELÉS REMOTOS
      if (modo!=1)
        {
        if (opcsec==3) printP1(server,opc053); else printP1(server,opc153);  // CONF. NODO
        if (opcsec==4) printP1(server,opc054); else printP1(server,opc154);  // BUSCA NODOS
        if (opcsec==9) printP1(server,opc05a); else printP1(server,opc15a);  // reset nodo
        }
      }
  if (modo !=2)    // MODO NO LOCAL
    if (opcprin==7)    // Radio 433 Mhz
      {
      if (opcsec==1) printP1(server,opc071); else printP1(server,opc171);  // RADIO
      if (opcsec==2) printP1(server,opc072); else printP1(server,opc172);  // DISPOSITIVOS
      if (modo!=1)
        if (opcsec==3) printP1(server,opc073); else printP1(server,opc173);  // LEER CODIGO
      }
   printP2(server,tr_f,table_f);
  }

void setupPrgHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sprghtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  int mp = 2;    // número de parámetros por fila

  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    int indice,resto,mival;
    for (int i=0;i<5;i++) actPrg[i]=0;
    do   
      {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      indice = param_number / mp;
      resto = param_number % mp;
      mival = atoi(value);
      if (resto==0) 
        for (int j=0; j<LEN20;j++) 
          EEwrite(dirEEdescPrg+(indice*LEN20)+j, value[j]);
      if (resto==1) actPrg[indice]=mival;
      } while (repeat);

    EEPROM_writeAnything(dirEEactprg, actPrg);
    server.httpSeeOther(PREFIX "/sprg.html"); 
    return;
    }
  
  server.httpSuccess();
  writeMenu(server, 2, 5);
  printP5(server,brn,Form_action,setupprg,Form_post,Form_input_send);
  printP3(server,tablaconf,trcolor1_i,td_if);
  printTDP(server, descripcion, false, true);
  printTDP(server, activo, false, true);
  printP1(server,tr_f);
  char buf[3];
  for (int i=0;i<5;i++) 
    {
      int mpi=mp*i;
      printP1(server,trcolor1_i);
      printP3(server,td,programa,b);
      server.print(i+1);                   
      printP1(server,td_f);
      
      if (actPrg[i]==1) printP1(server,th); else printP1(server,td);
      printP1(server,Form_input_text_start);      
      server.print(mpi);    // número de parámetro                  
      server.printP(Form_input_value);        
      printEE (server, dirEEdescPrg+(i*LEN20), LEN20);
      printPiP(server,Max_length,LEN20-1,size_i);
      server.print(LEN20-1);
      printP1(server,Form_input_end);
      if (actPrg[i]==1) printP1(server,th_f); else printP1(server,td_f);
      
      miCheckBox(server, "", (actPrg[i]==1), itoa(mpi+1,buf,10), "1", (actPrg[i]==1),true);
    }
  server.printP(tr_f);
  server.printP(table_f);
  
  //print the send button
  printP4(server,Form_input_send,Form_f,body_f,html_f);  
}

void setupLinksHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN80];
  int param_number = 0;
  int mp = 3;    // número de parámetros por fila
  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    int indice,resto,mival;
    bshowLink=0;
    do   
      {
      repeat = server.readPOSTparam(name, LEN5, value, LEN80);
      if (!repeat) break;
      param_number = atoi(name);  
      indice = param_number / mp;
      resto = param_number % mp;
      mival = atoi(value);
      if (resto==0) 
        for (int j=0; j<LEN20;j++) 
          EEwrite(dirEEdescLink+(indice*LEN20)+j, value[j]);
      if (resto==1) 
        for (int j=0; j<LEN80;j++) 
          EEwrite(dirEEurlLink+(indice*LEN80)+j, value[j]);
      if (resto==2) 
        setbit8(&bshowLink,indice,mival);
      } while (repeat);

    EEPROM_writeAnything(dirEEbshowLink, bshowLink);
    server.httpSeeOther(PREFIX "/slinks.html"); 
    return;
    }
  
  server.httpSuccess();
  writeMenu(server, 3, 10);
  printP5(server,brn,Form_action,setuplinks,Form_post,Form_input_send);
  printP3(server,tablaconf,trcolor1_i,td_if);
  printTDP(server, descripcion, false, true);
  printTDP(server, urltext, false, true);
  printTDP(server, activo, false, true);
  printP1(server,tr_f);
  char buf[3];
  for (int i=0;i<5;i++) 
    {
      int mpi=mp*i;
      printP1(server,trcolor1_i);
      printP3(server,td,linkusuario,b);
      server.print(i+1);                   
      printP1(server,td_f);
      
      if (getbit(bshowLink,i)==1) printP1(server,th); else printP1(server,td);
      printP1(server,Form_input_text_start);      
      server.print(mpi);    // número de parámetro                  
      server.printP(Form_input_value);        
      printEE (server, dirEEdescLink+(i*LEN20), LEN20);
      printPiP(server,Max_length,LEN20-1,size_i);
      server.print(LEN20-1);
      printP1(server,Form_input_end);
      if (getbit(bshowLink,i)==1) printP1(server,th_f); else printP1(server,td_f);
      
      if (getbit(bshowLink,i)==1) printP1(server,th); else printP1(server,td);
      printP1(server,Form_input_text_start);      
      server.print(mpi+1);    // número de parámetro                  
      server.printP(Form_input_value);        
      printEE (server, dirEEurlLink+(i*LEN80), LEN80);
      printPiP(server,Max_length,LEN80-1,size_i);
      server.print(LEN80-1);
      printP1(server,Form_input_end);
      if (getbit(bshowLink,i)==1) printP1(server,th_f); else printP1(server,td_f);
      
      miCheckBox(server, "", (getbit(bshowLink,i)==1), itoa(mpi+2,buf,10), "1", (getbit(bshowLink,i)==1),true);
    }
  server.printP(tr_f);
  server.printP(table_f);
  
  //print the send button
  printP4(server,Form_input_send,Form_f,body_f,html_f);  
}

void setupEscHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (seschtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  int mp = 5;    // número de parámetros por fila

  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    int indice,resto,mival;
    for (int i=0;i<5;i++) for (int j=0; j<11;j++) bescena[i][j]=0;
    bshowEsc=0;
    do   
      {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      mival = atoi(value);
      if (param_number>=500)
        {
        if (param_number>=600)
          {
          setbit(&bshowEsc,param_number-600, mival);
          }  
        else
         {
          for (byte i=0; i<LEN20; i++)
            EEwrite(dirEEdescEsc+((param_number-500)*LEN20)+i, value[i]);
        }
        }
      else
        {
        param_number = atoi(name);     
        indice = param_number / mp;
        resto = param_number % mp;
        setbit8(bescena[resto],indice,mival);
        }
      } while (repeat);

    EEPROM_writeAnything(dirEEesc, bescena);
    EEPROM_writeAnything(dirEEbshowEsc, bshowEsc);
    server.httpSeeOther(PREFIX "/sesc.html"); 
    return;
    }
  
  server.httpSuccess();
  writeMenu(server, 2, 4);
  printP5(server,brn,Form_action,setupesc,Form_post,Form_input_send);
  printP3(server,tablaconf,trcolor1_i,td_if);
  printTDP(server, escena1, false, true);
  printTDP(server, escena2, false, true);
  printTDP(server, escena3, false, true);
  printTDP(server, escena4, false, true);
  printTDP(server, escena5, false, true);
  printP1(server,tr_f);

  printP1(server,trcolor1_i);
  printTDP(server, descripcion, false, true);
  char buf[3];
  for (int i=0;i<nEsc;i++) 
    {
    printP1(server,(getbit(bshowEsc,i)==1)?th:td);
    printP1(server,Form_input_text_start);      
    server.print(500+i);                   
    server.printP(Form_input_value);        
    printEE (server, dirEEdescEsc+(i*LEN20), LEN20);
    printPiP(server,Max_length,LEN20-1,size_i);
    server.print(LEN20-1);
    printP1(server,Form_input_end);
    miCheckBox(server,"",getbit(bshowEsc,i)==1,itoa(600+i,buf,10), "1", (getbit(bshowEsc,i)==1),false);   
    printP1(server,(getbit(bshowEsc,i)==1)?th_f:td_f);
    }
  server.printP(tr_f);
  for (int i=0;i<86;i++)      
   if (((modo==2) && (i<=15)) || (modo!=2))
    {  
    int mpi = mp * i;
    printP2(server,tr,td);
    if (i<=15) printEE (server, dirEEdescpinD+((i+17)*LEN20), LEN20);
    if ((i>=16) && (i<=35)) printEE (server, dirEEdescSDigr+((i-16)*LEN20), LEN20);
    if ((i>=36) && (i<=55)) printEE (server, dirEEdescReler+((i-36)*LEN20), LEN20);
    if (i>=56) printEE (server, dirEEdesc433+((i-56)*LEN20), LEN20);
    printP1(server,td_f);
    char *milabel;
    for (int j=0; j<nEsc; j++)
      {
      if (getbit8(bescena[j],i)==1) milabel="ON"; else milabel="Off";
      miCheckBox(server,milabel,getbit8(bescena[j],i)==1,itoa(mpi+j,buf,10), "1", (getbit8(bescena[j],i)==1),true);   
      }
    server.printP(tr_f);
    }
  server.printP(table_f);
  
  //print the send button
  printP4(server,Form_input_send,Form_f,body_f,html_f);  
}

void setupEveHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (svhtml, 0,'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  int mp = 15;    // número de parámetros por fila
  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    boolean enANA;
    int indice; int resto;
    for (int i=0; i<nPRGeve;i++) for (int j=0; j<4;j++) bPRGeve[i][j]=0;
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      indice =(param_number / mp);
      resto= (param_number % mp);
      int mival = atoi(value);
      if (param_number<999)    
          {      
//          if (resto==0) setbit8(bacteve,indice,mival);
          if (resto==1) setbit8(beveacttipo,indice,mival);  // Tipo activadora: local o remota
          if (resto==2) evenact[indice] = mival;            // señal activadora 
          if (resto==3) evenvalD[indice] = mival;           // valor activador si digital
          if (resto==4) evencomp[indice]=mival;             // tipo comparador: meno-igual o mayor-igual 
          if (resto==5) evenvalA[indice] = atof(value)*100; // valor consigna 
          if (resto==6) evenhis[indice] = atof(value)*100;  // histéresis  
          if (resto==7) setbit8(bevesaltipo,indice,mival);  // Tipo salida: local o remota
          if (resto==8) evensal[indice] = mival;            // señal a activar
          if (resto==9) setbit8(bevenniv,indice, mival);    // valor que debe tomar la señal de salida
          if (resto>=10) {setbit8(bPRGeve[resto-10],indice,mival);  }   // asociacion PRG
          }
      } while (repeat);
    guardarDatosEventos();
    server.httpSeeOther(PREFIX "/sv.html"); return;
    }
  
  server.httpSuccess();
  writeMenu(server, 2,2);
  printP5(server, brn,Form_action, Form_eve, Form_post, Form_input_send);
  
  printP1(server,tablaconf);
  server.printP(trcolor1_i);
  printP8(server,td,evento,td_f,tdcolspan5,asociaraprograma,tdcolspan2,disparador,td_f);

  printTDP(server, dig, false, true);
  server.printP(tdcolspan3);
  printTDP(server, ana, false, false);
  server.printP(td_f);
  server.printP(tdcolspan3);
  printTDP(server, b, false, false);
  server.printP(td_f);
  server.printP(tr_f);

  server.printP(trcolor1_i);
  printTDP(server, n, false, true);
  printTDP(server, uno, false, true);
  printTDP(server, dos, false, true);
  printTDP(server, tres, false, true);
  printTDP(server, cuatro, false, true);
  printTDP(server, cinco, false, true);
  printTDP(server, lor, false, true);
  printTDP(server, senal, false, true);
  printTDP(server, onoff, false, true);
  printTDP(server, compa, false, true);
  printTDP(server, valor, false, true);
  printTDP(server, hist, false, true);
  printTDP(server, lor, false, true);
  printTDP(server, senalsalida, false, true);
  printTDP(server, onoff, false, true);
  server.printP(tr_f);
// 
/////////////////////////////////////////////////////////
  char buf[3];
  for (byte i=0;i<nPRG;i++)     
    {
    boolean colorea = ((getbit8(bPRGeve[0],i)==1) || (getbit8(bPRGeve[1],i)==1) ||
                       (getbit8(bPRGeve[2],i)==1) || (getbit8(bPRGeve[3],i)==1) ||
                       (getbit8(bPRGeve[4],i)==1));
    boolean actlocal= (getbit8(beveacttipo,i)==0);
    boolean actdigital= (evenact[i] <= 32) || ((evenact[i] >= 71) && (evenact[i] <= 170));
    boolean sallocal= (getbit8(bevesaltipo,i)==0);

    int  mpi = mp * i;
    int indice = (i*12)+420;  // parámetros del 420/426 en adelante
    printP1(server,tr);   

    if (colorea) printP1(server,th); else printP1(server,td);
    server.print(i+1);
    if (colorea) printP1(server,th_f); else printP1(server,td_f);
    
    for (byte j=0;j<5;j++)
      miCheckBox(server, "",(getbit8(bPRGeve[j],i)==1),itoa(mpi+10+j,buf,10),"1",getbit8(bPRGeve[j],i)==1,true);

    if (colorea)
      {
      if (colorea) printP1(server,th); else printP1(server,td);
      if (modo==2)
        printP1(server,Local);
      else
        printcampoSiNo(server, mpi+1, getbit8(beveacttipo,i), Remota,Local, true, false);    // valor local/remota
      if (colorea) printP1(server,th_f); else printP1(server,td_f);

      if (colorea) printP1(server,th); else printP1(server,td);
      server.printP(Select_name);
      server.print(mpi+2);   server.printP(barramayor);
      if (actlocal) // señal activadora es local
        {
        for (byte j=0; j<16; j++)  {  // añade entradas digitales locales
          printPiP(server, optionvalue, j, barraatras);
          if (j == evenact[i]) server.printP(selected);
          if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
          printEE (server, dirEEdescpinD+(j*LEN20), LEN20);
          }
        for (byte j=17; j<33; j++)  {  // añade salidas digitales locales
          printPiP(server, optionvalue, j, barraatras);
          if (j == evenact[i]) server.printP(selected);
          if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
          printEE (server, dirEEdescpinD+(j*LEN20), LEN20);
          }
        for (byte j=0; j<nANALO; j++)  {  // añade entradas analógicas locales
          printPiP(server, optionvalue, j+33, barraatras);
          if (j+33 == evenact[i]) server.printP(selected);
          if (showN == 1) printPiP(server, mayorparen, j+33, parenguion); else server.printP(cierre);
          printEE (server, dirEEdescpinA+(j*LEN20), LEN20);
          }
        for (byte j=0; j<maxTemp; j++)   // añade temperaturas locales
          if (getbit8(bshowTemp,j) == 1)        {             
            printPiP(server, optionvalue, j+171, barraatras);
            if (j+171 == evenact[i]) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j+171, parenguion); else server.printP(cierre);
            printEE (server, dirEEdescTemp+(j*LEN20), LEN20);
            }
        for (byte j=0; j<1; j++)   // añade precio kwh
          if (getbit8(bshowTemp,j) == 1)        {             
            printPiP(server, optionvalue, j+254, barraatras);
            if (j+254 == evenact[i]) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j+254, parenguion); else server.printP(cierre);
            printP1(server,preciokwh);
            }
          }
      else    // señal activadora es remota
        {
        for (int j=0; j<30; j++)    // añade salidas digitales 433
          if (getbit8(rbshow433,j)==1)          {
            printPiP(server,optionvalue,j,barraatras);
            if (evenact[i] == j) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
            printEE (server, dirEEdesc433+(j*LEN20), LEN20);
          }
        for (byte j=0; j<20; j++)    // añade entradas digitales remotas
          if (getbit8(rbshowpin,j+20) == 1)        {             
            printPiP(server, optionvalue, j+71, barraatras);
            if (j+71 == evenact[i]) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j+71, parenguion); else server.printP(cierre);
            printEE (server, dirEEdescEDigr+(j*LEN20), LEN20);
          }
        for (byte j=0; j<maxReler; j++)    // añade salidas relé remotas
          if (getbit8(rbshowpin,j+60) == 1)        {             
            printPiP(server, optionvalue, j+111, barraatras);
            if (j+111 == evenact[i]) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j+111, parenguion); else server.printP(cierre);
            printEE (server, dirEEdescReler+(j*LEN20), LEN20);
          }
        for (byte j=0; j<20; j++)   // añade entradas analogicas remotas 
          if (getbit8(rbshowpin,j)==1) { 
            printPiP(server, optionvalue, j+51, barraatras);
            if (j+51 == evenact[i]) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j+51, parenguion); else server.printP(cierre);
            printEE (server, dirEEdescAnalr+(j*LEN20), LEN20);
          }
        for (byte j=0; j<maxTempr; j++)   // añade temperaturas remotas 
          if (getbit8(rbshowpin,j+120)==1) { 
            printPiP(server, optionvalue, j+191, barraatras);
            if (j+191 == evenact[i]) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j+191, parenguion); else server.printP(cierre);
            printEE (server, dirEEdescTempr+(j*LEN20), LEN20);
          }
        }
      printP2(server,  Select_f, td_f);
      if (actdigital)    // la señal activadora es digital
        {
        printP1(server,th);   // valor ON/OFF que dispara
        printcampoSiNo(server, mpi+3, evenvalD[i], on, off,true,false);
        server.printP(th_f); 
        printP6(server,th,th_f,th,th_f,th,th_f);
        }
      else
        {
        printP2(server,th,th_f);
        printP1(server,th);   // tipo comparación 
        server.printP(Select_name);
        server.print(mpi+4);   server.printP(barramayor);
        for (byte j=0; j<2; j++)  {  // añade salidas posibles
          printPiP(server, optionvalue, j, barraatras);
          if (evencomp[i]==j) server.printP(selected);
          server.printP(cierre);
          server.printP((j==0)?mayoroigual:menoroigual);
          server.printP(option_f);  
          }
        printP2(server,  Select_f,th_f);
        printP1(server,th);   // valor analÃ³gico que dispara 
        printcampoF(server,mpi+5,float(evenvalA[i])/100, 5,2, false);
        printP1(server,th_f); 
        printP1(server,th);    // valor histÃ©resis que dispara 
        printcampoF(server,mpi+6,float(evenhis[i])/100, 5,2, false);
        printP1(server,th_f); 
        }
      if (colorea) printP1(server,th_f); else printP1(server,td_f);
      printP1(server,th);
      if (modo==2)
        printP1(server,Local);
      else
        printcampoSiNo(server, mpi+7, getbit8(bevesaltipo,i), Remota,Local, true, false);    // valor local/remota
      printP2(server,th_f,th);
      server.printP(Select_name);
      server.print(mpi+8);   server.printP(barramayor);  // señal de salida
      if (sallocal)    // salida a activar es local
        {
        for (byte j=17; j<33; j++)   {
          printPiP(server,optionvalue,j,barraatras);
          if (evensal[i] == j) server.printP(selected);
          if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
          printEE (server, dirEEdescpinD+(j*LEN20), LEN20);
          }  
        for (byte m=0; m<3; m++)  {  // añade eventos correo
          server.printP(optionvalue); server.print(m+despMail); server.printP(barraatras);
          if (m+despMail == evensal[i]) server.printP(selected);
          if (showN == 1) printPiP(server, mayorparen, m+despMail, parenguion); else server.printP(cierre);
          server.print(mailDEST[m]); }
        }
      else      // remotas
        {
        for (int j=0; j<30; j++)    // Salidas digitales 433
          if (getbit8(rbshow433,j)==1)
            {
            printPiP(server,optionvalue,j,barraatras);
            if (evensal[i] == j) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
            printEE (server, dirEEdesc433+(j*LEN20), LEN20);
            server.printP(option_f);
            }
        for (int j=91; j<111; j++)    // Salidas digitales RF24
          if (getbit8(rbshowpin,j-51)==1)      {
            printPiP(server,optionvalue,j,barraatras);
            if (evensal[i] == j) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
            printP2(server,saldigremota,b);
            server.print(j-91,DEC);
          }
        for (int j=111; j<171; j++)    // Relés remotos RF24
          if (getbit8(rbshowpin,j-51)==1)      {
            printPiP(server,optionvalue,j,barraatras);
            if (evensal[i] == j) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
            printEE (server, dirEEdescReler+((j-111)*LEN20), LEN20);
          }
        }
      printP3(server, option_f, Select_f,th_f);
      
      server.printP(th);    // Nivel de la salida
      printcampoSiNo(server, mpi+9, getbit8(bevenniv,i), on,off, true, false);
      server.printP(th_f);    // Nivel de la salida
      }
    }
  
    
  server.printP(table_f);
  printP4(server, Form_input_send,Form_f,body_f,html_f);
}

void setupClimaHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sclimahtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  int mp = 15;    // número de parámetros por fila
  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    boolean enANA;
    int indice; int resto;
    for (int i=0; i<nPRGeve;i++) for (int j=0; j<4;j++) bPRGeve[i][j]=0;
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      indice =(param_number / mp);
      resto= (param_number % mp);
      int mival = atoi(value);
      if (param_number<999)    
          {      
//          if (resto==0) setbit8(bacteve,indice,mival);
          if (resto==1) setbit8(beveacttipo,indice,mival);  // Tipo activadora: local o remota
          if (resto==2) evenact[indice] = mival;            // señal activadora 
          if (resto==3) evenvalD[indice] = mival;           // valor activador si digital
          if (resto==4) evencomp[indice]=mival;             // tipo comparador: meno-igual o mayor-igual 
          if (resto==5) evenvalA[indice] = atof(value)*100; // valor consigna 
          if (resto==6) evenhis[indice] = atof(value)*100;  // histéresis  
          if (resto==7) setbit8(bevesaltipo,indice,mival);  // Tipo salida: local o remota
          if (resto==8) evensal[indice] = mival;            // señal a activar
          if (resto==9) setbit8(bevenniv,indice, mival);    // valor que debe tomar la señal de salida
          if (resto>=10) {setbit8(bPRGeve[resto-10],indice,mival);}   // asociacion PRG
          }
      } while (repeat);
    guardarDatosEventos();
    server.httpSeeOther(PREFIX "/sclima.html"); return;
    }
  
  server.httpSuccess();
  writeMenu(server, 2,2);
  printP5(server, brn,Form_action, Form_clima, Form_post, Form_input_send);
  
  printP1(server,tablaconf);

  server.printP(trcolor1_i);
  printTDP(server, n, false, true);
  printTDP(server, letraa, false, true);
  printTDP(server, circuito, false, true);
  printTDP(server, onoff, false, true);
  printTDP(server, termostato, false, true);
  printTDP(server, onoff, false, true);
  server.printP(tr_f);
// 
/////////////////////////////////////////////////////////
  char buf[3];
  for (byte i=0;i<nPRG;i++)     
    {
    boolean colorea = ((getbit8(bPRGeve[0],i)==1));
    boolean actlocal= (getbit8(beveacttipo,i)==0);
    boolean actdigital= (evenact[i] <= 32) || ((evenact[i] >= 71) && (evenact[i] <= 170));
    boolean sallocal= (getbit8(bevesaltipo,i)==0);

    int  mpi = mp * i;
    int indice = (i*12)+420;  // parámetros del 420/426 en adelante
    printP1(server,tr);   

    if (colorea) printP1(server,th); else printP1(server,td);
    server.print(i+1);
    if (colorea) printP1(server,th_f); else printP1(server,td_f);
      
  //    for (byte j=0;j<5;j++)
    for (byte j=0;j<1;j++)
      miCheckBox(server, "",(getbit8(bPRGeve[j],i)==1),itoa(mpi+10+j,buf,10),"1",getbit8(bPRGeve[j],i)==1,true);

    if (colorea)
      {
        printP1(server,th);
        server.printP(Select_name);
        server.print(mpi+8);   server.printP(barramayor);  // señal de salida
        if (sallocal)    // salida a activar es local
          {
          for (byte j=17; j<33; j++)   {
            printPiP(server,optionvalue,j,barraatras);
            if (evensal[i] == j) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
            printEE (server, dirEEdescpinD+(j*LEN20), LEN20);
            }  
          }
        printP3(server, option_f, Select_f,th_f);
        
        server.printP(th);    // Nivel de la salida
        printcampoSiNo(server, mpi+9, getbit8(bevenniv,i), on,off, true, false);
        server.printP(th_f);    // Nivel de la salida
      }
  
    if (colorea)
      {
      printP1(server,th); 
      server.printP(Select_name);
      server.print(mpi+2);   server.printP(barramayor);
      if (actlocal) // señal activadora es local
        {
        for (byte j=0; j<16; j++)  {  // añade entradas digitales locales
          printPiP(server, optionvalue, j, barraatras);
          if (j == evenact[i]) server.printP(selected);
          if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
          printEE (server, dirEEdescpinD+(j*LEN20), LEN20);
          }
        }
      printP2(server,  Select_f, td_f);
      if (actdigital)    // la señal activadora es digital
        {
        printP1(server,th);   // valor ON/OFF que dispara
        printcampoSiNo(server, mpi+3, evenvalD[i], on, off,true,false);
        server.printP(th_f); 
//        printP6(server,th,th_f,th,th_f,th,th_f);
        }
      printP1(server,th_f);
      }
    }
  
    
  server.printP(table_f);
  printP4(server, Form_input_send,Form_f,body_f,html_f);
}

void printSD(ofstream &sdlog, const prog_uchar *str)
  {
  char c;
  while((c = pgm_read_byte(str++)))
    sdlog << c;  
  }

void logTemp()
 {
  ofstream sdlog(namelogTem, ios::out | ios::app);  
  if (sdlog)
    {
     char buf[3];
     sdlog << tmYearToCalendar(tm.Year);
     if (tm.Month < 10) printSD(sdlog,cero); 
     sdlog << itoa(tm.Month+1,buf,10);
     if (tm.Day < 10) printSD(sdlog,cero); 
     sdlog << itoa(tm.Day,buf,10); 
//     printSD(sdlog, guion); 
     if (tm.Hour < 10) printSD(sdlog,cero); 
     sdlog << itoa(tm.Hour,buf,10); 
     if (tm.Minute < 10) printSD(sdlog,cero); 
     sdlog << itoa(tm.Minute,buf,10); 
     if (tm.Second < 10) printSD(sdlog,cero); 
     sdlog << itoa(tm.Second,buf,10); 
     printSD(sdlog, guion); 
     char buff[10];
     for (int i=0; i<80;i++)
       {
        sdlog << itoa((Mb.R[i]/100),buff,10); 
        printSD(sdlog,punto);
        sdlog << itoa((Mb.R[i] % 100),buff,10); 
        printSD(sdlog,puntoycoma);
       }
     sdlog << char(10) << char(13); 
     sdlog.close();
    }
 }

void pinVAL(int n, byte value)
  { 
    if (valorpin[digitalRead(n)] != value)
      {
       digitalWrite(n, valorpin[value]);
       setbit8(Mb.C8, n, value);
       EEwrite(dirEEestado+(n/8), Mb.C8[n/8]);    // el byte modificado
      }
  }

void pinVALcond(int n, byte value)
  { 
    if (digitalRead(n) != valorpin[value])
      {
       digitalWrite(n, valorpin[value]);
       setbit8(Mb.C8, n, value);
       EEwrite(dirEEestado+(n/8), Mb.C8[n/8]);    // el byte modificado
      }
  }
  
void procesaradmsg()
  {
    if (headerR.type == 1)    // recibe estado relé (sólo uno, el que se ha usado)
      {
       printS(SDrele); 
       printS(blancop); Serial.print(radmsgR.pin,DEC);
       printS(blancov); Serial.println(getbit(radmsgR.acc,radmsgR.pin),DEC);
       for (int i=0; i<maxRem; i++)
         if (remNodo[i]==headerR.from_node)
           {
           if (remPin[i]==radmsgR.pin)
             setbit8(Mb.C8, i+44, getbit(radmsgR.acc,radmsgR.pin));
           }
      }
    if (headerR.type == 2)      // recibe temperaturas 1-wire 
      {
       nTempr = radmsgR.acc;               // número sondas leídas del nodo
       boolean encontrado=false;
       byte pos=0;
       do {
         encontrado =((nodotempr[pos] == headerR.from_node) &&
                      (radmsgR.pin == remTpin[pos]));
         if (encontrado)
           break;
         else
           pos++;
        }
       while ((!encontrado) && (pos<=maxTempr));
       Mb.R[pos+baseTempr]=(radmsgR.act*256)+radmsgR.res;
      }
      
    if (headerR.type == 3)      // recibe valores analógicos
      {
       boolean encontrado=false;
       byte pos=0;
       do {
         encontrado =((headerR.from_node==remNodo[pos]) &&
                      (radmsgR.pin == remPin[pos]));
         if ((encontrado) && (true))
           if (getbit8(bsumatArem,pos)==0)
             {
              Mb.R[pos+baseAnar]=(radmsgR.act*256) + radmsgR.res;
             }
         pos++;
         }
       while (pos<=nANALOr);
      }
    if (headerR.type == 5)      // recibe valores analógicos integrados
      {
       boolean encontrado=false;
       byte pos=0;
       do {
         encontrado =((headerR.from_node==remNodo[pos]) &&
                      (radmsgR.pin == remPin[pos]));
         if ((encontrado) && (true))
           if (getbit8(bsumatArem,pos)==1)
             {
              unsigned long auxINT=(long(radmsgR.valores[0]) * 16777216) +
                                    (long(radmsgR.valores[1]) * 65536) + 
                                    (long(radmsgR.valores[2]) * 256) + 
                                    (radmsgR.valores[3]); 
              Mb.R[pos+baseAnar]=auxINT/1000;
             }
           pos++;
         }
       while (pos<=nANALOr);
      }
    if (headerR.type == 4)      // recibe configuración actual nodo remoto
      {
       printS(blancoNodo);     Serial.print(headerR.from_node,OCT);
       printS(blancoPeriodo);  Serial.print(radmsgR.valores[0]);
       printS(blancocanal);    Serial.print(radmsgR.valores[2]);
       printS(blancosondas);   Serial.println(radmsgR.acc);
      }
    if (headerR.type == 7)      // recibe addr de sonda DS18B20
      {
       printS(blancoNodo);     Serial.print(headerR.from_node,OCT);
       printS(blancosonda);    Serial.print(radmsgR.acc);
       printS(blancoaddr);   
       for (byte i=0;i<8;i++)
         Serial.print(radmsgR.valores[i],HEX);
       Serial.println();
      }
    if (headerR.type == 6)    // recibe estados señales dig (todos)
      {
       for (int i=0; i<maxRem; i++)
         if (remNodo[i]==headerR.from_node)
           {
           if (remPin[i]==0) setbit8(Mb.C8, i+44, getbit(radmsgR.acc,remPin[i]));
           if (remPin[i]==1) setbit8(Mb.C8, i+44, getbit(radmsgR.acc,remPin[i]));
           if (remPin[i]==2) setbit8(Mb.C8, i+44, getbit(radmsgR.acc,remPin[i]));
           if (remPin[i]==3) setbit8(Mb.C8, i+44, getbit(radmsgR.acc,remPin[i]));
           if (remPin[i]==4) setbit8(Mb.C8, i+44, getbit(radmsgR.acc,remPin[i]));
           if (remPin[i]==5) setbit8(Mb.C8, i+44, getbit(radmsgR.acc,remPin[i]));
           }
      }
    if (headerR.type == 8)      // memoria libre
      {
      }
  }  
  
boolean pinValr(byte pinr, byte valor)
  {
    byte cont=nretryrem;
    boolean ok=false;
    headerT.type=0;
    headerT.to_node=remNodo[pinr];
    radmsgT.acc=valor;
    radmsgT.pin=remPin[pinr];
    while ((!ok) && (cont >0))
      {
      ok = network.write(headerT,&radmsgT,sizeof(radmsgT));
      if (!ok)
        {
        network.begin(radCan[0], radNOD);
        delay(delayretryrem);
        }
      cont--;
      }
    if (ok)
      {
      delay(delayACKrem);  // necesario para actualizar estado en panel
      while (network.available())
        {
        radmsgR.acc=0;
        network.read(headerR,&radmsgR,sizeof(radmsgR));
        procesaradmsg();
        byte n=buscaNodoenTab(headerR.from_node);
        if (n!=99) 
          tlastnodo[n]=0;
        }  
      }     
    return ok;
  }

boolean pinValrCond(byte pinr, byte valor)
  {
    if (getbit8(Mb.C8,pinr+44 ) != valor)
      pinValr(pinr, valor);
  }
  
void onescena(int nesc)
  {
    for (int j=0; j<16; j++)    //  Salidas digitales locales
      pinVAL (tabpin[j+17],getbit8(bescena[nesc],j));
    for (int j=0; j<40; j++)    //  Salidas digitales remotas / relés remotos
      pinValr (j+40,getbit8(bescena[nesc],j+16));
    for (int j=0; j<30; j++)    //  dispositivos 433
      pinVal433 (j,getbit8(bescena[nesc],j+56));
  }
  
void onCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  int auxi = atoi(url_tail);
  if (auxi < nPINES)
    {
    pinVAL(auxi,1);
    if (LOG==1) logAcc("",auxi,'L',true);
    }
  else
    {
    onescena(auxi-100);
    if (LOG==1) logAcc("",auxi-100,'E',true);
    }
  server.httpSeeOther(PREFIX "/");
}

void offCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  int auxi = atoi(url_tail);
  if (auxi < nPINES)
    {
    pinVAL(auxi,0);
    if (LOG==1) logAcc("",auxi,'L',false);
    }
  server.httpSeeOther(PREFIX "/");
}

void onoffremoto(WebServer &server, WebServer::ConnectionType type, byte valor, int pin)
  {
    boolean ok = pinValr(pin,valor);
    if (ok)
      {
      server.httpSeeOther(PREFIX "/");
      }
    else
      {
      network.begin(radCan[0], radNOD);
      server.printP(origen); server.print(headerT.from_node,OCT); 
      server.printP(destino); server.print(headerT.to_node,OCT); 
      server.printP(noaccesible);
      }
    if (LOG==1) logAcc("",pin,'R',(valor==1));
  }

void onrCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  int auxi = atoi(url_tail);
  if (auxi < maxRem) 
    onoffremoto(server, type, 1, auxi);
}

void offrCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  int auxi = atoi(url_tail);
  if (auxi < maxRem) 
    onoffremoto(server, type, 0, auxi);
}

//    byte cont=nretryrem;
//    boolean ok=false;
//    headerT.type=0;
//    headerT.to_node=remNodo[pinr];
//    radmsgT.acc=valor;
//    radmsgT.pin=remPin[pinr];
//    while ((!ok) && (cont >0))
//      {
//      ok = network.write(headerT,&radmsgT,sizeof(radmsgT));
//      if (!ok)
//        {
//        network.begin(radCan[0], radNOD);
//        delay(delayretryrem);
//        }
//      cont--;
//      }
//    if (ok)
//      {
//      delay(delayACKrem);  // necesario para actualizar estado en panel
//      while (network.available())
//        {
//        radmsgR.acc=0;
//        network.read(headerR,&radmsgR,sizeof(radmsgR));
//        procesaradmsg();
//        byte n=buscaNodoenTab(headerR.from_node);
//        if (n!=99) 
//          tlastnodo[n]=0;
//        }  
//      }     
//    return ok;


void pinVal433(byte pin433, byte valor)
  {
  if (((tipo433[pin433]/10)==1) || ((tipo433[pin433]/10)==2) || ((tipo433[pin433]/10)==3) || ((tipo433[pin433]/10)==4))    // Chacon D-IO, DEMA, CONUCO
    {
    if ((addr433[pin433]!=0) && (unit433[pin433]!=0))
      {
      TX433activo=true;
      if (tipo433[pin433]==41)   // CONUCO
        {
        randomSeed(millis());
        auxrnd433=random(255);
        long auxaddr=(addr433[pin433]<<16) + (auxrnd433<<8) + addr433[pin433];
        NewRemoteTransmitter transmitter(auxaddr, radTX, 255, 2);
        noInterrupts();
        transmitter.sendUnit(unit433[pin433], valor);
        interrupts();
        long timeout=2000;
        long timeact=millis();
        while (((millis()-timeact)<timeout) && (pendiente433Rec==0));
//        Serial.println(millis()-timeact);
        procesa433();
        }
      if ((tipo433[pin433]==31) || (tipo433[pin433]==32))   // Chacon D-IO, New
        {
        NewRemoteTransmitter transmitter(addr433[pin433], radTX, 255, 2);
        transmitter.sendUnit(unit433[pin433], (valor==1));
        setbit8(Mb.C8, pin433+168, valor);
        }
      if ((tipo433[pin433]==21) || (tipo433[pin433]==22))   // Dema, Old
        {
        ActionTransmitter actionTransmitter(radTX, 141, 3);
        actionTransmitter.sendSignal(addr433[pin433],unit433[pin433],(valor==1));
        setbit8(Mb.C8, pin433+168, valor);
        }
//      guardarDatos433();
      TX433activo=false;
      }
    }
  if ((tipo433[pin433]/10)==0)    // sin unit ni device, sólo codeON y codeOFF
    {
    if (valor==0)
      {
      if (addr433[pin433]!=0)
        {
        TX433activo=true;
        setbit8(Mb.C8, pin433+168, valor);
        RemoteTransmitter::sendCode(radTX, code433Off[pin433], 141, 2);
        }
      }
    if (valor==1)
      {
      if (code433Off[pin433]!=0)
        {
        TX433activo=true;
        setbit8(Mb.C8, pin433+168, valor);
        RemoteTransmitter::sendCode(radTX, addr433[pin433], 141, 2);
        }
      }
     
    }
  }

void pinVal433Cond(byte pin433, byte valor)  // sólo si hay cambio
  {
  if (getbit8(Mb.C8, pin433+168) != valor)
    pinVal433(pin433,valor);    // señal remota 433
  }
  
void onoff433(WebServer &server, WebServer::ConnectionType type, byte valor, int disp)
  {
    pinVal433(disp, valor);
    server.httpSeeOther(PREFIX "/");
//    if (LOG==1) logAcc("",disp,'4',(valor==1));
    }

void on4Cmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  int auxi = atoi(url_tail);
  if (auxi < max433) 
    onoff433(server, type, 1, auxi);
}

void off4Cmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  int auxi = atoi(url_tail);
  if (auxi < max433) 
    onoff433(server, type, 0, auxi);
}

boolean enviado=true;

void ShowFileCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIP (url_tail, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  ifstream sdlog(url_tail);
  ArduinoOutStream cout(Serial);
  int c;
  if (sdlog.is_open()) 
    {
    while ((c = sdlog.get()) >= 0) 
      server.print((char)c);
    }
  else
    printP1(server,ficheronoencontrado);
    
  printP2(server,body_f,html_f);
}

void indexHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
  { 
  outputPins(server, type);  
  }

void leerDatosDevice()   {
  for (byte i=0; i<LEN20; i++) user1[i] = EEread(dirEEuser+i);
  for (byte i=0; i<LEN20; i++) pass1[i] = EEread(dirEEpass+i);
  
  EEPROM_readAnything (dirEEmodo, modo);
  EEPROM_readAnything (dirEEsdHay, sdHay);
  EEPROM_readAnything (dirEEprectemp, prectemp);
  EEPROM_readAnything (dirEEnumIA, numIA);
  EEPROM_readAnything (dirEEnumID, numID);
  EEPROM_readAnything (dirEEnumOD, numOD);
  EEPROM_readAnything (dirEEshown, showN);
  EEPROM_readAnything (dirEEperact, peract);
  EEPROM_readAnything (dirEEusepass, usepass);
  EEPROM_readAnything(dirEEbshowLink, bshowLink);
  for (int i=0; i<8; i++)
    Mb.C8[i]=EEread(dirEEestado+i);    // los 8 primeros bytes de Mb.C8=bestado
  EEPROM_readAnything (dirEEbshowpin, bshowpin);
  EEPROM_readAnything (dirEEtipoED, tipoED);
  EEPROM_readAnything (dirEEshowTemp, bshowTemp);
  EEPROM_readAnything (dirEEshowTemp, bshowTemp);
  EEPROM_readAnything (dirEEshowTemp, bshowTemp);
//  EEPROM_readAnything (dirEEfactorA, factorA);

  if (numIA > nANALO) numIA = nANALO;
  if (numIA+numID > nPINES) numID = nPINES - numIA;
  if (numIA+numID+numOD > nPINES) numOD = nPINES - numIA - numID;
  numPines = numIA+numID+numOD;
  EEPROM_readAnything(dirEEactprg, actPrg);
  EEPROM_readAnything(dirEEpmactVar, pmactVar);
  EEPROM_readAnything(dirEEtipoLCD, tipoLCD);
  EEPROM_readAnything(dirEEbshowEsc, bshowEsc);
  }
//
void leerDatosRed()    {
    EEPROM_readAnything (dirEEmac, EEmac);
    EEPROM_readAnything (dirEEip, EEip);
    EEPROM_readAnything (dirEEmask, EEmask);
    EEPROM_readAnything (dirEEgw, EEgw);
    EEPROM_readAnything (dirEEdns, EEdns);
    EEPROM_readAnything (dirEEwebport, EEwebPort);
    EEPROM_readAnything (dirEEmailPER, mailPER);  
    EEPROM_readAnything (dirEEmailACT, mailACT);  
    EEPROM_readAnything (dirEEmailSVR, mailSVR);  
    EEPROM_readAnything (dirEEmailPORT, mailPORT);  
    EEPROM_readAnything (dirEEmailUSER, mailUSER);  
    EEPROM_readAnything (dirEEmailPASS, mailPASS);  
    EEPROM_readAnything (dirEEmailSENDER, mailSENDER);  
    EEPROM_readAnything (dirEEmailDEST, mailDEST);  
    EEPROM_readAnything (dirEEdyndnsHost, dyndnsHost);  
    EEPROM_readAnything (dirEEdyndnsUser, dyndnsUser);  
    EEPROM_readAnything (dirEEdyndnsPass, dyndnsPass);  
  }
  
void guardarDatosRed()    {
    EEPROM_writeAnything (dirEEmac, EEmac);
    EEPROM_writeAnything (dirEEip, EEip);
    EEPROM_writeAnything (dirEEmask, EEmask);
    EEPROM_writeAnything (dirEEgw, EEgw);
    EEPROM_writeAnything (dirEEdns, EEdns);
    EEPROM_writeAnything (dirEEwebport, EEwebPort);  
    EEPROM_writeAnything (dirEEmailPER, mailPER);  
    EEPROM_writeAnything (dirEEmailACT, mailACT);  
    EEPROM_writeAnything (dirEEmailSVR, mailSVR);  
    EEPROM_writeAnything (dirEEmailPORT, mailPORT);  
    EEPROM_writeAnything (dirEEmailUSER, mailUSER);  
    EEPROM_writeAnything (dirEEmailPASS, mailPASS);  
    EEPROM_writeAnything (dirEEmailSENDER, mailSENDER);  
    EEPROM_writeAnything (dirEEmailDEST, mailDEST);  
    EEPROM_writeAnything (dirEEdyndnsHost, dyndnsHost);  
    EEPROM_writeAnything (dirEEdyndnsUser, dyndnsUser);  
    EEPROM_writeAnything (dirEEdyndnsPass, dyndnsPass);  
  }
    
void leerDatosRad()    {
    EEPROM_readAnything (dirEEredNodo, redNodo);
    EEPROM_readAnything (dirEEradACT, radACT);
    EEPROM_readAnything (dirEEradFUN, radFUN);
    EEPROM_readAnything (dirEEradVEL, radVEL);   
    EEPROM_readAnything (dirEEradNOD, radNOD);     
    EEPROM_readAnything (dirEEradCan, radCan);      
    EEPROM_readAnything (dirEEreactRad, reactRad);      
    EEPROM_readAnything (dirEEpmactRem, pmactRem);   
  }
  
void guardarDatosRad()    {
    EEPROM_writeAnything (dirEEredNodo, redNodo);
    EEPROM_writeAnything (dirEEradACT, radACT);
    EEPROM_writeAnything (dirEEradFUN, radFUN);
    EEPROM_writeAnything (dirEEradVEL, radVEL);    
    EEPROM_writeAnything (dirEEradNOD, radNOD);      
    EEPROM_writeAnything (dirEEradCan, radCan);     
    EEPROM_writeAnything (dirEEreactRad, reactRad);      
    EEPROM_writeAnything (dirEEpmactRem, pmactRem);      
  }

void creatablanodos()
  {
    numNodos=0;
    for (int i=0; i<maxNodos; i++) tnodosact[i] = 0;  // inicializa
    for (int i=0; i<maxRem; i++)  // recorremos los puntos remotos
      if (remNodo[i] != 0)    // si el nodo remoto no es cero
        {
        byte n=0;
        boolean encontrado=false;
        while ((!encontrado) && (n<numNodos))  // lo buscamos en la tabla de nodos
          {
          if (tnodosact[n]==remNodo[i])
            encontrado=true;
          else
            n=n+1;
          }
        if (!encontrado)    // si se encuentra
          {
          tnodosact[numNodos]=remNodo[i];    // se guarda el nodo
          numNodos=numNodos+1;               // incrementamos número de nodos
          }
        }
  }
  
void leerDatosRem()    {
    EEPROM_readAnything (dirEEremNodo, remNodo);
    EEPROM_readAnything (dirEEremPin, remPin);
    EEPROM_readAnything (dirEEremshowpin, rbshowpin); 
    EEPROM_readAnything (dirEEfactorArem, factorArem);  
    EEPROM_readAnything (dirEEoffsetArem, offsetArem);  
    EEPROM_readAnything (dirEEsumatA, bsumatA);  
    EEPROM_readAnything (dirEEsumatArem, bsumatArem);  
    // crea tabla nodos 
  }
  
void guardarDatosRem()    {
    EEPROM_writeAnything (dirEEremNodo, remNodo);
    EEPROM_writeAnything (dirEEremPin, remPin);
    EEPROM_writeAnything (dirEEremshowpin, rbshowpin);  
    EEPROM_writeAnything (dirEEfactorArem, factorArem);  
    EEPROM_writeAnything (dirEEoffsetArem, offsetArem);  
    EEPROM_writeAnything (dirEEsumatA, bsumatA);  
    EEPROM_writeAnything (dirEEsumatArem, bsumatArem);  
    // crea tabla nodos 
    creatablanodos();    // actualiza si se han variado los nodos en la sesión
  }
  
void leerDatos433()    {
    EEPROM_readAnything (dirEErshow433, rbshow433);
    EEPROM_readAnything (dirEEaddr433, addr433);
    EEPROM_readAnything (dirEEcode433Off, code433Off);
    EEPROM_readAnything (dirEEunit433, unit433);
    EEPROM_readAnything (dirEEtipo433, tipo433);
    EEPROM_readAnything (dirEEradACT433, radACT433);
    EEPROM_readAnything (dirEEreact433, react433);      
    for (int i=0; i<4; i++)
      Mb.C8[i+21]=EEread(dirEEest433+i);    // los 8 primeros bytes de Mb.C8=bestado
      }
  
void guardarDatos433()    {
    EEPROM_writeAnything (dirEErshow433, rbshow433);
    EEPROM_writeAnything (dirEEaddr433, addr433);
    EEPROM_writeAnything (dirEEcode433Off, code433Off);
    EEPROM_writeAnything (dirEEunit433, unit433);
    EEPROM_writeAnything (dirEEtipo433, tipo433);
    EEPROM_writeAnything (dirEEradACT433, radACT433);
    EEPROM_writeAnything (dirEEreact433, react433);      
    for (int i=0; i<4; i++)
      EEwrite(dirEEest433+i,Mb.C8[i+21]);    // los 8 primeros bytes de Mb.C8=bestado
    }
  
void leerDatosrTemp()    {
    EEPROM_readAnything (dirEEnodotempr, nodotempr);
    EEPROM_readAnything (dirEEremTpin, remTpin);
    EEPROM_readAnything (dirEEremshowpin, rbshowpin);  
  }
  
void guardarDatosrTemp()    {
    EEPROM_writeAnything (dirEEnodotempr, nodotempr);
    EEPROM_writeAnything (dirEEremTpin, remTpin);
    EEPROM_writeAnything (dirEEremshowpin, rbshowpin);  
  }
  
void leerDatosEventos()  {
  EEPROM_readAnything (dirEEevenact, evenact);
  EEPROM_readAnything (dirEEevencomp, evencomp);
  EEPROM_readAnything (dirEEevenvalA, evenvalA);
  EEPROM_readAnything (dirEEevenvalD, evenvalD);
  EEPROM_readAnything (dirEEevensal, evensal);
  EEPROM_readAnything (dirEEevenniv, bevenniv);
  EEPROM_readAnything (dirEEevenhis, evenhis);   
  EEPROM_readAnything (dirEEevenTemp, evenTemp);  
  EEPROM_readAnything (dirEEbeveacttipo, beveacttipo);  
  EEPROM_readAnything (dirEEbevesaltipo, bevesaltipo);  
  EEPROM_readAnything (dirEEbacteve, bacteve);
  EEPROM_readAnything (dirEEPRGeve, bPRGeve);
  }

void guardarDatosEventos()  {
  EEPROM_writeAnything (dirEEevenact, evenact);
  EEPROM_writeAnything (dirEEevencomp, evencomp);
  EEPROM_writeAnything (dirEEevenvalA, evenvalA);
  EEPROM_writeAnything (dirEEevenvalD, evenvalD);
  EEPROM_writeAnything (dirEEevensal, evensal);
  EEPROM_writeAnything (dirEEevenhis, evenhis);
  EEPROM_writeAnything (dirEEevenniv, bevenniv);  
  EEPROM_writeAnything (dirEEevenTemp, evenTemp);  
  EEPROM_writeAnything (dirEEbeveacttipo, beveacttipo);  
  EEPROM_writeAnything (dirEEbevesaltipo, bevesaltipo);  
  EEPROM_writeAnything (dirEEbacteve, bacteve);
  EEPROM_writeAnything (dirEEPRGeve, bPRGeve);
  }
    
void guardarDatosFechas()  {
  EEPROM_writeAnything (dirEEfecsal, fecsal);
  EEPROM_writeAnything (dirEEfecval, bfecval);
  EEPROM_writeAnything (dirEEfecdia, fecdia);
  EEPROM_writeAnything (dirEEfecmes, fecmes);
  EEPROM_writeAnything (dirEEfecano, fecano);
  EEPROM_writeAnything (dirEEfechor, fechor);
  EEPROM_writeAnything (dirEEfecmin, fecmin);    
  EEPROM_writeAnything (dirEEbfectipo, bfectipo);    
  EEPROM_writeAnything (dirEEbactfec, bactfec);
}  
  
void leerDatosFechas()  {
  EEPROM_readAnything (dirEEfecsal, fecsal);
  EEPROM_readAnything (dirEEfecdia, fecdia);
  EEPROM_readAnything (dirEEfecmes, fecmes);
  EEPROM_readAnything (dirEEfecano, fecano);
  EEPROM_readAnything (dirEEfechor, fechor);
  EEPROM_readAnything (dirEEfecmin, fecmin);    
  EEPROM_readAnything (dirEEbfectipo, bfectipo);    
  EEPROM_readAnything (dirEEbactfec, bactfec);
}  

void leerDatosSemanal()  {
  EEPROM_readAnything (dirEEprgsal, prgsal);
  EEPROM_readAnything (dirEEprgval, bprgval);
  EEPROM_readAnything (dirEEprgdia, prgdia);
  EEPROM_readAnything (dirEEprghor, prghor);
  EEPROM_readAnything (dirEEprgmin, prgmin);
  EEPROM_readAnything (dirEEprgcon, prgcon);
//  EEPROM_readAnything (dirEEprgconv, prgconv);
  EEPROM_readAnything (dirEEprgcomp, prgcomp);
  EEPROM_readAnything (dirEEbprgtipo, bprgtipo);
  EEPROM_readAnything (dirEEbactsem, bactsem);
  EEPROM_readAnything (dirEEPRGsem, bPRGsem);
  }

void guardarDatosSemanal()  {
  EEPROM_writeAnything (dirEEprgsal, prgsal);
  EEPROM_writeAnything (dirEEprgval, bprgval);
  EEPROM_writeAnything (dirEEprgdia, prgdia);
  EEPROM_writeAnything (dirEEprghor, prghor);
  EEPROM_writeAnything (dirEEprgmin, prgmin);
  EEPROM_writeAnything (dirEEprgcon, prgcon);
//  EEPROM_writeAnything (dirEEprgconv, prgconv);
  EEPROM_writeAnything (dirEEprgcomp, prgcomp);
  EEPROM_writeAnything (dirEEbprgtipo, bprgtipo);
  EEPROM_writeAnything (dirEEbactsem, bactsem);
  EEPROM_writeAnything (dirEEPRGsem, bPRGsem);
  }
  
void leerEstado() {
  for (int i=0; i<8; i++)
    Mb.C8[i+32]=EEread(dirEEestado+i);    // los 8 primeros bytes de Mb.C8=bestado
  EEPROM_readAnything (dirEEcontadores, contadores);
  }

void miCheckBox (WebServer &server, char *label, boolean colorea, char *name, char *valor, boolean checked, boolean contd)
  {
    if (contd) printP1(server,(colorea?th:td));
    server.checkBox(name,valor,label,checked);
    if (contd) printP1(server,(colorea?th_f:td_f));
  }

/*********************************************************************************************/
void setupNetHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sNehtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN40];
  int param_number = 0;

  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    mailACT=0;
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN40);
      if (!repeat) break;
      param_number = atoi(name);        
        if (param_number >=0 && param_number <=5) EEmac[param_number]=strtol(value,NULL,16);       
        if (param_number >=6 && param_number <=9) EEip[param_number-6]=atoi(value);       
        if (param_number >=10 && param_number <=13) EEmask[param_number-10]=atoi(value);
        if (param_number >=14 && param_number <=17) EEgw[param_number-14]=atoi(value); 
        if (param_number >=18 && param_number <=21) EEdns[param_number-18]=atoi(value);
        if (param_number == 22) EEwebPort=atoi(value); 

        if (param_number == 23) mailACT=atoi(value); 
        if (param_number == 24) mailPER=atoi(value); 
        if (param_number == 25) for (byte i=0; i<LEN40; i++) mailSVR[i] = value[i];    
        if (param_number == 26) mailPORT = atoi(value); 
        if (param_number == 27) for (byte i=0; i<LEN40; i++) mailUSER[i] = value[i];        
        if (param_number == 28) for (byte i=0; i<LEN40; i++) mailPASS[i] = value[i];        
        if (param_number == 29) for (byte i=0; i<LEN40; i++) mailSENDER[i] = value[i];        
        if (param_number == 30) for (byte i=0; i<LEN40; i++) mailDEST[0][i] = value[i];        
        if (param_number == 31) for (byte i=0; i<LEN40; i++) mailDEST[1][i] = value[i];        
        if (param_number == 32) for (byte i=0; i<LEN40; i++) mailDEST[2][i] = value[i];        
        if (param_number == 34) for (byte i=0; i<LEN40; i++) dyndnsHost[i] = value[i];        
        if (param_number == 35) for (byte i=0; i<LEN40; i++) dyndnsUser[i] = value[i];        
        if (param_number == 36) for (byte i=0; i<LEN40; i++) dyndnsPass[i] = value[i];        

      } while (repeat);
    guardarDatosRed();  //  Guardar datos red en EEPROM
    server.httpSeeOther(PREFIX "/sNe.html"); return;
    }

  server.httpSuccess();
  writeMenu(server,3,3);
  printP6(server,brn,Form_action,netSetup,Form_post,Form_input_send,tablanormal);
  
  server.printP(tr); 
  printTDP(server, MAC, false,true);
  server.printP(td);
  for (byte i=0;i<6;i++) printcampoH(server, i, EEmac[i], 2,false);
  printTDTR(server, 1, true, true);

  // print the current IP
  server.printP(tr);  
  printTDP(server, DIRIP, false,true);
  server.printP(td);    
  for (byte i=0; i<4; i++) printcampoI(server, i+6, EEip[i], 3,false);
  printTDTR(server, 1, true, true);
  
  // print the current SUBNET
  server.printP(tr);  
  printTDP(server, SUBNET,false,true);
  server.printP(td); 
  for (byte i=0;i<4;i++) printcampoI(server, i+10, EEmask[i], 3,false);
  printTDTR(server, 1, true, true);
  
  // print the current GATEWAY
  server.printP(tr);  
  printTDP(server, GATEWAY,false,true);
  server.printP(td); 
  for (byte i=0;i<4;i++) printcampoI(server, i+14, EEgw[i], 3,false);
  printTDTR(server, 1, true, true);
  
  // print the current DNS-SERVER
  server.printP(tr);  
  printTDP(server, DNS_SERVER,false,true);
  server.printP(td); 
  for (byte i=0;i<4;i++) printcampoI(server, i+18, EEdns[i], 3,false);
  printTDTR(server, 1, true, true);

  printparIP(server,WEB_PORT, 22, EEwebPort, 5);
  lineaH(server, 2);
  server.printP(tr);  
  printTDP(server, actenviomail, false, true);
  miCheckBox(server,"",mailACT==1,"23","1",mailACT==1,true);
  server.printP(tr_f); 
  
  printparIP(server, smtpPER, 24, mailPER, 2);
  printparCP(server, smtpSVR, 25, mailSVR, 40,false);
  printparIP(server, smtpPORT, 26, mailPORT, 5);
  printparCP(server, smtpUSER, 27, mailUSER, 40,false);
  printparCP(server, smtpPASS, 28, mailPASS, 40,true);
  printparCP(server, smtpREMI, 29, mailSENDER, 40,false);
  printparCP(server, smtpDEST1, 30, mailDEST[0], 40,false);
  printparCP(server, smtpDEST2, 31, mailDEST[1], 40,false);
  printparCP(server, smtpDEST3, 32, mailDEST[2], 40,false);
  lineaH(server, 2);
 
  printP5(server,tr,td,dyndnsserv,td_f,td);
  printPiP(server,Select_name,33,barramayor);  // número de parámetro
  for (byte j=0; j<2; j++)  
    {
    printPiP(server,optionvalue,j,barraatras);
    if (dyndnsServ == j) server.printP(selected);
    server.printP(cierre);
    if (j==0) server.printP(noip);
    if (j==1) server.printP(dyndns);
    server.printP(option_f);
    }  
  printP2(server,td_f,tr);             // modo de funcionamiento
//  printparCP(server, dyndnsserv, 33, dyndnsServ, 40,false);
  printparCP(server, dyndnshost, 34, dyndnsHost, 40,false);
  printparCP(server, dyndnsuser, 35, dyndnsUser, 40,false);
  printparCP(server, dyndnspass, 36, dyndnsPass, 40,false);

  printP4(server, table_f,Form_input_send,Form_f,body_f); 
  server.printP(html_f);
}

/*********************************************************************************************/
void setupRadHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sRahtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN40];
  int param_number = 0;

  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    int mival;
    radACT=0;
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN40);
      if (!repeat) break;
      param_number = atoi(name);    
      mival = atoi(value);    
        if (param_number == 0) radACT=mival;
        if (param_number == 1) {
          mival = strtol(value,NULL, 8);    
          radNOD=mival; 
          network.begin(radCan[0], radNOD); }
        if (param_number == 2) radCan[0] = mival; 
        if (param_number == 3) reactRad=mival;
        if (param_number == 4) 
          {
            pmactRem=atol(value);
            tareason=(pmactRem>0);
           } 
      } while (repeat);
    guardarDatosRad();
    server.httpSeeOther(PREFIX "/sRa.html"); return;
    }

  server.httpSuccess();
  writeMenu(server, 5, 1);
  printP5(server,brn,Form_action,radSetup,Form_post,tablanormal);

  server.printP(trcolor1_i);
  printColspan(server,2,true);
  server.printP(general);
  printTDTR(server, 1, true, true);

  printP1(server,tr);
  printTDP(server, radActiva, false, true);
  if (radACT==1) printP1(server,th); else printP1(server,td);
  server.checkBox("0","1","",(radACT==1));
  if (radACT==1) printP1(server,th_f); else printP1(server,td_f);
  
  printP1(server,tr_f);
  printparIPOCT(server, radNodo, 1, radNOD, 5);
  printP2(server,tr,td);
  printTDP(server,radCanal,false,false);
  server.printP(td_f);
  printcampoCB(server, 2, radCan[0], 5, 126, true, true); 
  server.printP(tr_f);

//  printP2(server,tr_i,td);
//  printTDP(server,reactrad,false,false);
//  server.printP(td_f);
//  printcampoCB(server, 3, reactRad, 0, 60, true, true); 
//  server.printP(tr_f);

  printparLP(server, tmactRem, 4, pmactRem, 6);

// PENDIENTE DE PREPARAR LA GESTION DE REDES  

  printP4(server,table_f,Form_input_send,Form_f,body_f); 
  server.printP(html_f);
}

/*********************************************************************************************/
void setupRad4HTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sRa4html, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN40];
  int param_number = 0;

  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    int mival;
    radACT433=0;
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN40);
      if (!repeat) break;
      param_number = atoi(name);    
      mival = atoi(value);    
      if (param_number == 0) radACT433=mival;
      if (param_number == 1) react433=mival;
      } while (repeat);
    guardarDatos433();
    server.httpSeeOther(PREFIX "/sRa4.html"); return;
    }

  server.httpSuccess();
  writeMenu(server,7,1);
  printP5(server,brn,Form_action,rad4Setup,Form_post,tablaconf);

  server.printP(trcolor1_i);
  printColspan(server,2,true);
  server.printP(general);
  printTDTR(server, 1, true, true);

  printP1(server, tr);
  printTDP(server, radActiva, false, true);
  miCheckBox(server, "", radACT433==1,"0","1",radACT433==1,true); 

  printP5(server,tr_f,table_f,Form_input_send,Form_f,body_f); 
  server.printP(html_f);
}

void setupDevHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sdhtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    showN=0;
    sdHay=0;
    debugserial=0;
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      byte mival = atoi(value);
      if (param_number == 0) showN=mival;  
      if (param_number == 1) peract=mival;
      if (param_number == 2) debugserial=mival;
      if (param_number == 3) prectemp=mival;
      if (param_number == 4) sdHay = mival; 
      if (param_number == 5) modo = mival; 
      if (param_number == 6) pmactVar = atol(value); 
      if (param_number == 7) tipoLCD = mival; 
      } while (repeat);
      
    //// guardar
    if (numIA > nANALO) numIA = nANALO;
    if (numIA+numID > nPINES) numID = nPINES - numIA;
    if (numIA+numID+numOD > nPINES) numOD = nPINES - numIA - numID;
    numPines = numIA+numID+numOD;

    EEPROM_writeAnything (dirEEnumIA, numIA);        
    EEPROM_writeAnything (dirEEnumID, numID);
    EEPROM_writeAnything (dirEEnumOD, numOD);
    EEPROM_writeAnything (dirEEshown, showN);
    EEPROM_writeAnything (dirEEperact, peract);
    EEPROM_writeAnything (dirEEprectemp, prectemp);
    EEPROM_writeAnything (dirEEsdHay, sdHay);
    EEPROM_writeAnything (dirEEmodo, modo);
    EEPROM_writeAnything (dirEEpmactVar, pmactVar);
    EEPROM_writeAnything (dirEEtipoLCD, tipoLCD);
    if (modo==2)
      {
      for (int i=0; i<4; i++) {bfectipo[i]=0;  }
        guardarDatosFechas();
      for (int i=0; i<4; i++) {
        bactsem[i]=0;
        bacteve[i]=0;}
        guardarDatosEventos();
      for (int i=0; i<4; i++) {bprgtipo[i]=0;  }
        guardarDatosSemanal();
      }
    
    server.httpSeeOther(PREFIX "/sd.html"); 
    return;
    }
  
  server.httpSuccess();
  writeMenu(server,3,0);
  printP5(server,brn,Form_action,setupDev,Form_post,tablanormal);

  printP2(server,tr,td);             // modo de funcionamiento
  printP1(server,Modo);
  printP2(server,td_f,td);
  printPiP(server,Select_name,5,barramayor);  // número de parámetro
  for (byte j=0; j<6; j++)  
    {
    printPiP(server,optionvalue,j,barraatras);
    if (modo == j) server.printP(selected);
    server.printP(cierre);
    if (j==0) server.printP(avanzado);
    if (j==1) server.printP(basico);
    if (j==2) server.printP(local);
    if (j==3) server.printP(domotica);
    if (j==4) server.printP(riego);
    if (j==5) server.printP(climatizacion);
    server.printP(option_f);
    }  
  printP2(server,td_f,tr);             // modo de funcionamiento

  printP4(server,tr,td,sN,td_f);
  if (showN==1) printP1(server,th); else printP1(server,td);
  server.checkBox("0","1","",(showN==1));
  if (showN==1) printP1(server,th_f); else printP1(server,td_f);
  printP1(server,tr_f);
  
    
  printparIP(server, periodoact, 1, peract, 5);
  
  printP4(server,tr,td,debugserie,td_f);
  if (debugserial==1) printP1(server,th); else printP1(server,td);
  server.checkBox("2","1","",(debugserial==1));
  if (debugserial==1) printP1(server,th_f); else printP1(server,td_f);
  printP1(server, tr_f);
  
  printP1(server,tr);
  printTDP(server,precisionsondas,false,true);
  printcampoCB(server,3,prectemp,9,12,true,true); 
  printP1(server,tr_f);

  server.printP(tr);
  printTDP(server,acttarjetaSD,false,true);   
  if (sdHay==1) printP1(server,th); else printP1(server,td);
  server.checkBox("4","1","",(sdHay==1));
  if (sdHay==1) printP1(server,th_f); else printP1(server,td_f);
  printP1(server,tr_f);

  printparLP(server, tmactVar, 6, pmactVar, 5);
  
//  printP2(server,tr,td);             // tipo de LCD
//  printP1(server,ttipoLCD);
//  printP2(server,td_f,td);
//  printPiP(server,Select_name,7,barramayor);  // número de parámetro
//  for (byte j=0; j<5; j++)  
//    {
//    printPiP(server,optionvalue,j,barraatras);
//    if (tipoLCD == j) server.printP(selected);
//    server.printP(cierre);
//    if (j==0) server.printP(ninguno);
//    if (j==1) server.printP(lcdHitachiHD44780);
//    if (j==2) server.printP(lcdPowertipPC4004);
//    server.printP(option_f);
//    }  
//  printP2(server,td_f,tr);             
  
//  for (int i=0;i<5;i++)
//    {
//
//    printP1(server,tr);
//    if ((getbit8(bshowLink,i)==1)) printP1(server,th); else printP1(server,td);
//    printP3(server,b,linkusuario,b);
//    server.print(i+1); 
//    printP1(server,td_f);
//    
//    if ((getbit8(bshowLink,i)==1)) printP1(server,th); else printP1(server,td);
//    printP1(server,Form_input_text_start);      
//    server.print(8+i);                   
//    server.printP(Form_input_value);        
//    printEE (server, dirEEdescLink+(i*LEN40), LEN40);
//    printPiP(server,Max_length,LEN40-1,size_i);
//    server.print(LEN40-1);
//    printP2(server,Form_input_end,td_f);
//    char buff[3];
//    if (getbit8(bshowLink,i)==1==1) printP1(server,th); else printP1(server,td);
//    server.checkBox(itoa(20+i,buff,10),"1","",(getbit8(bshowLink,i)==1==1));
//    if (getbit8(bshowLink,i)==1==1) printP1(server,th_f); else printP1(server,td_f);
//
//
//    }
  printP5(server,table_f,Form_input_send,Form_f,body_f,html_f);
}

void ShowLogCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (slogshtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  server.httpSuccess();
  writeMenu(server,4,1);
 
  printP2(server, brn, tablanormal);
  
  printP1(server, tr);
  printP3(server, td,sistema,td_f);
  printP2(server, versistema, delsistema);
  printP1(server,tr_f);

  printP1(server, tr);
  printP3(server, td,acciones,td_f);
  printP2(server, veracciones, delacciones);
  printP1(server,tr_f);

  printP1(server, tr);
  printP3(server, td,Ent_temp,td_f);
  printP2(server, verEnt_temp, delEnt_temp);
  printP1(server,tr_f);

  printP1(server, tr);
  printP3(server, td,Ent_ana, td_f);
  printP2(server, verEnt_ana, delEnt_ana);
  printP1(server,tr_f);

  printP1(server, tr);
  printP3(server, td,eventos,td_f);
  printP2(server, vereventos, deleventos);
  printP1(server,tr_f);

  printP3(server,table_f,body_f,html_f);
}

void ShowFilesSD(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sfileshtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  server.httpSuccess();
  writeMenu(server,4,2);
  
  printP2(server,brn,tablanormal);
  if (!SD.begin(sdSEL, SPI_HALF_SPEED)) SD.initErrorHalt();
  while (file.openNext(SD.vwd(), O_READ)) 
    {
    char buffile[13];
    if (file.getFilename(buffile))
      if (!file.isDir())
        {
        printP2(server,tr,td);
        printP1(server, letraf);
        printP2(server, td_f,td);
        printP1(server,showfile01);
        server.print(buffile);
        printP1(server,showfile02);
        server.print(buffile);
        printP1(server,showfile03);
        printP2(server,td_f,td);
        server.print(file.fileSize());
        printP2(server,b,bytes);
        printP2(server,td_f,td);
        printP1(server,deletefile01);
        server.print(buffile);
        printP3(server,deletefile02,borrar,deletefile03);
        printP2(server,td_f,tr_f);
        }
    file.close();
    }

  printP3(server,table_f,body_f,html_f);
}

boolean nodovalido(int nodo)
{
  boolean result = true;
  while(nodo > 0)
  {
//    uint8_t digit = (nodo & B111);
    byte digit = (nodo & 7);
    if (digit < 1 || digit > 5)
    {
      result = false;
      break;
    }
    nodo >>= 3;
  }
}

byte actdescr=0;

void setupNodHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (snhtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  int PerAct = 15;
  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      byte mival = atoi(value);
      if (param_number == 0) nodact=strtol(value,NULL,8); 
      if (param_number == 1) canact=mival;  
      if (param_number == 2) nodnue=strtol(value,NULL,8);        
      if (param_number == 3) cannue=mival;  
      if (param_number == 4) Nodoraiz=mival;  
      if (param_number == 5) PerAct=mival;
      } while (repeat);
      
    //// enviar datos generales: Nodo, Canal, Nodo raíz, Período actualización
    headerT.type=4;          // configuración  
    headerT.from_node = radNOD;
    headerT.to_node = nodact;
    radmsgT.acc = cannue;
    radmsgT.pin = 0;
    radmsgT.valores[0] = nodnue;      
    radmsgT.valores[1] = Nodoraiz;
    radmsgT.valores[2] = PerAct;
    byte cont=nretryrem;
    boolean ok=false;
    while ((!ok) && (cont>0))
      {
      ok = network.write(headerT,&radmsgT,sizeof(radmsgT));
      if (!ok)
        {
        network.begin(radCan[0], radNOD);
        delay(delayretryrem);
        cont--;
        }
      }
    server.httpSeeOther(PREFIX "/sn.html"); return;
    }
  
  server.httpSuccess();
  writeMenu(server,5,3);
  printP4(server,brn,Form_action,setupNod,Form_post);
  printP2(server,tablanormal,trcolor1_i);
  printColspan(server,2,true);
  printP3(server,h3_i,confnodos,h3_f);
  printTDTR(server, 1, true, true);
  printparIP(server, canalact, 1, radCan[0], 3);
  printparIPOCT(server, nodoact, 0, radNOD, 5);
  printparIP(server, canalnue, 3, radCan[0], 3);
  printparIPOCT(server, nodonue, 2, radNOD, 5);
  printparIPOCT(server, nodoraiz, 4, Nodoraiz, 5);
  printparIP(server, periodoact, 5, PerAct, 3);
  printP1(server,table_f);
  printP3(server,Form_input_send,Form_f,body_f); 
  server.printP(html_f);
}

void (*resetFunc)(void)=0;

void resetHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (rsthtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  int auxpar=0;
  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      byte mival = atoi(value);
      if (param_number == 0) auxpar=1; 
      if (param_number == 1) resetfab=1; 
      } while (repeat);
      
    //// reset nodo 
    if (auxpar==1)
      if (resetfab==1)
        iniciavalores();          // reset fabrica  
      else
        resetFunc();          // reset simple  
    server.httpSeeOther(PREFIX "/rst.html"); 
    return;
    }
  
  server.httpSuccess();
  writeMenu(server,4,4);
  printP4(server,brn,Form_action,setupSys,Form_post);
  printP2(server,tablanormal,trcolor1_i);
  printColspan(server,3,true);
  printP3(server,h3_i,resetsystem,h3_f);
  printTDTR(server, 1, true, true);

  printP3(server,td,resetsystem,td_f);
  printP1(server,td);
  server.checkBox("0","1","",(auxpar==1));
  printP4(server,td_f,td,td_f,tr);

  resetfab=0;
  printP4(server,tr,td,resetfabrica,td_f);
  printP1(server,td);
  server.checkBox("1","1","",(resetfab==1));
  printP2(server,td_f,td);
  printP1(server,avisoresetfab);
  printP2(server,td_f,tr_f);

  printP1(server,table_f);
  printP3(server,Form_input_exec,Form_f,body_f); 
  server.printP(html_f);
}

void resetNodHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (rnhtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      byte mival = atoi(value);
      if (param_number == 0) 
        nodact=strtol(value,NULL,8); 
      if (param_number == 1) 
        resetfab=1; 
      } while (repeat);
      
    //// reset nodo 
    if (resetfab==1)
      headerT.type=8;          // reset fabrica  
    else
      headerT.type=7;          // configuración  
    Serial.println(headerT.type);
    headerT.from_node = radNOD;
    headerT.to_node = nodact;
    byte cont=nretryrem;
    boolean ok=false;
    while ((!ok) && (cont>0))
      {
      ok = network.write(headerT,&radmsgT,sizeof(radmsgT));
      if (!ok)
        {
        network.begin(radCan[0], radNOD);
        delay(delayretryrem);
        cont--;
        }
      }
    server.httpSeeOther(PREFIX "/rn.html"); return;
    }
  
  server.httpSuccess();
  writeMenu(server,5,9);
  printP4(server,brn,Form_action,resetNod,Form_post);
  printP2(server,tablanormal,trcolor1_i);
  printColspan(server,2,true);
  printP3(server,h3_i,resetnodo,h3_f);
  printTDTR(server, 1, true, true);
  printparIPOCT(server, nodoreset, 0, radNOD, 5);

  resetfab=0;
  printP3(server,td,resetfabrica,td_f);
  printP1(server,td);
  server.checkBox("1","1","",(resetfab==1));
  printP1(server,td_f);

  printP1(server,table_f);
  printP3(server,Form_input_exec,Form_f,body_f); 
  server.printP(html_f);
}

boolean RF24Ping (WebServer &server, uint16_t nodotest, int nretry, long timeout)
  {
    int testigo=111;
    byte cont=nretry;
    long mact;
    boolean ok=false;
    boolean alcanzado=false;
    headerT.type=9;
    headerT.from_node = radNOD;
    headerT.to_node = nodotest;
    radmsgT.valores[0] = testigo;
    while ((!ok) && (cont>0))
      {
      ok = network.write(headerT,&radmsgT,sizeof(radmsgT));
      if (ok)
        {
        if (nodotest > 5)
          {
          ok=false;
          mact=millis();
          while ((!network.available()) && (millis() < (mact+timeout)))
            network.update();
          while (network.available())
            {
            radmsgR.valores[0]=0;
            network.read(headerR,&radmsgR,sizeof(radmsgR));
            network.update();
            if ((headerR.type==9) && (headerR.from_node==nodotest) && (testigo==radmsgR.valores[0]))
              ok=true;
            else
              ok=false;
            }
          }
        }
      else
        delay(100);
      cont--;
      }
    printP1(server,b);
    return ok;
  }

void setupNod2HTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sn2html, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  unsigned int nodoexpl=1;
  int nretry=5;
  long timeout=500;
  int param_number = 0;
  byte modeexpl=0;
  if (type == WebServer::HEAD) return;
//////////////////////////////////////////////
  if (type == WebServer::POST)   {
    bool repeat;
    modeexpl=0;
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      byte mival = atoi(value);
      if (param_number == 0) nretry=mival;  
      if (param_number == 1) timeout=strtol(value,NULL,10);  
      if (param_number == 0) nodact=strtol(value,NULL,8); 
      if (param_number == 2) nodoexpl=strtol(value,NULL,8);;
      if (param_number == 3) modeexpl=mival;
      } while (repeat);
    }
  
///////////////////////////////////////////////
  server.httpSuccess();
  writeMenu(server,5,0);
  printP5(server,brn,Form_action,setupNod2,Form_post,tablanormal);

  printP1(server,tr);
  printP3(server,td,reintentos,td_f);
  printP1(server,td);
  printP1(server,Form_input_text_start);      
  server.print(0);                   
  server.printP(Form_input_value);  
  server.print(nretry);
  printPiP(server,Max_length,3,size_i);
  server.print(3);
  printP2(server,Form_input_end,td_f);
  printP1(server,tr_f);
  
  printP1(server,tr);
  printP1(server,td);
  printP1(server,ttimeout);
  printP1(server,td_f);
  printP1(server,td);
  printP1(server,Form_input_text_start);      
  server.print(1);                   
  server.printP(Form_input_value);  
  server.print(timeout);
  printPiP(server,Max_length,6,size_i);
  server.print(6);
  printP2(server,Form_input_end,td_f);
  printP1(server,tr_f);

  printP2(server,tr,td);             // modo de funcionamiento
  printP1(server,Modo);
  printP2(server,td_f,td);
  printPiP(server,Select_name,3,barramayor);  // número de parámetro
  for (byte j=0; j<2; j++)  
    {
    printPiP(server,optionvalue,j,barraatras);
    if (modeexpl == j) server.printP(selected);
    server.printP(cierre);
    if (j==0) server.printP(explorar);
    if (j==1) server.printP(probarnodo);
    server.printP(option_f);
    }  
  printP2(server,td_f,tr_f);             // modo de funcionamiento
  
  printparIPOCT(server, nodooct, 2, nodoexpl, 5);
  
  printP3(server,table_f,Form_input_exec,brn);
 
  
 /************************************/
  if (type == WebServer::POST)   
    {
    printP1(server,brn);
    printPiP(server,nodoactual,radNOD,br); 
    printPiP(server,canalactual,radCan[0],br); 
    
    if (modeexpl==0)    //  buscar nodos niveles 1 y 2
      {
      for (int i=1; i<=5; i++)
        {
        printP1(server,brn);
        printP2(server,nodo,b);
        server.print(i,OCT);
        if (RF24Ping(server, i,nretry,timeout))    // NODOS PRINCIPALES
          {
          printP4(server,b,b_i,encontrado,b_f);
          for (int j=0; j<5; j++)
            {
            uint16_t auxnodo=i+8+(8*j);
            printP1(server,brn);
            printP3(server,puntoflecha,nodo,b);
            server.print(auxnodo,OCT);
            if (RF24Ping (server,auxnodo,nretry,timeout))    // NODOS nivel 2
              printP4(server,b,b_i,encontrado,b_f);
            else
              printP2(server,b,noencontrado);
            }
          }
        else
          printP2(server,b,noencontrado);
        printP1(server,brn);
          }
        }
      else    // explorar un solo nodo
        {
          printP1(server,brn);
          for (int i=0; i<nretry; i++)
            {
            server.print(i+1); 
            printP4(server,dospuntos,b,nodo,b);
            network.update();
            if (RF24Ping(server,nodoexpl,1,timeout))    // test nodo
              printP4(server,b,b_i,encontrado,b_f);
            else
              printP2(server,b,noencontrado);
            printP1(server,brn);
            delay(100);
            }
        }
    printP4(server,brn,b_i,hecho,b_f);
    }
  printP3(server,Form_f,body_f,html_f);
  printP2(server,body_f,html_f);
}

void setupSondHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sthtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;

  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    for (byte i=0; i<3;i++) bshowTemp[i]=0;
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      int indice1 = ((param_number)/3); 
      int indice2 = ((param_number+1)/3); 
      int resto = param_number % 3;
      byte mival = atoi(value);
      if (param_number >= nDIG*3)        
        if (param_number<200)   // TEMPERATURAS 1-WIRE LOCALES
          {
          if (resto == 1)        // descripción
            for (byte i=0; i<LEN20; i++)  
              EEwrite(dirEEdescTemp+((indice2-nDIG)*LEN20)+i, value[i]);
          if (resto == 2) setbit8(bshowTemp, indice1-nDIG, mival);  // Mostrar si/no   
          }
      } while (repeat);
      
    //// guardar
    EEPROM_writeAnything (dirEEshowTemp, bshowTemp);
    server.httpSeeOther(PREFIX "/st.html"); return;
    }
  
  server.httpSuccess();
  writeMenu(server,3,1);
  printP6(server,brn,Form_action,setupSond,Form_post,Form_input_send,tablaconf);
  
  printP1(server,trcolor1_i);
  printColspan(server,2,true);
  server.printP(sondastemperatura); 
  printP1(server,td_f);
  printColspan(server,2,true);
  printP1(server,b);
  printTDTR(server, 1, true, true);
  printP2(server,trcolor1_i,td_if);
  printTDP(server, descripcion, false, true);
  printTDP(server, ver, false, true);
  printP1(server,tr_f); 

  char buf[5];
// TEMPERATURAS 1-WIRE LOCALES
  for (byte i=0; i<maxTemp; i++) 
    {
    printP1(server,tr);
    if ((getbit8(bshowTemp,i)==1)) printP1(server,th); else printP1(server,td);
    server.printP(paren_i);
    printPiP(server,letras,i,paren_f1b); 
    printP3(server,b,sondatemp,b);
    server.print(i+1); 
    printP1(server,td_f);
    
    if ((getbit8(bshowTemp,i)==1)) printP1(server,th); else printP1(server,td);
    printP1(server,Form_input_text_start);      
    server.print(((i+nDIG)*3)+1);                   
    server.printP(Form_input_value);        
//    int dir=i;
    printEE (server, dirEEdescTemp+(i*LEN20), LEN20);
    printPiP(server,Max_length,LEN20-1,size_i);
    server.print(LEN20-1);
    printP2(server,Form_input_end,td_f);
    
    if (getbit8(bshowTemp,i)==1) printP1(server,th); else printP1(server,td);
    server.checkBox(itoa(((i+nDIG)*3)+2,buf,10),"1","",(getbit8(bshowTemp,i)==1));
    if (getbit8(bshowTemp,i)==1) printP1(server,th_f); else printP1(server,td_f);

    printP1(server,tr_f);
    }
  server.printP(table_f);
  printP4(server,Form_input_send,Form_f,body_f,html_f);
}

void setupEAnagHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (seahtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  int mp=5;

  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    for (int i=0; i<nANALO;i++) setbit8(bshowpin,i+41,0);
    for (int i=0; i<2;i++) setbit8(bsumatA,i,0);
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      int indice = param_number / mp;
      int resto = param_number % mp;
      byte mival = atoi(value);
      if (resto == 0)        // descripción
        for (byte i=0; i<LEN20; i++)  
          EEwrite(dirEEdescpinA+(indice*LEN20)+i, value[i]);
      if (resto == 1) setbit8(bshowpin, indice+41, mival);   // Mostrar si/no panel   
      if (resto == 2) factorA[indice] = atof(value); 
      if (resto == 3) offsetA[indice] = atof(value); 
      if (resto == 4) setbit8(bsumatA, indice, mival); 
      } while (repeat);
      
    //// guardar
    EEPROM_writeAnything (dirEEbshowpin, bshowpin);
    EEPROM_writeAnything (dirEEfactorA, factorA);
    EEPROM_writeAnything (dirEEoffsetA, offsetA);
    EEPROM_writeAnything (dirEEsumatA, bsumatA);
    server.httpSeeOther(PREFIX "/sea.html"); return;
    }
  
  server.httpSuccess();
  writeMenu(server,3,7);
  printP6(server,brn,Form_action,setupEAnag,Form_post,Form_input_send,tablaconf);
  
  char buf[5];
    
// E/S LOCALES ANALÓGICAS
  printP1(server,trcolor1_i);
  printColspan(server,8,true);
  server.printP(eanalogicas); printP2(server,td_f,tr_f);
  
  printP2(server,trcolor1_i,td_if); 
  printTDP(server, descripcion, false, true);
  printTDP(server, ver, false, true);
  printTDP(server, factor, false, true);
  printTDP(server, offset, false, true);
  printTDP(server, sumat, false, true);
  server.printP(tr_f);  
  for (byte i=0; i<nANALO; i++)  
    {
    int mpi=mp*i;
    printP1(server, tr);
    if (getbit8(bshowpin,i+41)==1) printP1(server,th); else printP1(server,td);
    server.printP(paren_i);
    printPiP(server,letraa,i,paren_f1b); 
    printP3(server,b,entana,b);
    server.print(i+1); 
    printP1(server,b);
    if (getbit8(bshowpin,i+41)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(bshowpin,i+41)==1) printP1(server,th); else printP1(server,td);
    printP1(server,Form_input_text_start);      
    server.print(mpi);                   
    server.printP(Form_input_value);        
    int dir=i;
    printEE (server, dirEEdescpinA+(dir*LEN20), LEN20);
    printPiP(server,Max_length,LEN20-1,size_i);
    server.print(LEN20-1);
    printP1(server,Form_input_end);
    if (getbit8(bshowpin,i+41)==1) printP1(server,th_f); else printP1(server,td_f);

    if (getbit8(bshowpin,i+41)==1) printP1(server,th); else printP1(server,td);
    server.checkBox(itoa(mpi+1,buf,10),"1","",(getbit8(bshowpin,i+41)==1));
    if (getbit8(bshowpin,i+41)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(bshowpin,i+41)==1) printP1(server,th); else printP1(server,td);
    printcampoF(server,mpi+2, factorA[i], 10,5, false);
    if (getbit8(bshowpin,i+41)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(bshowpin,i+41)==1) printP1(server,th); else printP1(server,td);
    printcampoF(server,mpi+3, offsetA[i], 10,5, false);
    if (getbit8(bshowpin,i+41)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(bshowpin,i+41)==1) printP1(server,th); else printP1(server,td);
    server.checkBox(itoa(mpi+4,buf,10),"1","",(getbit8(bsumatA,i)==1));
    if (getbit8(bshowpin,i+41)==1) printP1(server,th_f); else printP1(server,td_f);
    
    printTDTR(server, 1, true, false);
    
    }
  printP5(server,table_f,Form_input_send,Form_f,body_f,html_f);
}

void setupEdigHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sedhtml, 0,'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  int mp=3;

  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    for (int i=0;i<16;i++)
      setbit8(bshowpin,i,0);
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      int indice1 = ((param_number)/mp); 
      int indice2 = ((param_number+1)/mp); 
      int resto = param_number % mp;
      byte mival = atoi(value);
      if (param_number < nDIG*mp)        
        {  
        if (resto == 0)        // descripción
          for (byte i=0; i<LEN20; i++) 
            EEwrite(dirEEdescpinD+(indice2*LEN20)+i, value[i]);
            
        if (resto == 1) tipoED[indice1]=mival;                 // Mostrar si/no panel 
        if (resto == 2) setbit8(bshowpin, indice1, mival);     // Mostrar si/no panel 
        }
      } while (repeat);
      
    //// guardar
//    EEPROM_writeAnything (dirEEshowTemp, bshowTemp);
    EEPROM_writeAnything (dirEEtipoED, tipoED);
    EEPROM_writeAnything (dirEEbshowpin, bshowpin);
    server.httpSeeOther(PREFIX "/sed.html"); return;
    }
  
  server.httpSuccess();
  writeMenu(server,3,8);
  printP6(server,brn,Form_action,setupEDig,Form_post,Form_input_send,tablaconf);

  char buf[5];
    
// E. LOCALES DIGITALES
  printP1(server,trcolor1_i);
  printColspan(server,4,true);
  if (modo==5) printP1(server,termostatos); else printP1(server,edigitales);
  printTDTR(server, 1, true, true);
  
  printP2(server,trcolor1_i,td_if); 
  printTDP(server, descripcion, false, true);
  printTDP(server, tipo, false, true);
  printTDP(server, ver, false, true);
  printP1(server,tr_f);  
  for (byte i=0; i<16; i++)  
    {
    int mpi=mp*i;
    printP1(server,tr);
    if (getbit8(bshowpin,i)==1) printP1(server,th); else printP1(server,td);
    server.printP(paren_i);
    printPiP(server,letrad,tabpin[i],paren_f1b); 
    server.printP((i<17)?entdig:saldig);
    printPiP(server,b,(i<17)?i+1:i-17+1,b);
    if (getbit8(bshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);

    if (getbit8(bshowpin,i)==1) printP1(server,th); else printP1(server,td);
    printP1(server,Form_input_text_start);      
    server.print((mpi));  // número parámetro
    server.printP(Form_input_value);        
    int dir=i;
    printEE (server, dirEEdescpinD+(dir*LEN20), LEN20);
    printPiP(server,Max_length,LEN20-1,size_i);
    server.print(LEN20-1);
    printP1(server,Form_input_end);
    if (getbit8(bshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);


    if (getbit8(bshowpin,i)==1) printP1(server,th); else printP1(server,td);
    printPiP(server,Select_name,mpi+1,barramayor);
    for (byte j=0; j<4; j++)  
      {
      printPiP(server,optionvalue,j,barraatras);
      if (tipoED[i]==j) server.printP(selected);
      if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
      if (j==0) server.printP(ONOFF);
      if (j==1) server.printP(dht11);
      if (j==2) server.printP(dht21);
      if (j==3) server.printP(dht22);
      server.printP(option_f);
      }  
    printP1(server,Select_f);
    if (getbit8(bshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);

    if (getbit8(bshowpin,i)==1) printP1(server,th); else printP1(server,td);
    server.checkBox(itoa(mpi+2,buf,10),"1","",(getbit8(bshowpin,i)==1));
    if (getbit8(bshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);
    printTDTR(server, 1, true, false);
    }
  printP5(server,table_f,Form_input_send ,Form_f,body_f,html_f);
}

void setupSdigHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (ssdhtml, 0,'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;

  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    for (int i=17;i<33;i++)
      setbit8(bshowpin,i,0);
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      int indice1 = ((param_number)/3); 
      int indice2 = ((param_number+1)/3); 
      int resto = param_number % 3;
      byte mival = atoi(value);
      if (param_number < nDIG*3)        {  // E/S LOCALES
        if (resto == 1)        // descripción
          for (byte i=0; i<LEN20; i++)  
            EEwrite(dirEEdescpinD+(indice2*LEN20)+i, value[i]);
        if (resto == 2) setbit8(bshowpin, indice1, mival);  // Mostrar si/no panel   
        }
      } while (repeat);
      
    //// guardar
//    EEPROM_writeAnything (dirEEshowTemp, bshowTemp);
    EEPROM_writeAnything (dirEEbshowpin, bshowpin);
    server.httpSeeOther(PREFIX "/ssd.html"); return;
    }
  
  server.httpSuccess();
  writeMenu(server,3,9);
  printP6(server,brn,Form_action,setupSDig,Form_post,Form_input_send,tablaconf);
  
  char buf[5];
  printP1(server,trcolor1_i);
  printColspan(server,2,true);
  printP1(server,sdigitales);
  printP2(server,td,b);
  printTDTR(server, 1, true, true);
  
  printP2(server,trcolor1_i,td_if); 
  printTDP(server, descripcion, false, true);
  printTDP(server, ver, false, true);
  server.printP(tr_f);  
// S LOCALES DIGITALES
  for (byte i=17; i<33; i++)  
    {
    if (getbit8(bshowpin,i)==1) printP1(server,th); else printP1(server,td);
    server.printP(paren_i);
    printPiP(server,letrad,tabpin[i],paren_f1b); 
    server.printP((i<17)?entdig:saldig);
    printPiP(server,b,(i<17)?i+1:i-17+1,b);
    if (getbit8(bshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);

    if (getbit8(bshowpin,i)==1) printP1(server,th); else printP1(server,td);
    printP1(server,Form_input_text_start);      
    server.print((i*3)+1);  // nÃºmero parÃ¡metro
    server.printP(Form_input_value);        
    int dir=i;
    printEE (server, dirEEdescpinD+(dir*LEN20), LEN20);
    printPiP(server,Max_length,LEN20-1,size_i);
    server.print(LEN20-1);
    printP1(server,Form_input_end);
    if (getbit8(bshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);

//    if (getbit8(bshowpin,i)==1) printP1(server,th); else printP1(server,td);
//    server.radioButton (itoa((i*3)+2,buf,10),"1","", (getbit8(bshowpin,i) ==1));  // número parámetro
//    if (getbit8(bshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);
//    if (getbit8(bshowpin,i)==1) printP1(server,th); else printP1(server,td);
//    server.radioButton (itoa((i*3)+2,buf,10),"0","", (getbit8(bshowpin,i) ==0));
//    if (getbit8(bshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);
//    
    if (getbit8(bshowpin,i)==1) printP1(server,th); else printP1(server,td);
    server.checkBox(itoa((i*3)+2,buf,10),"1","",(getbit8(bshowpin,i)==1));
    if (getbit8(bshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);
    
    printTDTR(server, 1, true, false);
    }
  printP5(server,table_f,Form_input_send,Form_f,body_f,html_f);
}

void setupRemEAnagHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (srhtml, 0,'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  int mp=7;
  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    for (int i=0; i<20; i++) setbit8(rbshowpin,i,0);
    for (int i=0; i<3;i++) setbit8(bsumatArem,i,0);
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      byte indice = param_number/mp;    // posiciones en rbshowpin 0..19
      byte resto = param_number % mp; 
      unsigned int mival = atoi(value);
      if (resto == 0) setbit8(rbshowpin, indice, mival);     
      if (resto == 1) remNodo[indice]=strtol(value,NULL,8);        
      if (resto == 2) remPin[indice] = mival; 
      if (resto == 3)        // descripción
        for (byte j=0; j<LEN20; j++)  
          EEwrite(dirEEdescAnalr+(indice*LEN20)+j, value[j]);
      if (resto == 4) factorArem[indice] = atof(value); 
      if (resto == 5) offsetArem[indice] = atof(value); 
      if (resto == 6) setbit8(bsumatArem, indice, mival);     
      } while (repeat);
    guardarDatosRem();
    server.httpSeeOther(PREFIX "/sr.html"); return;
    }
  
  server.httpSuccess();
  writeMenu(server,5,2);
  printP6(server,brn,Form_action,remEAnaSetup,Form_post,Form_input_send,tablaconf);
  
  printP1(server,trcolor1_i);
  printColspan(server,8,true);
  printP2(server,Ent_ana,td_f); 
  printTDTR(server, 1, true, false);
  
  printP2(server,trcolor1_i,td_if);  
  printTDP(server, descripcion, false, true);
  printTDP(server, ver, false, true);
  printTDP(server, nodooct, false, true);
  printTDP(server, pin, false, true);
  printTDP(server, factor, false, true);
  printTDP(server, offset, false, true);
  printTDP(server, sumat, false, true);
  server.printP(tr_f);
  
  char buf[5];
  for (byte i=0; i<20; i++)
    {
    int mpi=mp*i;
    if (getbit8(rbshowpin,i)==1) server.printP(trcolor2_i); else server.printP(tr);
    
    if (getbit8(rbshowpin,i)==1) printP1(server,th); else printP1(server,td);
    printP2(server,paren_i,letrar);
    printPiP(server,letraa,i,paren_f1b); 
    printP2(server,entana,remota); 
    printPiP(server,b,i+1,b);
    if (getbit8(rbshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i)==1) printP1(server,th); else printP1(server,td);
    printP1(server,Form_input_text_start);      
    server.print(mpi+3);                  // número de parámetro +3                 
    server.printP(Form_input_value);        
    printEE (server, dirEEdescAnalr+(i*LEN20), LEN20);
    printPiP(server,Max_length,LEN20-1,size_i);
    server.print(LEN20-1);
    printP1(server,Form_input_end);        
    if (getbit8(rbshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i)==1) printP1(server,th); else printP1(server,td);
    server.checkBox(itoa(mpi,buf,10),"1","",(getbit8(rbshowpin,i)==1));
    if (getbit8(rbshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i)==1) printP1(server,th); else printP1(server,td);
    printcampouIOCT(server, mpi+1, remNodo[i], 5,false);
    if (getbit8(rbshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i)==1) printP1(server,th); else printP1(server,td);
    printcampoCB(server, mpi+2, remPin[i], 0, 3,true,false);
    if (getbit8(rbshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i)==1) printP1(server,th); else printP1(server,td);
    printcampoF(server,mpi+4, factorArem[i], 10,8, false);
    if (getbit8(rbshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);

    if (getbit8(rbshowpin,i)==1) printP1(server,th); else printP1(server,td);
    printcampoF(server,mpi+5, offsetArem[i], 10,5, false);
    if (getbit8(rbshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);

    if (getbit8(rbshowpin,i)==1) printP1(server,th); else printP1(server,td);
    server.checkBox(itoa(mpi+6,buf,10),"1","",(getbit8(bsumatArem,i)==1));
    if (getbit8(rbshowpin,i)==1) printP1(server,th_f); else printP1(server,td_f);
    
    server.printP(tr_f);
    }
  server.printP(table_f);
  printP4(server ,Form_input_send,Form_f,body_f,html_f);
}

void setupRemEDigHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sredhtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  int mp=4;

  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    for (int i=20; i<40; i++) setbit8(rbshowpin,i,0);
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      byte indice = param_number / mp;
      byte resto = param_number % mp; 
      unsigned int mival = atoi(value);
      if (resto == 0) setbit8(rbshowpin, indice+20, mival);     
      if (resto == 1) remNodo[indice+20]=strtol(value,NULL,8);        
      if (resto == 2) remPin[indice+20] = mival; 
      if (resto == 3)        // descripción
        {
        for (byte j=0; j<LEN20; j++)  
          EEwrite(dirEEdescEDigr+(indice*LEN20)+j, value[j]);
        }
      } while (repeat);
    guardarDatosRem();
    server.httpSeeOther(PREFIX "/sred.html"); return;
    }
  server.httpSuccess();
  writeMenu(server,5,6);
  printP6(server,brn,Form_action,remEDigSetup,Form_post,Form_input_send,tablaconf);
  
  printP1(server,trcolor1_i); 
  printColspan(server,2,true);
  server.printP(Ent_dig); 
  printP2(server,td_f,td);
  printP2(server,b,td_f); 
  printColspan(server,2,true);
  printP1(server,b);
  printTDTR(server, 1, true, true);
  
  printP2(server,trcolor1_i,td_if);  
  printTDP(server, descripcion, false, true);
  printTDP(server, ver, false, true);
  printTDP(server, nodooct, false, true);
  printTDP(server, pin, false, true);
  
  server.printP(tr_f);
  char buf[5];
  for (byte i=0; i<20; i++)
    {
    int mpi=mp*i;
    int iaux=(i*mp)+20;
    server.printP(tr);
    if (getbit8(rbshowpin,i+20)==1) printP1(server,th); else printP1(server,td);
    printP2(server,paren_i,letrar);
    printPiP(server,ed,i+20,paren_f1b); 
    printP2(server,entdig,remota); 
    printPiP(server,b,i+1,b);
    if (getbit8(rbshowpin,i+20)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i+20)==1) printP1(server,th); else printP1(server,td);
    printP1(server,Form_input_text_start);      
    server.print(mpi+3);                  // número de parámetro +3                 
    server.printP(Form_input_value);        
    printEE (server, dirEEdescEDigr+(i*LEN20), LEN20);
    printPiP(server,Max_length,LEN20-1,size_i);
    server.print(LEN20-1);
    printP1(server,Form_input_end);
    if (getbit8(rbshowpin,i+20)==1) printP1(server,th_f); else printP1(server,td_f);

    if (getbit8(rbshowpin,i+20)==1) printP1(server,th); else printP1(server,td);
    server.checkBox(itoa(mpi,buf,10),"1","",(getbit8(rbshowpin,i+20)==1));
    if (getbit8(rbshowpin,i+20)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i+20)==1) printP1(server,th); else printP1(server,td);
    printcampouIOCT(server, mpi+1, remNodo[i+20], 5,false);
    if (getbit8(rbshowpin,i+20)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i+20)==1) printP1(server,th); else printP1(server,td);
    printcampoCB(server, mpi+2, remPin[i+20], 0, 1,true,false);
    if (getbit8(rbshowpin,i+20)==1) printP1(server,th_f); else printP1(server,td_f);
    server.printP(tr_f);
    }
  server.printP(table_f);
  printP4(server ,Form_input_send,Form_f,body_f,html_f);
}

void setupRemSDigHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (srsdhtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  int mp=4;

  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    for (int i=40; i<60; i++) setbit8(rbshowpin,i,0);
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      byte indice = param_number / mp;
      byte resto = param_number % mp; 
      unsigned int mival = atoi(value);
      if (resto == 0) setbit8(rbshowpin, indice+40, mival);     
      if (resto == 1) remNodo[indice+40]=strtol(value,NULL,8);        
      if (resto == 2) remPin[indice+40] = mival; 
      if (resto == 3)        // descripción
        {
        for (byte j=0; j<LEN20; j++)  
          EEwrite(dirEEdescSDigr+(indice*LEN20)+j, value[j]);
        }
      } while (repeat);
    guardarDatosRem();
    server.httpSeeOther(PREFIX "/srsd.html"); return;
    }
  
  server.httpSuccess();
  writeMenu(server,5,7);
  printP6(server,brn,Form_action,remSDigSetup,Form_post,Form_input_send,tablaconf);
  
  printP1(server,trcolor1_i); 
  printColspan(server,2,true);
  server.printP(Sal_dig); 
  printP2(server,td_f,td);
  printP2(server,b,td_f);
  printColspan(server,2,true);
  printP1(server,b);
  printTDTR(server, 1, true, true);
  
  printP2(server,trcolor1_i,td_if);  
  printTDP(server, descripcion, false, true);
  printTDP(server, ver, false, true);
  printTDP(server, nodooct, false, true);
  printTDP(server, pin, false, true);
  
  server.printP(tr_f);
  char buf[5];
  for (byte i=0; i<20; i++)
    {
    int mpi=mp*i;
    server.printP(tr);
    if (getbit8(rbshowpin,i+40)==1) printP1(server,th); else printP1(server,td);
    printP2(server,paren_i,letrar);
    printPiP(server,sd,i,paren_f1b); 
    printP2(server,saldig,remota); 
    printPiP(server,b,i+1,b);
    if (getbit8(rbshowpin,i+40)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i+40)==1) printP1(server,th); else printP1(server,td);
    printP1(server,Form_input_text_start);      
    server.print(mpi+3);                  // número de parámetro +3                 
    server.printP(Form_input_value);        
    printEE (server, dirEEdescSDigr+(i*LEN20), LEN20);
    printPiP(server,Max_length,LEN20-1,size_i);
    server.print(LEN20-1);
    printP1(server,Form_input_end);        
    if (getbit8(rbshowpin,i+40)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i+40)==1) printP1(server,th); else printP1(server,td);
    server.checkBox(itoa(mpi,buf,10),"1","",(getbit8(rbshowpin,i+40)==1));
    if (getbit8(rbshowpin,i+40)==1) printP1(server,th_f); else printP1(server,td_f);

    if (getbit8(rbshowpin,i+40)==1) printP1(server,th); else printP1(server,td);
    printcampouIOCT(server, mpi+1, remNodo[i+40], 5,false);
    if (getbit8(rbshowpin,i+40)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i+40)==1) printP1(server,th); else printP1(server,td);
    printcampoCB(server, mpi+2, remPin[i+40], 2, 3,true,false);
    if (getbit8(rbshowpin,i+40)==1) printP1(server,th_f); else printP1(server,td_f);
    server.printP(tr_f);
    }
  server.printP(table_f);
  printP4(server ,Form_input_send,Form_f,body_f,html_f);
}

void setupRemReleHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (srsrhtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  int mp=4;

  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    for (int i=60; i<120; i++) setbit8(rbshowpin,i,0);
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      byte indice = param_number / mp;
      byte resto = param_number % mp; 
      unsigned int mival = atoi(value);
      if (resto == 0) setbit8(rbshowpin, indice+60, mival);     
      if (resto == 1) remNodo[indice+60]=strtol(value,NULL,8);        
      if (resto == 2) remPin[indice+60] = mival; 
      if (resto == 3)        // descripción
        for (byte j=0; j<LEN20; j++)  
          EEwrite(dirEEdescReler+(indice*LEN20)+j, value[j]);
      } while (repeat);
    guardarDatosRem();
    server.httpSeeOther(PREFIX "/srsr.html"); return;
    }
  
  server.httpSuccess();
  writeMenu(server,5,8);
  printP6(server,brn,Form_action,remSRelSetup,Form_post,Form_input_send,tablaconf);
  
  printP1(server,trcolor1_i); 
  printColspan(server,2,true);
  printP3(server,Sal_rel,td_f,td);
  printP2(server,b,td_f);
  printColspan(server,2,true);
  printP1(server,b);
  printTDTR(server, 1, true, true);
  
  printP2(server,trcolor1_i,td_if);  
  printTDP(server, descripcion, false, true);
  printTDP(server, ver, false, true);
  printTDP(server, nodooct, false, true);
  printTDP(server, n, false, true);
  server.printP(tr_f);

  char buf[5];
  for (byte i=0; i<60; i++)
    {
    int mpi=mp*i;
    int iaux=mpi+60;
    printP1(server,tr);
    if (getbit8(rbshowpin,i+60)==1) printP1(server,th); else printP1(server,td);
    printP2(server,paren_i,letrar);
    printPiP(server,letrar,i,paren_f1b); 
    printP1(server,releremoto); 
    printPiP(server,b,i+1,b);
    if (getbit8(rbshowpin,i+60)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i+60)==1) printP1(server,th); else printP1(server,td);
    printP1(server,Form_input_text_start);      
    server.print(mpi+3);                  // número de parámetro +3                 
    server.printP(Form_input_value);        
    printEE (server, dirEEdescReler+(i*LEN20), LEN20);
    printPiP(server,Max_length,LEN20-1,size_i);
    server.print(LEN20-1);
    printP1(server,Form_input_end);
    if (getbit8(rbshowpin,i+60)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i+60)==1) printP1(server,th); else printP1(server,td);
    server.checkBox(itoa(mpi,buf,10),"1","",(getbit8(rbshowpin,i+60)==1));
    if (getbit8(rbshowpin,i+60)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i+60)==1) printP1(server,th); else printP1(server,td);
    printcampouIOCT(server, mpi+1, remNodo[i+60], 5,false);      // parámetro +1
    if (getbit8(rbshowpin,i+60)==1) printP1(server,th_f); else printP1(server,td_f);
    if (getbit8(rbshowpin,i+60)==1) printP1(server,th); else printP1(server,td);
    printcampoCB(server, mpi+2, remPin[i+60], 3, 6,true,false); // parámetro +2
    if (getbit8(rbshowpin,i+60)==1) printP1(server,th_f); else printP1(server,td_f);
    server.printP(tr_f);
    }
  server.printP(table_f);

  //print the send button
  printP4(server ,Form_input_send,Form_f,body_f,html_f);
}

void setupRem4HTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sr4html, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  byte mp=7;   // número de parámetros por fila
  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    for (int i=0; i<lTAB; i++) rbshow433[i]=0;
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      byte indice = param_number/mp;
      byte resto = param_number % mp; 
      unsigned int mival = atoi(value);
      char mivalc = value[0];
      if (resto == 0) setbit8(rbshow433, indice, mival);     
      if (resto == 1)        // descripción
        for (byte i=0; i<LEN20; i++)  {
          EEwrite(dirEEdesc433+(LEN20*indice)+i, value[i]);
          }
      if (resto == 2) tipo433[indice]=10*atol(value); 
      if (resto == 3) addr433[indice]=atol(value);     
      if (resto == 4) code433Off[indice]=atol(value);     
      if (resto == 5) unit433[indice]=mivalc;     
      if (resto == 6) tipo433[indice]=tipo433[indice]+mival;     
      } while (repeat);
    guardarDatos433();
    server.httpSeeOther(PREFIX "/sr4.html"); return;
    }
  
  server.httpSuccess();
  writeMenu(server,7,2);
  printP6(server,brn,Form_action,rem4Setup,Form_post,Form_input_send,tablaconf);
  server.printP(trcolor1_i);
  printColspan(server,8,true);
  server.printP(Devs_433);
  printTDTR(server, 1, true, true); 
  
  printP2(server,trcolor1_i,td_if);  
  printTDP(server, ver, false, true);
  printTDP(server, descripcion, false, true);
  printTDP(server, marcaclase, false, true);
  printTDP(server, taddress, false, true);
  printTDP(server, tcodeoff, false, true);
  printTDP(server, tunit, false, true);
  printTDP(server, tipo, false, true);
  
  server.printP(tr_f);
  char buf[5];
  for (byte i=0; i<max433; i++)
    {
    int mpi=mp*i;
    printP1(server,tr);
    if (getbit8(rbshow433,i)==1) server.printP(th); else server.printP(td);
    printP2(server,paren_i,letrar);
    printPiP(server,letrak,i,paren_f1b); 
    server.printP(Dev_433);
    printPiP(server,b,i+1,b);
    printP1(server,td_f); 
    if (getbit8(rbshow433,i)==1) server.printP(th_f); else server.printP(td_f);
    
    if (getbit8(rbshow433,i)==1) printP1(server,th); else printP1(server,td);
    server.checkBox(itoa((mpi),buf,10),"1","",(getbit8(rbshow433,i)==1));
    if (getbit8(rbshow433,i)==1) printP1(server,th_f); else printP1(server,td_f);


    if (getbit8(rbshow433,i)==1) server.printP(th); else server.printP(td);
    printP1(server,Form_input_text_start);     
    server.print(mpi+1);                   
    server.printP(Form_input_value);     
    printEE (server, dirEEdesc433+(i*LEN20), LEN20);
    printPiP(server,Max_length,LEN20-1,size_i);
    server.print(LEN20-1);
    printP1(server,Form_input_end);
    if (getbit8(rbshow433,i)==1) server.printP(th_f); else server.printP(td_f);

    if (getbit8(rbshow433,i)==1) server.printP(th); else server.printP(td);
    printPiP(server,Select_name,mpi+2,barramayor);
    for (byte j=0; j<5; j++)  
      {
      printPiP(server,optionvalue,j,barraatras);
      if ((tipo433[i]/10) == j) server.printP(selected);
      if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
      if (j==0) server.printP(generico);
      if (j==1) server.printP(genericonuevo);
      if (j==2) server.printP(dema);
      if (j==3) server.printP(chacon);
      if (j==4) server.printP(conuco);
      server.printP(option_f);
      }  
    printP1(server,Select_f);
    if (getbit8(rbshow433,i)==1) server.printP(th_f); else server.printP(td_f);

    if (getbit8(rbshow433,i)==1) server.printP(th); else server.printP(td);
    printcampoL(server, mpi+3, addr433[i], 12, false);   // Address/Sistema
    if (getbit8(rbshow433,i)==1) server.printP(th_f); else server.printP(td_f);
    
    if (getbit8(rbshow433,i)==1) server.printP(th); else server.printP(td);
    if (tipo433[i]/10==0)
      printcampoL(server, mpi+4, code433Off[i], 12, false);   // codeOff
    if (getbit8(rbshow433,i)==1) server.printP(th_f); else server.printP(td_f);
    
    if (getbit8(rbshow433,i)==1) server.printP(th); else server.printP(td);
    if (tipo433[i]/10!=0)
      printcampoChr(server, mpi+5, unit433[i], 1, false);    // Unit/Dev alfanumerico
    if (getbit8(rbshow433,i)==1) server.printP(th_f); else server.printP(td_f);
    
    if (getbit8(rbshow433,i)==1) server.printP(th); else server.printP(td);
    printPiP(server,Select_name,mpi+6,barramayor);
    for (byte j=0; j<3; j++)  
      {
      printPiP(server,optionvalue,j,barraatras);
      if ((tipo433[i] %10) == j) server.printP(selected);
      if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
      if (j==0) server.printP(emisor);
      if (j==1) server.printP(actuador);
      if (j==2) server.printP(emisoractuador);
      server.printP(option_f);
      }  
    printP1(server,Select_f);
    if (getbit8(rbshow433,i)==1) server.printP(th_f); else server.printP(td_f);
    printP1(server,tr_f);
    }
  server.printP(table_f);
  
  //print the send button
  printP4(server ,Form_input_send,Form_f,body_f,html_f);
}

void setupPerRemHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  logIPP (sr4html, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  byte mp=6;   // número de parámetros por fila
  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    for (int i=0; i<lTAB; i++) rbshow433[i]=0;
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      byte indice = param_number/mp;
      byte resto = param_number % mp; 
      unsigned int mival = atoi(value);
      char mivalc = value[0];
      if (resto == 0) setbit8(rbshow433, indice, mival);     
      if (resto == 1)        // descripción
        for (byte i=0; i<LEN20; i++)  {
          EEwrite(dirEEdesc433+(LEN20*indice)+i, value[i]);
          }
      if (resto == 2) tipo433[indice]=10*atol(value); 
      if (resto == 3) addr433[indice]=atol(value);     
      if (resto == 4) unit433[indice]=mivalc;     
      if (resto == 5) tipo433[indice]=tipo433[indice]+mival;     
      } while (repeat);
    guardarDatos433();
    server.httpSeeOther(PREFIX "/sr4.html"); return;
    }
  
  server.httpSuccess();
  writeMenu(server,7,2);
  printP6(server,brn,Form_action,rem4Setup,Form_post,Form_input_send,tablaconf);
  server.printP(trcolor1_i);
  printColspan(server,7,true);
  server.printP(Devs_433);
  printTDTR(server, 1, true, true); 
  
  printP2(server,trcolor1_i,td_if);  
  printTDP(server, ver, false, true);
  printTDP(server, descripcion, false, true);
  printTDP(server, marcaclase, false, true);
  printTDP(server, taddress, false, true);
  printTDP(server, tunit, false, true);
  printTDP(server, tipo, false, true);
  
  server.printP(tr_f);
  char buf[5];
  for (byte i=0; i<max433; i++)
    {
    int mpi=mp*i;
    printP1(server,tr);
    if (getbit8(rbshow433,i)==1) server.printP(th); else server.printP(td);
    printP2(server,paren_i,letrar);
    printPiP(server,letrak,i,paren_f1b); 
    server.printP(Dev_433);
    printPiP(server,b,i+1,b);
    printP1(server,td_f); 
    if (getbit8(rbshow433,i)==1) server.printP(th_f); else server.printP(td_f);
    
       if (getbit8(rbshow433,i)==1) printP1(server,th); else printP1(server,td);
    server.checkBox(itoa((mpi),buf,10),"1","",(getbit8(rbshow433,i)==1));
    if (getbit8(rbshow433,i)==1) printP1(server,th_f); else printP1(server,td_f);

    if (getbit8(rbshow433,i)==1) server.printP(th); else server.printP(td);
    printP1(server,Form_input_text_start);     
    server.print(mpi+1);                   
    server.printP(Form_input_value);     
    printEE (server, dirEEdesc433+(i*LEN20), LEN20);
    printPiP(server,Max_length,LEN20-1,size_i);
    server.print(LEN20-1);
    printP1(server,Form_input_end);
    if (getbit8(rbshow433,i)==1) server.printP(th_f); else server.printP(td_f);

    if (getbit8(rbshow433,i)==1) server.printP(th); else server.printP(td);
    printPiP(server,Select_name,mpi+2,barramayor);
    for (byte j=0; j<5; j++)  
      {
      printPiP(server,optionvalue,j,barraatras);
      if ((tipo433[i]/10) == j) server.printP(selected);
      if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
      if (j==0) server.printP(generico);
      if (j==1) server.printP(genericonuevo);
      if (j==2) server.printP(dema);
      if (j==3) server.printP(chacon);
      if (j==4) server.printP(conuco);
      server.printP(option_f);
      }  
    printP1(server,Select_f);
    if (getbit8(rbshow433,i)==1) server.printP(th_f); else server.printP(td_f);

    if (getbit8(rbshow433,i)==1) server.printP(th); else server.printP(td);
    printcampoL(server, mpi+3, addr433[i], 12, false);   // Address/Sistema
    if (getbit8(rbshow433,i)==1) server.printP(th_f); else server.printP(td_f);
    
    if (getbit8(rbshow433,i)==1) server.printP(th); else server.printP(td);
    printcampoChr(server, mpi+4, unit433[i], 1, false);    // Unit/Dev alfanumerico
    if (getbit8(rbshow433,i)==1) server.printP(th_f); else server.printP(td_f);
    
    if (getbit8(rbshow433,i)==1) server.printP(th); else server.printP(td);
    printPiP(server,Select_name,mpi+5,barramayor);
    for (byte j=0; j<3; j++)  
      {
      printPiP(server,optionvalue,j,barraatras);
      if ((tipo433[i] %10) == j) server.printP(selected);
      if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
      if (j==0) server.printP(emisor);
      if (j==1) server.printP(actuador);
      if (j==2) server.printP(emisoractuador);
      server.printP(option_f);
      }  
    printP1(server,Select_f);
    if (getbit8(rbshow433,i)==1) server.printP(th_f); else server.printP(td_f);
    printP1(server,tr_f);
    }
  server.printP(table_f);
  
  //print the send button
  printP4(server ,Form_input_send,Form_f,body_f,html_f);
}

void setupRem4codeHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sr4codehtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  if (type == WebServer::HEAD) return;
  
  server.httpSuccess();
  server.printP(htmlHead_i0);
  server.printP(htmlHead_ix);
  server.printP(htmlHead_i1);
  printP4(server,head_f,body_i,tablamenu,trcolor1_i);    // formato menú
  server.printP(htmlRefresh_i);
  printIP(server, 5,10,comillascierre);
  
  printP6(server,opc11,opc12,opc13,opc15,opc07,opc14);
  printP4(server,tr_f,table_f,tablamenu,trcolor1_i);    // formato menú
  printP3(server,opc171,opc172,opc073); 
  printP3(server,tr_f,table_f,brn);
  
  printP2(server,estaventanaseactualizacada3segundos,brn);
  printP3(server,nodejarestaventanaabierta,brn,brn);
  printP2(server,ultrecibido,b);
  server.print(pendiente433RecMostrar);
  printP1(server,brn);
  
  printP2(server,tiposugerido,b);
  if (modo433Rec==0) printP2(server,DIOmode,b);
  if (modo433Rec==1) printP2(server,DEMAmode,b);
  printP2(server,brn,brn);
  
  server.printP(tablaconf);

  server.printP(tr);
  if (modo433Rec==0) printTDP(server,taddress,false,true);
  if (modo433Rec==1) printTDP(server,taddress,false,true);
  server.printP(td);
  server.print(address433Rec);
  server.printP(td_f);
  
  printTDP(server, tunit, false, true);
  server.printP(td);
  server.print(unit433Rec);
  server.printP(td_f);
  
  printTDP(server, accion, false, true);
  server.printP(td);
  if (switchType433Rec==NewRemoteCode::on) 
    server.printP(ON);
  if (switchType433Rec==NewRemoteCode::off)
    server.printP(OFF);
  server.printP(td_f);

  server.printP(tr_f);
  server.printP(table_f);
  printP2(server,body_f,html_f);
}

void setuprTempHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (srthtml, 0,'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  byte md=4;
  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    for (byte i=0; i<maxTempr; i++)
      setbit8(rbshowpin,i+120,0);
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);  
      byte indice = param_number/md;
      byte resto = param_number % md; 
      unsigned int mival = atoi(value);
      if (resto == 0) setbit8(rbshowpin, indice+120, mival);     
      if (resto == 1) nodotempr[indice]=strtol(value,NULL,8);        
      if (resto == 2) remTpin[indice] = mival;  
      if (resto == 3)        // descripción
        {
        for (byte i=0; i<LEN20; i++)  
          EEwrite(dirEEdescTempr+(indice*LEN20)+i, value[i]);
        }
      } while (repeat);
    guardarDatosrTemp();
    server.httpSeeOther(PREFIX "/srt.html"); return;
    }
  
  server.httpSuccess();
  writeMenu(server,5,5);
  printP6(server,brn,Form_action,rTempSetup,Form_post,Form_input_send,tablaconf);
  
  printP1(server,trcolor1_i); 
  printColspan(server,2,true);
  printP2(server,Temp_rem,td_f);
  printP3(server,td,b,td_f);
  printColspan(server,2,true);
  printP1(server,b);
  printTDTR(server, 1, true, true);
  
  printP2(server,trcolor1_i,td_if);  
  printTDP(server, descripcion, false, true);
  printTDP(server, ver, false, true);
  printTDP(server, nodooct, false, true);
  printTDP(server, n, false, true);
  server.printP(tr_f);

  char buf[5];
  for (byte i=0; i<maxTempr; i++)
    {
    int mdi=md * i;
    server.printP(tr);
    if (getbit8(rbshowpin,i+120)==1) printP1(server,th); else printP1(server,td);
    printP1(server,paren_i);
    printPiP(server,rs,i,paren_f1b); 
    server.printP(sondaremota);
    server.print(i+1);    
    printP1(server,b);    
    if (getbit8(rbshowpin,i+120)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i+120)==1) printP1(server,th); else printP1(server,td);
    printP1(server,Form_input_text_start);      
    server.print(mdi+3);    // número de parámetro +3                 
    server.printP(Form_input_value);        
    printEE (server, dirEEdescTempr+(i*LEN20), LEN20);
    printPiP(server,Max_length,LEN20-1,size_i);
    server.print(LEN20-1);
    printP1(server,Form_input_end);
    if (getbit8(rbshowpin,i+120)==1) printP1(server,th_f); else printP1(server,td_f);

    if (getbit8(rbshowpin,i+120)==1) printP1(server,th); else printP1(server,td);
    server.checkBox(itoa(mdi,buf,10),"1","",(getbit8(rbshowpin,i+120)==1));
    if (getbit8(rbshowpin,i+120)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i+120)==1) printP1(server,th); else printP1(server,td);
    printcampouIOCT(server, mdi+1, nodotempr[i], 5,false);      // parámetro +1
    if (getbit8(rbshowpin,i+120)==1) printP1(server,th_f); else printP1(server,td_f);
    
    if (getbit8(rbshowpin,i+120)==1) printP1(server,th); else printP1(server,td);
    printcampoCB(server, mdi+2, remTpin[i], 0, 15,true,false);  // parámetro +2
    if (getbit8(rbshowpin,i+120)==1) printP1(server,th_f); else printP1(server,td_f);
    server.printP(tr_f);
    }
  server.printP(table_f);
  
  //print the send button
  printP4(server ,Form_input_send,Form_f,body_f,html_f);
}

void setupAvaHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sahtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  if (type == WebServer::HEAD) return;
  server.httpSuccess();
  writeMenu(server,4,0);
  printP3(server,brn,body_f,html_f);
}

void setupSegHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sshtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  char pass1temp[LEN20]="";    // 20 bytes
  char pass2temp[LEN20]="";    // 20 bytes
  int param_number = 0;
  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    usepass=0;
    opcode=0;
    do   
     {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);        
      if (param_number == 0) usepass=atoi(value);  
      if (param_number == 1) for (byte i=0; i<LEN20; i++) pass1temp[i] = value[i];      
      if (param_number == 2) for (byte i=0; i<LEN20; i++) pass2temp[i] = value[i];      
      if (param_number == 3) for (byte i=0; i<LEN20; i++) user1[i] = value[i];      
      } while (repeat);

    if (usepass)    // contraseña activa
      {
       if (strcmp(pass1temp,pass2temp) == 0)    // si coinciden ambas password se almacena
         {
          for (byte i=0; i<LEN20; i++) 
            {
              opcode=1;
              pass1[i]=pass1temp[i];
              EEwrite(dirEEuser+i, user1[i]); 
              EEwrite(dirEEpass+i, pass1temp[i]); 
            }
         }
       else
        {
         opcode=2;
         usepass=0;  // no se guarda y se desactiva contraseña
         }
      }
    else// contraseña NO activa
      if (strcmp(pass1temp,pass1)!= 0)    // si no se da la contraseña correcta, no se desactiva
        {
        opcode=3;
        usepass=1;
        }
    EEPROM_writeAnything (dirEEusepass, usepass);
    server.httpSeeOther(PREFIX "/ss.html"); return;
    }

/////////////////////
  server.httpSuccess();
  writeMenu(server,3,5);
  if (opcode!=99)
    {
    printP1(server,brn);
    if (opcode==0) printP2(server, operacionrealizada,brn);
    if (opcode==1) printP2(server, contrasenaguardada,brn);
    if (opcode==2) printP2(server, contrasenanocoincide,brn);
    if (opcode==3) printP2(server, contrasenaincorrecta,brn);
    opcode=99;
    }
  printP6(server,brn,Form_action,Form_seg,Form_post,tablaconf,tr); 
  printTDP(server, activarcontrasena, false,true);

  if (usepass==1) printP1(server,th); else printP1(server,td);
  server.checkBox("0","1","",(usepass==1));
  if (usepass==1) printP1(server,th_f); else printP1(server,td_f);
//  printparCP(server, contrasena, 1, pass1, 20,true);
  printparCP(server, usuario, 3, user1, 20,false);
  printparCP(server, contrasena, 1, "", 20,true);
  printparCP(server, confcontrasena, 2, "", 20,true);
  printP5(server,table_f,Form_input_send,Form_f,body_f,html_f);
}

void setupRelHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (srlhtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
//
  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)
    {
    bool repeat;
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
        param_number = atoi(name);   
        int mival = atoi(value);   
        if (param_number==0) tm.Wday = mival;  
        if (param_number==1) tm.Day = mival;   
        if (param_number==2) tm.Month = mival;   
        if (param_number==3) tm.Year = CalendarYrToTm(mival);  
        if (param_number==4) tm.Hour = mival;   
        if (param_number==5) tm.Minute = mival;  
        if (param_number==6) tm.Second = mival;      
      } while (repeat);
    if (RTC.write(tm)) delay(100);
    server.httpSeeOther(PREFIX "/srl.html"); return;
    }
//
  server.httpSuccess();
  writeMenu(server,3,6);
  printP6(server,brn,Form_action,Form_rel,Form_post,tablaconf,tr);
  printTDP(server,diasemana,false,true);
  printColspan(server,3,true);
  printPiP(server,Select_name,0,barramayor);
  for (byte j=0; j<7; j++)  {
    printPiP(server,optionvalue,j,barraatras);
    if (j == tm.Wday) server.printP(selected);
    server.printP(cierre); 
    printDayName(server, j); 
    server.printP(option_f);  
    }
  server.printP(Select_f);  
  printTDTR(server, 1, true, false);   

  server.printP(tr); 
  printTDP(server,diamesano,false,true);
  printcampoCB(server, 1, tm.Day, 1, 31,true,true);
  printP1(server,td);
  printPiP(server,Select_name,2,barramayor);
  for (byte j=0; j<12; j++)  {
    printPiP(server,optionvalue,j,barraatras);
    if (j == tm.Month) server.printP(selected);
    server.printP(cierre); 
    printMes(server,j);
    server.printP(option_f);  }
  printP2(server,Select_f,td_f);   
  printcampoCB(server, 3, tmYearToCalendar(tm.Year), 2010, 2025,true,true);
  printP2(server,tr_f,tr);
  printTDP(server, horaminseg,false,true);
  printcampoCB(server, 4, tm.Hour, 0, 23,true,true);
  printcampoCB(server, 5, tm.Minute, 0, 59,true,true);
  printcampoCB(server, 6, tm.Second, 0, 59,true,true);
  printP2(server,tr_f,table_f); 
  
  //print the send button
  printP4(server,Form_input_send,Form_f,body_f,html_f);
}

void printLCD(const prog_uchar *str)
  {
  char c;
   while((c = pgm_read_byte(str++)))
     lcd.write(c);
  }

void iniciavalores()
  {
  lcd.clear();  printLCD(inic);
  for (int i=0;i<16000;i++) EEwrite(i,0);  // borra EEPROM
  
  printLCD(uno);
  modo=0;
    EEPROM_writeAnything (dirEEmodo, modo);  
  for (byte i=0; i<nPRG; i++) evenTemp[i]=inact;
    EEPROM_writeAnything (dirEEevenTemp, evenTemp);  
  radNOD=0;    // nodo radio 
    EEPROM_writeAnything (dirEEradNOD, radNOD);     
  radACT=0;    // radio inactiva
    EEPROM_writeAnything (dirEEradACT, radACT);
  radFUN=0;    // función radio = master
    EEPROM_writeAnything (dirEEradFUN, radFUN);
  reactRad=0;
    EEPROM_writeAnything (dirEEreactRad, reactRad);
  react433=0;
    EEPROM_writeAnything (dirEEreact433, react433);
  radVEL=0;    // velocidad datos radio
    EEPROM_writeAnything (dirEEradVEL, radVEL);   
  sdHay=0;     // tarjeta SD no activada
    EEPROM_readAnything (dirEEsdHay, sdHay);
  EEmac[0]=170;EEmac[1]=171;EEmac[2]=172;EEmac[3]=173;EEmac[4]=174;EEmac[5]=133;
    EEPROM_writeAnything (dirEEmac, EEmac);
  EEip[0]=192;EEip[1]=168;EEip[2]=1;EEip[3]=177;
    EEPROM_writeAnything (dirEEip, EEip);
  EEmask[0]=255;EEmask[1]=255;EEmask[2]=255;EEmask[3]=0;
    EEPROM_writeAnything (dirEEmask, EEmask);
  EEgw[0]=192;EEgw[1]=168;EEgw[2]=1;EEgw[3]=1;
    EEPROM_writeAnything (dirEEgw, EEgw);
  EEdns[0]=8;EEdns[1]=8;EEdns[2]=8;EEdns[3]=8;
    EEPROM_writeAnything (dirEEdns, EEdns);
  EEwebPort=82; 
    EEPROM_writeAnything (dirEEwebport, EEwebPort);  
  for (byte i=0; i<20; i++) pass1[i]=0;    // pasword
    for (byte i=0; i<LEN20; i++) EEwrite(dirEEpass+i, pass1[i]); 
  usepass = 0;    // activar password NO
    EEPROM_writeAnything (dirEEusepass, usepass);
  for (int i=0;i<3; i++) bshowTemp[i]=0;
    EEPROM_writeAnything (dirEEshowTemp, bshowTemp);
  for (byte i=0; i<lTAB; i++) bfecval[i]=0;      // Valor a poner la salida en programación fechas
    EEPROM_writeAnything (dirEEfecval, bfecval);
  numIA = 16;     // número de entradas analÃ³gicas
    EEPROM_writeAnything (dirEEnumIA, numIA);
  numID = 16;     // número de entradas digitales
    EEPROM_writeAnything (dirEEnumID, numID);
  numOD = 16;     // número de salidas digitales
    EEPROM_writeAnything (dirEEnumOD, numOD);
  showN = 1;      // mostrar número de pin
    EEPROM_writeAnything (dirEEshown, showN);
  peract = 15;     // período de actualización automática de la página principal
    EEPROM_writeAnything (dirEEperact, peract);
  for (byte i=0; i<maxRem;i++) radCan[i]=29+i;  // Canales 29 a 37
    EEPROM_writeAnything (dirEEradCan, radCan);      
  for (int i=0; i<lTAB; i++) rbshow433[i]=0;
    EEPROM_writeAnything (dirEErshow433, rbshow433);
  radACT433=0;
    EEPROM_writeAnything (dirEEradACT433, radACT433);
  for (byte i=0; i<maxRem;i++) remPin[i]=0;     // pines de las E/S remotas
    EEPROM_writeAnything (dirEEremPin, remPin);
  for (byte i=0; i<32; i++) Mb.C8[i]=0;          // estado de E/S locales y remotas
    EEPROM_writeAnything (dirEEestado, Mb.C8);
  printLCD(dos);
  for (int i=0; i<16; i++) {    // descripciones de E digitales
    EEwrite(dirEEdescpinD+(i*LEN20), byte('E'));
    EEwrite(dirEEdescpinD+(i*LEN20)+1, byte('N'));
    EEwrite(dirEEdescpinD+(i*LEN20)+2, byte('T'));
    EEwrite(dirEEdescpinD+(i*LEN20)+3, byte(' '));
    EEwrite(dirEEdescpinD+(i*LEN20)+4, byte('D'));
    EEwrite(dirEEdescpinD+(i*LEN20)+5, byte('I'));
    EEwrite(dirEEdescpinD+(i*LEN20)+6, byte('G'));
    EEwrite(dirEEdescpinD+(i*LEN20)+7, byte(' '));
    if (i<9) 
      EEwrite(dirEEdescpinD+(i*LEN20)+8, char(i+49));
    else
      {
      EEwrite(dirEEdescpinD+(i*LEN20)+8, '1');
      EEwrite(dirEEdescpinD+(i*LEN20)+9, char(i+39));
      }
    }
  for (int i=17; i<34; i++) {    // descripciones de S digitales
    EEwrite(dirEEdescpinD+(i*LEN20), byte('S'));
    EEwrite(dirEEdescpinD+(i*LEN20)+1, byte('A'));
    EEwrite(dirEEdescpinD+(i*LEN20)+2, byte('L'));
    EEwrite(dirEEdescpinD+(i*LEN20)+3, byte(' '));
    EEwrite(dirEEdescpinD+(i*LEN20)+4, byte('D'));
    EEwrite(dirEEdescpinD+(i*LEN20)+5, byte('I'));
    EEwrite(dirEEdescpinD+(i*LEN20)+6, byte('G'));
    EEwrite(dirEEdescpinD+(i*LEN20)+7, byte(' '));
    byte auxi=i-17;
    if (auxi<9) 
      EEwrite(dirEEdescpinD+(i*LEN20)+8, char(auxi+49));
    else
      {
      EEwrite(dirEEdescpinD+(i*LEN20)+8, '1');
      EEwrite(dirEEdescpinD+(i*LEN20)+9, char(auxi+39));
      }
    }
  printLCD(tres);
  for (int i=0; i<16; i++) {    // descripciones de E/S analógicas
    EEwrite(dirEEdescpinA+(i*LEN20), byte('E'));
    EEwrite(dirEEdescpinA+(i*LEN20)+1, byte('N'));
    EEwrite(dirEEdescpinA+(i*LEN20)+2, byte('T'));
    EEwrite(dirEEdescpinA+(i*LEN20)+3, byte(' '));
    EEwrite(dirEEdescpinA+(i*LEN20)+4, byte('A'));
    EEwrite(dirEEdescpinA+(i*LEN20)+5, byte('N'));
    EEwrite(dirEEdescpinA+(i*LEN20)+6, byte('A'));
    EEwrite(dirEEdescpinA+(i*LEN20)+7, byte(' '));
    if (i<9) 
      EEwrite(dirEEdescpinA+(i*LEN20)+8, char(i+49));
    else
      {
      EEwrite(dirEEdescpinA+(i*LEN20)+8, '1');
      EEwrite(dirEEdescpinA+(i*LEN20)+9, char(i+39));
      }
    }
  for (int i=0;i<maxTempr;i++) remTpin[i]=0;
    EEPROM_writeAnything (dirEEremTpin, remTpin);
  for (byte i=0; i<nPRG; i++) fecdia[i]=0;       // Día en programación fechas
    EEPROM_writeAnything (dirEEfecdia, fecdia);
  for (byte i=0; i<120; i++) remNodo[i]=0;    // todos los nodos remotos a 0
    EEPROM_writeAnything (dirEEremNodo, remNodo);
  for (byte i=0; i<nPRG; i++) evenvalD[i]=0;  // todos los valores a 0
    EEPROM_writeAnything (dirEEevenvalD, evenvalD);
  for (byte i=0; i<max433; i++) tipo433[i]=1;  // todos los valores a 1: actuador
    EEPROM_writeAnything (dirEEtipo433, tipo433);
  for (int i=0; i<4; i++) {bprgtipo[i]=0;  }
    guardarDatosSemanal();
  for (int i=0; i<4; i++) {
    beveacttipo[i]=0;
    bevesaltipo[i]=0; }
  printLCD(cuatro);
  guardarDatosEventos();
  for (byte i=0; i<nPRG; i++) fecmin[i]=0;     // minuto
    EEPROM_writeAnything (dirEEfecmin, fecmin);     
  for (int i=0; i<nPRG; i++) evenact[i]=inact;
    EEPROM_writeAnything (dirEEevenact, evenact);
  for (byte i=0; i<nPRG; i++) evensal[i]=0;    // seÃ±al sobre la que se actÃºa en eventos
    EEPROM_writeAnything (dirEEevensal, evensal);
  for (byte i=0; i<nPRG; i++) fecsal[i]=inact;       // Salida a activar en programaciÃ³n fechas
    EEPROM_writeAnything (dirEEfecsal, fecsal);
  for (int i=0; i<4; i++) {bfectipo[i]=0;  }
    guardarDatosFechas();
  for (byte i=0; i<lTAB; i++) bevenniv[i]=0;    // nivel al que se activa la seÃ±al sobre la que se actÃºa en eventos
    EEPROM_writeAnything (dirEEevenniv, bevenniv);  
  for (int i=0; i<4; i++) {
    Mb.C8[i+21]=0;  
    EEwrite(dirEEest433+i,Mb.C8[i+21]);    }
  for (byte i=0; i<maxRem; i++) redNodo[i]=0;   // red a la que pertenece cada nodo
    EEPROM_readAnything (dirEEredNodo, redNodo);
  printLCD(cinco);
  for (int i=0; i<20; i++) {    // descripciones de sondas temperatura
    EEwrite(dirEEdescTemp+(i*LEN20), byte('S'));
    EEwrite(dirEEdescTemp+(i*LEN20)+1, byte('o'));
    EEwrite(dirEEdescTemp+(i*LEN20)+2, byte('n'));
    EEwrite(dirEEdescTemp+(i*LEN20)+3, byte('d'));
    EEwrite(dirEEdescTemp+(i*LEN20)+4, byte('a'));
    EEwrite(dirEEdescTemp+(i*LEN20)+5, byte(' '));
    if (i<9) 
      EEwrite(dirEEdescTemp+(i*LEN20)+6, char(i+49));
    else
      if (i<19)
        {
        EEwrite(dirEEdescTemp+(i*LEN20)+6, '1');
        EEwrite(dirEEdescTemp+(i*LEN20)+7, char(i+39));
        }
      else
        {
        EEwrite(dirEEdescTemp+(i*LEN20)+6, '2');
        EEwrite(dirEEdescTemp+(i*LEN20)+7, char(i+29));
        }
    }
  for (int i=0; i<8; i++) bshowpin[i]=0;
    EEPROM_writeAnything (dirEEbshowpin, bshowpin);
  for (int i=0; i<16; i++) tipoED[i]=0;
    EEPROM_writeAnything (dirEEtipoED, tipoED);

  for (byte i=0; i<24; i++) rbshowpin[i]=0;
    EEPROM_writeAnything (dirEEremshowpin, rbshowpin);  
  for (int i=0; i<4; i++) {
    bactsem[i]=0;
    bacteve[i]=0;}
    guardarDatosEventos();
  for (int i=0; i<4; i++) {bactfec[i]=0;}
    guardarDatosFechas();
  for (byte i=0; i<nPRG; i++) prgsal[i] = inact;    // todos desactivadas
    EEPROM_writeAnything (dirEEprgsal, prgsal);
  for (byte i=0; i<nPRG; i++) fecano[i]=13;      // Año en programación fechas
    EEPROM_writeAnything (dirEEfecano, fecano);
  for (byte i=0; i<lTAB; i++) bprgval[i] = 0;     // todos desactivadas a Off
    EEPROM_writeAnything (dirEEprgval, bprgval);
  for (byte i=0; i<nPRG; i++) prgdia[i] = 0;     // todos desactivadas 
    EEPROM_writeAnything (dirEEprgdia, prgdia);
  for (byte i=0; i<nPRG; i++) prghor[i] = 0;     // hora 0 
    EEPROM_writeAnything (dirEEprghor, prghor);
  for (byte i=0; i<nPRG; i++) prgmin[i] = 0;     // minuto 0 
    EEPROM_writeAnything (dirEEprgmin, prgmin);
  for (byte i=0; i<nPRG; i++) prgcon[i] = inact;    // todos desactivadas  
    EEPROM_writeAnything (dirEEprgcon, prgcon);
//  for (byte i=0; i<nPRG; i++) prgconv[i] = 0.0;   // todos a 0  
//    EEPROM_writeAnything (dirEEprgconv, prgconv);
  for (byte i=0; i<nPRG; i++) prgcomp[i] = 0;   // todos a 0  
    EEPROM_writeAnything (dirEEprgcomp, prgcomp);
  for (byte i=0; i<maxTempr; i++) nodotempr[i] = 0;   // todos a 0  
    EEPROM_writeAnything (dirEEnodotempr, nodotempr);
  for (byte i=0; i<max433; i++) addr433[i] = 0;   // todos a 0  
    EEPROM_writeAnything (dirEEaddr433, addr433);
  for (byte i=0; i<max433; i++) unit433[i] = 48;   // todos a 0  
    EEPROM_writeAnything (dirEEunit433, unit433);
  for (byte i=0; i<nPRG; i++) fechor[i]=0;       // Hora en programación fechas
    EEPROM_writeAnything (dirEEfechor, fechor);
   mailACT=0;                                    // mail desactivado
    EEPROM_writeAnything (dirEEmailACT, mailACT);  
  for (byte i=0; i<40; i++) mailSVR[i]=0;        // servidor smtp
    EEPROM_writeAnything (dirEEmailSVR, mailSVR);  
  mailPER=60;                                    // período entre mails
    EEPROM_writeAnything (dirEEmailPER, mailPER);  
  mailPORT=25;                                   // puerto servidor smtp
    EEPROM_writeAnything (dirEEmailPORT, mailPORT);  
  for (byte i=0; i<40; i++) mailUSER[i]=0;       // usuario smtp
    EEPROM_writeAnything (dirEEmailUSER, mailUSER);  
  for (byte i=0; i<40; i++) mailPASS[i]=0;       // password smtp
    EEPROM_writeAnything (dirEEmailPASS, mailPASS);  
  for (byte i=0; i<40; i++) mailSENDER[i]=0;     // remitente smtp
    EEPROM_writeAnything (dirEEmailSENDER, mailSENDER);  
  for (byte i=0; i<40; i++) mailDEST[0][i]=0;    // destinatario 1
  for (byte i=0; i<40; i++) mailDEST[1][i]=0;    // destinatario 2
  for (byte i=0; i<40; i++) mailDEST[2][i]=0;    // destinatario 3
    EEPROM_writeAnything (dirEEmailDEST, mailDEST);  
  for (byte i=0; i<nPRG; i++) evencomp[i]=0;     // comparador analógicas
    EEPROM_writeAnything (dirEEevencomp, evencomp);
  for (byte i=0; i<lTAB; i++) contadores[i]=0;   // contadores Entradas digitales 0 a 7 3
    EEPROM_writeAnything (dirEEcontadores, contadores);
  for (byte i=0; i<nPRG; i++) fecmes[i]=0;       // Mes en programación fechas
    EEPROM_writeAnything (dirEEfecmes, fecmes);
  printLCD(seis);
  for (int i=0; i<max433; i++) {    // descripciones de devices 433
    EEwrite(dirEEdesc433+(i*LEN20), byte('R'));
    EEwrite(dirEEdesc433+(i*LEN20)+1, byte('e'));
    EEwrite(dirEEdesc433+(i*LEN20)+2, byte('m'));
    EEwrite(dirEEdesc433+(i*LEN20)+3, byte('o'));
    EEwrite(dirEEdesc433+(i*LEN20)+4, byte('t'));
    EEwrite(dirEEdesc433+(i*LEN20)+5, byte('o'));
    EEwrite(dirEEdesc433+(i*LEN20)+6, byte(' '));
    if (i<9) 
      EEwrite(dirEEdesc433+(i*LEN20)+7, char(i+49));
    else
      if (i<19)
        {
        EEwrite(dirEEdesc433+(i*LEN20)+7, '1');
        EEwrite(dirEEdesc433+(i*LEN20)+8, char(i+39));
        }
      else
        if (i<29)
          {
          EEwrite(dirEEdesc433+(i*LEN20)+7, '2');
          EEwrite(dirEEdesc433+(i*LEN20)+8, char(i+29));
          }
      else
          {
          EEwrite(dirEEdesc433+(i*LEN20)+7, '3');
          EEwrite(dirEEdesc433+(i*LEN20)+8, char(i+19));
          }
    }
  for (int i=0; i<60; i++) {    // descripciones de sondas temperatura
    int pos=dirEEdescTempr+(i*LEN20);
    EEwrite(pos, byte('S'));
    EEwrite(pos+1, byte('o'));
    EEwrite(pos+2, byte('n'));
    EEwrite(pos+3, byte('d'));
    EEwrite(pos+4, byte('a'));
    EEwrite(pos+5, byte(' '));
    EEwrite(pos+6, byte('r'));
    EEwrite(pos+7, byte('e'));
    EEwrite(pos+8, byte('m'));
    EEwrite(pos+9, byte('.'));
    EEwrite(pos+10, byte(' '));
    if (i<9) 
      EEwrite(pos+11, char(i+49));
    else
      if (i<19)        {
        EEwrite(pos+11, '1');
        EEwrite(pos+12, char(i+39));        }
      else
        if (i<29)          {
          EEwrite(pos+11, '2');
          EEwrite(pos+12, char(i+29));          }
        else
          if (i<39)            {
            EEwrite(pos+11, '3');
            EEwrite(pos+12, char(i+19));            }
          else
            if (i<49)              {
              EEwrite(pos+11, '4');
              EEwrite(pos+12, char(i+9));              }
            else
              if (i<59)                {
                EEwrite(pos+11, '5');
                EEwrite(pos+12, char(i-1));                }
              else                {
                EEwrite(pos+11, '6');
                EEwrite(pos+12, char(i-11));                }
    }
  for (int i=0; i<60; i++) {    // descripciones de reles remotos
    int pos=dirEEdescReler+(i*LEN20);
    EEwrite(pos, byte('R'));
    EEwrite(pos+1, byte('e'));
    EEwrite(pos+2, byte('l'));
    EEwrite(pos+3, byte('e'));
    EEwrite(pos+4, byte(' '));
    EEwrite(pos+5, byte('r'));
    EEwrite(pos+6, byte('e'));
    EEwrite(pos+7, byte('m'));
    EEwrite(pos+8, byte('o'));
    EEwrite(pos+9, byte('t'));
    EEwrite(pos+10, byte('o'));
    EEwrite(pos+11, byte(' '));
    if (i<9) 
      EEwrite(pos+11, char(i+49));
    else
      if (i<19)        {
        EEwrite(pos+12, '1');
        EEwrite(pos+13, char(i+39));        }
      else
        if (i<29)          {
          EEwrite(pos+12, '2');
          EEwrite(pos+13, char(i+29));          }
        else
          if (i<39)            {
            EEwrite(pos+12, '3');
            EEwrite(pos+13, char(i+19));            }
          else
            if (i<49)              {
              EEwrite(pos+12, '4');
              EEwrite(pos+13, char(i+9));              }
            else
              if (i<59)                {
                EEwrite(pos+12, '5');
                EEwrite(pos+13, char(i-1));                }
              else                {
                EEwrite(pos+12, '6');
                EEwrite(pos+13, char(i-11));                }
    }
  for (int i=0; i<20; i++) {    // descripciones de analogicas remotas
    EEwrite(dirEEdescAnalr+(i*LEN20), byte('E'));
    EEwrite(dirEEdescAnalr+(i*LEN20)+1, byte('n'));
    EEwrite(dirEEdescAnalr+(i*LEN20)+2, byte('t'));
    EEwrite(dirEEdescAnalr+(i*LEN20)+3, byte('.'));
    EEwrite(dirEEdescAnalr+(i*LEN20)+4, byte('A'));
    EEwrite(dirEEdescAnalr+(i*LEN20)+5, byte('n'));
    EEwrite(dirEEdescAnalr+(i*LEN20)+6, byte('a'));
    EEwrite(dirEEdescAnalr+(i*LEN20)+7, byte('.'));
    EEwrite(dirEEdescAnalr+(i*LEN20)+8, byte('R'));
    EEwrite(dirEEdescAnalr+(i*LEN20)+9, byte('e'));
    EEwrite(dirEEdescAnalr+(i*LEN20)+10, byte('m'));
    EEwrite(dirEEdescAnalr+(i*LEN20)+11, byte('.'));
    EEwrite(dirEEdescAnalr+(i*LEN20)+12, byte(' '));
    if (i<9) 
      EEwrite(dirEEdescAnalr+(i*LEN20)+13, char(i+49));
    else
      if (i<19)
        {
        EEwrite(dirEEdescAnalr+(i*LEN20)+13, '1');
        EEwrite(dirEEdescAnalr+(i*LEN20)+14, char(i+39));
        }
      else
        {
        EEwrite(dirEEdescAnalr+(i*LEN20)+13, '2');
        EEwrite(dirEEdescAnalr+(i*LEN20)+14, char(i+29));
        }
    }
  for (int i=0; i<20; i++) {    // descripciones de entradas digitales remotas
    EEwrite(dirEEdescEDigr+(i*LEN20), byte('E'));
    EEwrite(dirEEdescEDigr+(i*LEN20)+1, byte('n'));
    EEwrite(dirEEdescEDigr+(i*LEN20)+2, byte('t'));
    EEwrite(dirEEdescEDigr+(i*LEN20)+3, byte('.'));
    EEwrite(dirEEdescEDigr+(i*LEN20)+4, byte('D'));
    EEwrite(dirEEdescEDigr+(i*LEN20)+5, byte('i'));
    EEwrite(dirEEdescEDigr+(i*LEN20)+6, byte('g'));
    EEwrite(dirEEdescEDigr+(i*LEN20)+7, byte('.'));
    EEwrite(dirEEdescEDigr+(i*LEN20)+8, byte('R'));
    EEwrite(dirEEdescEDigr+(i*LEN20)+9, byte('e'));
    EEwrite(dirEEdescEDigr+(i*LEN20)+10, byte('m'));
    EEwrite(dirEEdescEDigr+(i*LEN20)+11, byte('.'));
    EEwrite(dirEEdescEDigr+(i*LEN20)+12, byte(' '));
    if (i<9) 
      EEwrite(dirEEdescEDigr+(i*LEN20)+13, char(i+49));
    else
      if (i<19)
        {
        EEwrite(dirEEdescEDigr+(i*LEN20)+13, '1');
        EEwrite(dirEEdescEDigr+(i*LEN20)+14, char(i+39));
        }
      else
        {
        EEwrite(dirEEdescEDigr+(i*LEN20)+13, '2');
        EEwrite(dirEEdescEDigr+(i*LEN20)+14, char(i+29));
        }
    }
  for (int i=0; i<20; i++) {    // descripciones de salidas digitales remotas
    int pos=dirEEdescSDigr+(i*LEN20);
    EEwrite(pos, byte('S'));
    EEwrite(pos+1, byte('a'));
    EEwrite(pos+2, byte('l'));
    EEwrite(pos+3, byte('.'));
    EEwrite(pos+4, byte('D'));
    EEwrite(pos+5, byte('i'));
    EEwrite(pos+6, byte('g'));
    EEwrite(pos+7, byte('.'));
    EEwrite(pos+8, byte('R'));
    EEwrite(pos+9, byte('e'));
    EEwrite(pos+10, byte('m'));
    EEwrite(pos+11, byte('.'));
    EEwrite(pos+12, byte(' '));
    if (i<9) 
      EEwrite(pos+13, char(i+49));
    else
      if (i<19)
        {
        EEwrite(pos+13, '1');
        EEwrite(pos+14, char(i+39));
        }
      else
        {
        EEwrite(pos+13, '2');
        EEwrite(pos+14, char(i+29));
        }
    }
  for (byte i=0; i<nANALOr; i++) factorArem[i]=1.0;  
    EEPROM_writeAnything (dirEEfactorArem, factorArem);    
  for (byte i=0; i<nANALO; i++) factorA[i]=1.0;  
    EEPROM_writeAnything (dirEEfactorA, factorA);    
    
  for (int i=0; i<5; i++) {    // descripciones de escenas
    int pos=dirEEdescEsc+(i*LEN20);
    EEwrite(pos, byte('E'));
    EEwrite(pos+1, byte('s'));
    EEwrite(pos+2, byte('c'));
    EEwrite(pos+3, byte('e'));
    EEwrite(pos+4, byte('n'));
    EEwrite(pos+5, byte('a'));
    EEwrite(pos+6, byte(' '));
    EEwrite(pos+7, char(i+49));
    EEwrite(pos+8, 0);
    }
    
  for (int i=0; i<5; i++) {    // descripciones de programas
    int pos=dirEEdescPrg+(i*LEN20);
    EEwrite(pos, byte('P'));
    EEwrite(pos+1, byte('r'));
    EEwrite(pos+2, byte('o'));
    EEwrite(pos+3, byte('g'));
    EEwrite(pos+4, byte('r'));
    EEwrite(pos+5, byte('a'));
    EEwrite(pos+6, byte('m'));
    EEwrite(pos+7, byte('a'));
    EEwrite(pos+8, byte(' '));
    EEwrite(pos+9, char(i+49));
    EEwrite(pos+10, 0);
    }
    
//xxxxxxxxxxxxxxxxxxxx    
  for (byte i=0; i<nPRG; i++) evenvalA[i]=0.0;  // todos los valores a 0
    EEPROM_writeAnything (dirEEevenvalA, evenvalA);
  for (byte i=0; i<nPRG; i++) evenhis[i]=0;  // todos los valores a 0
    EEPROM_writeAnything (dirEEevenhis, evenhis);
  }
  
void iniciavaloresIP()
  {
  lcd.clear();  
  printLCD(iniciando);
  printLCD(blancoIP);

  EEmac[0]=170;EEmac[1]=171;EEmac[2]=172;EEmac[3]=173;EEmac[4]=174;EEmac[5]=175;
  EEip[0]=192;EEip[1]=168;EEip[2]=1;EEip[3]=177;
  EEmask[0]=255;EEmask[1]=255;EEmask[2]=255;EEmask[3]=0;
  EEgw[0]=192;EEgw[1]=168;EEgw[2]=1;EEgw[3]=1;
  EEdns[0]=8;EEdns[1]=8;EEdns[2]=8;EEdns[3]=8;
  EEwebPort=82; 

  printLCD(uno);
  guardarDatosRed();
  
  }
  
void resetcontador(WebServer &server, byte n, boolean panel)
{
  contadores[n]=0;
  EEPROM_writeAnything(dirEEcontadores+(4*n), contadores[n]);
  server.httpSeeOther(PREFIX "/"); 
}

void systemHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{ 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  URLPARAM_RESULT rc;
  char name[LEN5];
  char value[LEN20];
  boolean params_present = false;
  
////////////////////////////////////////  

  if (type == WebServer::HEAD) return;
  // check for parameters
  if (strlen(url_tail)) 
    {
    while (strlen(url_tail)) {
      rc = server.nextURLparam(&url_tail, name, LEN5, value, LEN20);
      if (rc != URLPARAM_EOS) {
        params_present=true;
        if (strcmp(name, "act") == 0)    // actuación a realizar
          {
          if (LOG==1) logIPP (act, 0, 'I', false); 
          if (strcmp (value, "r0") == 0) { resetcontador(server,0,false); return; }
          if (strcmp (value, "r1") == 0) { resetcontador(server,1,false); return; }
          if (strcmp (value, "r2") == 0) { resetcontador(server,2,false); return; }
          if (strcmp (value, "r3") == 0) { resetcontador(server,3,false); return; }
          if (strcmp (value, "r4") == 0) { resetcontador(server,4,false); return; }
          if (strcmp (value, "r5") == 0) { resetcontador(server,5,false); return; }
          if (strcmp (value, "r6") == 0) { resetcontador(server,6,false); return; }
          if (strcmp (value, "r7") == 0) { resetcontador(server,7,false); return; }
          if (strcmp (value, "rp0") == 0) { resetcontador(server,0,false); return; }
          if (strcmp (value, "rp1") == 0) { resetcontador(server,1,false); return; }
          if (strcmp (value, "rp2") == 0) { resetcontador(server,2,false); return; }
          if (strcmp (value, "rp3") == 0) { resetcontador(server,3,false); return; }
          if (strcmp (value, "rp4") == 0) { resetcontador(server,4,false); return; }
          if (strcmp (value, "rp5") == 0) { resetcontador(server,5,false); return; }
          if (strcmp (value, "rp6") == 0) { resetcontador(server,6,false); return; }
          if (strcmp (value, "rp7") == 0) { resetcontador(server,7,false); return; }
          if (strcmp (value, "rsteex") == 0)  // reinicia EEPROM
            { 
            if (LOG==1) logIPP (rsteex, 0, 'I', false); 
            server.httpSuccess();
            writeMenu(server,4,4);
            iniciavalores(); 
            printP2(server,br,operacionrealizada);
            printP2(server,body_f,html_f);  
            }
          if (strcmp (value, "shee") == 0)  // show EEPROM
            {
            if (LOG==1) logIPP (shee, 0, 'I', false); 
            server.httpSuccess();
            writeMenu(server,4,3);
            printP3(server,brn,tablamenu,trcolor1_i);
            printColspan(server,17,true);
            printP3(server,h3_i,volcadoeeprom,h3_f);
            printTDTR(server, 1, true, true); 
            printP6(server,table_f,tablaconf,tralignright,th,Bytes,th_f); 
            for (byte i=0; i<=15; i++) printPiP(server,th,i,th_f);     
            server.printP(tr_f);
            for (int i=0; i<4096; i++)   {
              server.printP(tralignright);
              printPiP(server,td,i*16,guion); 
              unsigned int auxi=i*16+15;
              server.print (auxi); 
              server.printP(td_f); 
              for (int j=0; j<16; j++)     // DECIMAL
                printPiP(server,td,EEread((i*16)+j),td_f);  
              server.printP(tr_f);
              }
            server.printP(table_f);
            printP2(server,body_f,html_f);  
            }
          if (strcmp (value, "backup") == 0)  // guarda EEPROM en fichero SD
            { 
            if (LOG==1) logIPP (backup, 0, 'I', false); 
            server.httpSuccess();
            writeMenu(server,4,5);
            printP2(server,espere,brn);
            if (backupEE()==0)
              {
              printP2(server,br,ficheroguardado);
              server.print(namefilebackup);
              }
            else
              printP2(server,br,erroralguardar);
            printP2(server,body_f,html_f);  
            }
          if (strcmp (value, "restore") == 0)  // recupera EEPROM de fichero SD
            { 
            if (LOG==1) logIPP (restore, 0, 'I', false); 
            server.httpSuccess();
            writeMenu(server,4,6);
            printP2(server,espere,brn);
            if (restoreEE()==0)
              {
              printP2(server,br,ficherorecuperado);
              server.print(namefilebackup);
              }
            else
              printP2(server,br,erroralguardar);
            printP2(server,body_f,html_f);  
            }
         }
        }
      }
    }
}

void errorHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{ server.httpFail();
  if (type == WebServer::HEAD) return;
  printP3(server,Http400,body_f,html_f);  }

void DeleFileCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (deletefile, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  server.httpSuccess();
  writeMenu(server,4,2);
  if (SD.exists(url_tail)) 
    {
    if (file.isDir())
      SD.rmdir(url_tail);
    else
      SD.remove(url_tail); 
    printP3(server,fichborrado,body_f,html_f);  
    }
  else
    {
    printP3(server,fichnoexiste,body_f,html_f);  
    }

  }  

void setupSemHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (ssehtml, 0,'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  URLPARAM_RESULT rc;
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  int mp = 18;    // número de parámetros por fila

  if (type == WebServer::HEAD) return;
  if (type == WebServer::POST)   {
    bool repeat;
    int indice,resto,mival;
    for (int j=0; j<5;j++) for (int i=0;i<4;i++) bPRGsem[j][i]=0;
    for (int i=0; i<nPRG; i++) 
      {
        prgdia[i]=0;
        setbit8(bactsem,i,0);
      }
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
      param_number = atoi(name);     
      indice = param_number / mp;
      resto = param_number % mp;
      mival = atoi(value);
      if (resto==0) {setbit8(bactsem,indice,mival);  }    // Activo: sí/no
      if (resto==1) {setbit8(bprgtipo,indice,mival);  }    // Tipo: local/remoto
      if (resto==2) {prgsal[indice] = mival;  }            // Número de salida
      if (resto==3) {setbit8(bprgval, indice, mival);  }   // valor de la salida
      if (resto==4) {setbit(&prgdia[indice], 0, mival);  }  // domingo
      if (resto==5) {setbit(&prgdia[indice], 1, mival);  }  // lunes
      if (resto==6) {setbit(&prgdia[indice], 2, mival);  }
      if (resto==7) {setbit(&prgdia[indice], 3, mival);  }
      if (resto==8) {setbit(&prgdia[indice], 4, mival);  }
      if (resto==9) {setbit(&prgdia[indice], 5, mival);  }  // viernes
      if (resto==10) {setbit(&prgdia[indice], 6, mival);  }  // sábado
      if (resto==11) {prghor[indice] = mival;  }           // Hora
      if (resto==12) {prgmin[indice] = mival;  }          // Minuto
      if (resto>=13) {setbit8(bPRGsem[resto-13],indice,mival);  }   // asociacion PRG
      } 
    while (repeat);

    guardarDatosSemanal();
    server.httpSeeOther(PREFIX "/sse.html"); return;  }
  
  server.httpSuccess();
  writeMenu(server,2,1);
  printP5(server,brn,Form_action,setupsem,Form_post,Form_input_send);
  printP2(server,tablaconf,trcolor1_i);
  printP1(server,td);
  printP2(server,tdcolspan5,asociaraprograma); 
  printP1(server,tdcolspan3);
  printP4(server,salida,td_f,tdcolspan7,dias);
  printP2(server,tdcolspan2,hora); 
  printP1(server,tr_f);
  
  printP1(server,trcolor1_i);
  printTDP(server, programa, false, true);
  printTDP(server, uno, false, true);
  printTDP(server, dos, false, true);
  printTDP(server, tres, false, true);
  printTDP(server, cuatro, false, true);
  printTDP(server, cinco, false, true);
  printTDP(server, lor, false, true);
  printTDP(server, nombre, false, true);
  printTDP(server, valor, false, true);
  printTDP(server, D, false, true);
  printTDP(server, L, false, true);
  printTDP(server, M, false, true);
  printTDP(server, X, false, true);
  printTDP(server, J, false, true);
  printTDP(server, V, false, true);
  printTDP(server, S, false, true);
  printTDP(server, hora, false, true);
  printTDP(server, minuto, false, true);
  server.printP(tr_f);
  char buf[5];
  for (int i=0; i<nPRG; i++) 
    {
    int mpi = mp*i;
    printP1(server,tr); 
    boolean colorea=((getbit8(bPRGsem[0],i)==1) || 
                     (getbit8(bPRGsem[1],i)==1) || (getbit8(bPRGsem[2],i)==1) || 
                     (getbit8(bPRGsem[3],i)==1) || (getbit8(bPRGsem[4],i)==1));
    
    printP1(server,(colorea?th:td));
    server.print(i+1);
    printP1(server,(colorea?th_f:td_f));
    
    for (byte j=0;j<5;j++)
      miCheckBox(server,"",getbit8(bPRGsem[j],i),itoa(mpi+13+j,buf,10),"1",(getbit8(bPRGsem[j],i)==1),true);
      
    if (colorea)
      {
      printP1(server,(colorea?th:td));
      if (modo==2)
        printP1(server,Local);
      else
        printcampoSiNo(server, mpi+1, getbit8(bprgtipo,i),Remota,Local, true, false);    // valor local/remota
      printP1(server,(colorea?th_f:td_f));

      printP1(server,(colorea?th:td));
      printPiP(server,Select_name,mpi+2,barramayor);
      if (getbit8(bprgtipo,i)==0)    // locales
        {
        for (byte j=17; j<33; j++)   {
          printPiP(server,optionvalue,j,barraatras);
          if (prgsal[i] == j) server.printP(selected);
          if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
          printEE (server, dirEEdescpinD+(j*LEN20), LEN20);
          server.printP(option_f);  }  
        }
      else      // remotas
        {
        for (int j=0; j<30; j++)    // Salidas digitales 433
          if (getbit8(rbshow433,j)==1)
            {
            printPiP(server,optionvalue,j,barraatras);
            if (prgsal[i] == j) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
            printEE (server, dirEEdesc433+(j*LEN20), LEN20);
            server.printP(option_f);
            }
        for (int j=91; j<111; j++)    // Salidas digitales RF24
          if (getbit8(rbshowpin,j-51)==1)
            {
            printPiP(server,optionvalue,j,barraatras);
            if (prgsal[i] == j) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
            printP2(server,saldigremota,b);
            server.print(j-91,DEC);
            server.printP(option_f);
            }
        for (int j=111; j<171; j++)    // Relés remotos RF24
          if (getbit8(rbshowpin,j-51)==1)
            {
            printPiP(server,optionvalue,j,barraatras);
            if (prgsal[i] == j) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
            printEE (server, dirEEdescReler+((j-111)*LEN20), LEN20);
            server.printP(option_f);
            }
        }
      printP3(server,Select_f,td_f,(colorea?th_f:td_f));
      
      printP1(server,(colorea?th:td));
      printcampoSiNo(server, mpi+3, getbit8(bprgval,i), on,off, true, false);    // valor on/off
      printP1(server,(colorea?th_f:td_f));
      for (byte j=0; j<7;j++) {    // días de la semana
        if (getbit(prgdia[i],j)==1) printP1(server,th); else printP1(server,td);
        printPiP(server,inputcheckbox,mpi+4+j,value1);
        if (getbit(prgdia[i],j) == 1) server.printP(checked); 
        printP2(server,cierre,br);
        if (getbit(prgdia[i],j)==1) printP1(server,th_f); else printP1(server,td_f);
       }
      if (colorea) printP1(server,th); else printP1(server,td);
      printcampoCB(server, mpi+11, prghor[i], 0, 23,false,false);  // hora
      if (colorea) printP1(server,th_f); else printP1(server,td_f);
      
      if (colorea) printP1(server,th); else printP1(server,td);
      printcampoCB(server, mpi+12, prgmin[i], 0, 59,false,false);  // minuto
      if (colorea) printP1(server,th_f); else printP1(server,td_f);
      
      }
    server.printP(tr_f);
    }
  server.printP(table_f);
  
  printP4(server,Form_input_send,Form_f,body_f,html_f);  
}

void setupFecHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (sfhtml, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  char name[LEN5];
  char value[LEN20];
  int param_number = 0;
  int mp = 9;    // número de parámetros por fila

  if (type == WebServer::HEAD) return;
  
  if (type == WebServer::POST)   {
    bool repeat;
    int indice,resto,mival;
    for (int i=0;i<4;i++) bactfec[i]=0;
    do   {
      repeat = server.readPOSTparam(name, LEN5, value, LEN20);
      if (!repeat) break;
        param_number = atoi(name);     
        indice = param_number / mp;
        resto = param_number % mp;
        mival = atoi(value);
        if (resto==0) setbit8(bactfec,indice,mival);    // programa activo si/no
        if (resto==1) setbit8(bfectipo,indice,mival);   // tipo de salida local/remota
        if (resto==2) fecsal[indice] = mival;           // Número de salida
        if (resto==3) setbit8(bfecval, indice, mival);  // valor de la salida
        if (resto==4) fecdia[indice] = mival;           // día
        if (resto==5) fecmes[indice] = mival;           // mes
        if (resto==6) fecano[indice] = mival-2000;      // año
        if (resto==7) fechor[indice] = mival;           // Hora
        if (resto==8) fecmin[indice] = mival;           // minuto
      } while (repeat);

    guardarDatosFechas();
    server.httpSeeOther(PREFIX "/sf.html"); 
      }
  
  server.httpSuccess();
  writeMenu(server,2,3);
  printP5(server,brn,Form_action,setupfec,Form_post,Form_input_send);

  printP2(server,tablaconf,trcolor1_i);
  printP3(server,tdcolspan2,tdcolspan3,salida);
  printP4(server,td_f,tdcolspan5,fechahora,td_f);
  
  printP2(server,tr_f,trcolor1_i);
  printTDP(server, programa, false, true);
  printTDP(server, Act, false, true);
  printTDP(server, lor, false, true);
  printTDP(server, nombre, false, true);
  printTDP(server, valor, false, true);
  printTDP(server, dia, false, true);
  printTDP(server, mes, false, true);
  printTDP(server, ano, false, true);
  printTDP(server, hora, false, true);
  printTDP(server, minuto, false, true);
  server.printP(tr_f);
  char buf[3];
  for (byte i=0;i<nPRG;i++)      
    {
    int mpi = mp * i;
    boolean colorea=(getbit8(bactfec,i)==1);
    printP1(server,tr);
    
    if (colorea) printP1(server,th); else printP1(server,td);
    server.print(i+1);
    if (colorea) printP1(server,th_f); else printP1(server,td_f);

    miCheckBox(server,"",colorea, itoa(mpi,buf,10), "1", getbit8(bactfec,i)==1,true);
    if (colorea)
      {  
      if (colorea) printP1(server,th); else printP1(server,td);
      if (modo==2)
        printP1(server,Local);
      else
        printcampoSiNo(server, mpi+1, getbit8(bfectipo,i), Remota,Local, true, false);    // valor local/remota
      if (colorea) printP1(server,th_f); else printP1(server,td_f);
      if (colorea) printP1(server,th); else printP1(server,td);
      printPiP(server,Select_name,mpi+2,barramayor);
      if (getbit8(bfectipo,i)==0)    // locales
        {
        for (byte j=17; j<33; j++)   {
          printPiP(server,optionvalue,j,barraatras);
          if (fecsal[i] == j) server.printP(selected);
          if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
          printEE (server, dirEEdescpinD+(j*LEN20), LEN20);
          server.printP(option_f);  }  
        }
      else      // remotas
        {
        for (int j=0; j<30; j++)    // Salidas digitales 433
          if (getbit8(rbshow433,j)==1)
            {
            printPiP(server,optionvalue,j,barraatras);
            if (fecsal[i] == j) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
            printEE (server, dirEEdesc433+(j*LEN20), LEN20);
            server.printP(option_f);
            }
        for (int j=91; j<111; j++)    // Salidas digitales RF24
          if (getbit8(rbshowpin,j-51)==1)
            {
            printPiP(server,optionvalue,j,barraatras);
            if (fecsal[i] == j) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
            printP2(server,saldigremota,b);
            server.print(j-91,DEC);
            server.printP(option_f);
            }
        for (int j=111; j<171; j++)    // Relés remotos RF24
          if (getbit8(rbshowpin,j-51)==1)
            {
            printPiP(server,optionvalue,j,barraatras);
            if (fecsal[i] == j) server.printP(selected);
            if (showN == 1) printPiP(server, mayorparen, j, parenguion); else server.printP(cierre);
            printEE (server, dirEEdescReler+((j-111)*LEN20), LEN20);
            server.printP(option_f);
            }
        }
      printP1(server,Select_f);
      if (colorea) printP1(server,th_f); else printP1(server,td_f);
  
      if (colorea) printP1(server,th); else printP1(server,td);
      printcampoSiNo(server, mpi+3, getbit8(bfecval,i), on,off, true, false);
      if (colorea) printP1(server,th_f); else printP1(server,td_f);
      if (colorea) printP1(server,th); else printP1(server,td);
      printcampoCB(server, mpi+4, fecdia[i], 1, 31,false,false);
      if (colorea) printP1(server,th_f); else printP1(server,td_f);
      if (colorea) printP1(server,th); else printP1(server,td);
      printcampoCB(server, mpi+5, fecmes[i], 1, 12,false,false);
      if (colorea) printP1(server,th_f); else printP1(server,td_f);
      if (colorea) printP1(server,th); else printP1(server,td);
      printcampoCB(server, mpi+6, fecano[i]+2000, 2013, 2050,false,false);
      if (colorea) printP1(server,th_f); else printP1(server,td_f);
      if (colorea) printP1(server,th); else printP1(server,td);
      printcampoCB(server, mpi+7, fechor[i], 0, 23,false,false);
      if (colorea) printP1(server,th_f); else printP1(server,td_f);
      if (colorea) printP1(server,th); else printP1(server,td);
      printcampoCB(server, mpi+8, fecmin[i], 0, 59,false,false);
      if (colorea) printP1(server,th_f); else printP1(server,td_f);
      }
    server.printP(tr_f);
    }
  server.printP(table_f);
  
  //print the send button
  printP4(server,Form_input_send,Form_f,body_f,html_f);  
}

void jsonCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (json, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  if (type == WebServer::POST)  {
    server.httpFail();
    return;  }
  server.httpSuccess("application/json");
  if (type == WebServer::HEAD) return;

  server.printP(llave_i);
  for (byte i = 0; i < 16; i++)    // entradas digitales locales
    {
      byte val = digitalRead(tabpin[i]);
      printPiP(server,barradi,i,dospuntos);
      server.print(val);
      server.printP(coma);   
    }
  for (byte i = 17; i < nDIG-1; i++)    // salidas digitales locales
    {
      byte val = valorpin[digitalRead(tabpin[i])];
      printPiP(server,barrads,i-17,dospuntos);
      server.print(val);
      server.printP(coma);    
  }
  for (byte i = 0; i < 16 ; i++)    // entradas analógicas locales
    {
      printPiP(server,barraa,i,dospuntos);
      server.print(float(Mb.R[i+baseAna])*factorA[i]);
      server.printP(coma);}
  for (byte i = 0; i < 20 ; i++)    // temperaturas locales
    {
      printPiP(server,barras,i,dospuntos);
      server.print(float(Mb.R[i])/100);
      server.printP(coma);}
  for (byte i = 20; i < 40 ; i++)    // entradas digitales remotas
    {
      printPiP(server,barrared,i-20,dospuntos);
      server.print(getbit8(Mb.C8,i+44));
      getbit8(Mb.C8,i+44);
      server.printP(coma);}
  for (byte i = 40; i < 60 ; i++)    // salidas digitales remotas
    {
      printPiP(server,barrarsd,i-40,dospuntos);
      server.print(getbit8(Mb.C8,i+44));
      server.printP(coma);}
  for (byte i = 60; i < maxRem ; i++)    // salidas relés remotas
    {
      printPiP(server,barrarr,i-60,dospuntos);
      server.print(getbit8(Mb.C8,i+44));
      server.printP(coma);}
  for (byte i = 0; i < 20 ; i++)    // entradas analógicas remotas
    {
      printPiP(server,barrara,i,dospuntos);
      server.print(float(Mb.R[i+baseAnar])*factorArem[i]); 
      server.printP(coma);}
  for (byte i = 0; i < maxTempr ; i++)    // entradas analógicas remotas
    {
      printPiP(server,barrars,i,dospuntos);
      server.print(float(Mb.R[i+baseTempr])/100); 
      if (i<maxTempr-1)
        server.printP(coma);}

  server.printP(llave_f);
}

void jsonnamesCmd(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (LOG==1) logIPP (json, 0, 'I', false); 
  if (!autOK(server))  {server.httpUnauthorized(); return; }
  if (type == WebServer::POST)  {
    server.httpFail();
    return;  }
  server.httpSuccess("application/json");
  if (type == WebServer::HEAD) return;

  server.printP(llave_i);
  for (byte i = 0; i < 16; i++)    // entradas digitales locales
    {
      byte val = digitalRead(tabpin[i]);
      printPiP(server,barradi,i,letran);
      printP2(server,dospuntos,barraatras);
      printEE (server, dirEEdescpinD+(i*LEN20), LEN20);
      printP2(server,barraatras,coma);          
    }
  for (byte i = 17; i < nDIG-1; i++)    // salidas digitales locales
    {
      printPiP(server,barrads,i-17,letran);
      printP2(server,dospuntos,barraatras);
      printEE (server, dirEEdescpinD+(i*LEN20), LEN20);
      printP2(server,barraatras,coma);          
    }
  for (byte i = 0; i < 16 ; i++)    // entradas analógicas locales
    {
      printPiP(server,barraa,i,dospuntos);
      server.print(float(Mb.R[i+baseAna])*factorA[i]);
      server.printP(coma);}
  for (byte i = 0; i < 20 ; i++)    // temperaturas locales
    {
      printPiP(server,barras,i,letran);
      printP2(server,dospuntos,barraatras);
      printEE (server, dirEEdescTemp+(i*LEN20), LEN20);
      printP2(server,barraatras,coma);  }        
  for (byte i = 20; i < 40 ; i++)    // entradas digitales remotas
    {
      printPiP(server,barrared,i-20,dospuntos);
      server.print(getbit8(Mb.C8,i+44));
      getbit8(Mb.C8,i+44);
      server.printP(coma);}
  for (byte i = 40; i < 60 ; i++)    // salidas digitales remotas
    {
      printPiP(server,barrarsd,i-40,dospuntos);
      server.print(getbit8(Mb.C8,i+44));
      server.printP(coma);}
  for (byte i = 60; i < maxRem ; i++)    // salidas relés remotas
    {
      printPiP(server,barrarr,i-60,dospuntos);
      server.print(getbit8(Mb.C8,i+44));
      server.printP(coma);}
  for (byte i = 0; i < 20 ; i++)    // entradas analógicas remotas
    {
      printPiP(server,barrara,i,dospuntos);
      server.print(float(Mb.R[i+baseAnar])*factorArem[i]); 
      server.printP(coma);}
  for (byte i = 0; i < maxTempr ; i++)    // entradas analógicas remotas
    {
      printPiP(server,barrars,i,dospuntos);
      server.print(float(Mb.R[i+baseTempr])/100); 
      if (i<maxTempr-1)
        server.printP(coma);}

  server.printP(llave_f);
}

int minutosentre(tmElements_t tm, tmElements_t tmI)
  {long actual;
   long inicio; 
   actual = (tmYearToCalendar(tm.Year)*525600) + (tm.Month*44640) + (tm.Hour*60) + tm.Minute;
   inicio = (tmYearToCalendar(tmI.Year)*525600) + (tmI.Month*44640) + (tmI.Hour*60) + tmI.Minute;
   return actual - inicio;
  }
 
byte inverso(byte valor)
  {if (valor==0) return(1); else return(0);}
  
void procesaEventos()
  {
  if (minutosentre(tm,tmInicio)>=mailPER)
    {
    RTC.read(tmInicio);
    for (byte i=0; i<8; i++) { bevenENABLE[0][i] = 255; bevenENABLE[1][i] = 255; }
    }
  long tiempo=millis();
  for (byte i=0; i<nPRG; i++) 
    if (((actPrg[0]==1) && (getbit8(bPRGeve[0],i)==1)) ||
        ((actPrg[1]==1) && (getbit8(bPRGeve[1],i)==1)) || 
        ((actPrg[2]==1) && (getbit8(bPRGeve[2],i)==1)) || 
        ((actPrg[3]==1) && (getbit8(bPRGeve[3],i)==1)) || 
        ((actPrg[4]==1) && (getbit8(bPRGeve[4],i)==1)) )
    {
     if (getbit8(beveacttipo,i)==0)      // activadora local
       {
       if (evenact[i] <=32)      // activadora digital local (0-15 ED, 16-32 SD)
         {
          if (digitalRead(tabpin[evenact[i]]) == evenvalD[i])  // si valor pin = valor en evento
            {
             if (getbit8(bevesaltipo,i)==0)   // señal de salida local
               {
               if (evensal[i] < despMail)            // señal local
                 {
                 pinVAL(tabpin[evensal[i]],getbit8(bevenniv,i));    // señal local
                 if (LOG==1) logEve("",tabpin[evensal[i]],'L',(getbit8(bevenniv,i)==1));
                 }
               else
                 if (getbit8(bevenENABLE[0],i) == 1) 
                   {    // si correo activado
                   sendEmail(i, 0, 0);
                   setbit8(bevenENABLE[0],i,0);  
                   if (LOG==1) logEve("",tabpin[evensal[i]],'M',(getbit8(bevenniv,i)==1));
                   }  
               }
             else
               {
               if (evensal[i] < 30) 
                 { 
                 pinVal433Cond(evensal[i],bevenniv[i]);    // señal remota 433
                 if (LOG==1) logEve("",tabpin[evensal[i]],'4',(getbit8(bevenniv,i)==1));
                 }
               else 
                 {
                 pinValrCond(evensal[i]-51,bevenniv[i]);    // señal remota 2.4
                 if (LOG==1) logEve("",tabpin[evensal[i]],'R',(getbit8(bevenniv,i)==1));
                 }
               }
            }                
         }
       else                      // activadora analógica local (33-48 EA, 171-250 TEMP)
         {
          float tact;
          if ((evenact[i] <=48) || (evenact[i] <=254))   // activadora entrada analógica local (33-48 EA) o precio kwh
            tact = float(Mb.R[evenact[i]+47])/100;   // Ent. analógica local en modbus
          else
            tact = float(Mb.R[evenact[i]-171])/100;   // temperatura local en modbus
          float tcomp = float(evenvalA[i])/100;
          float thist = float(evenhis[i])/100;
//**************************************************
          if (evencomp[i]==0)    // comparador mayor o igual
            {
            if (tact >= tcomp)     // si valor pin >= valor en evento
              {
               if (getbit8(bevesaltipo,i)==0)   // señal de salida local
                 if (evensal[i] < despMail)    // si la salida es DIGITAL
                   {
                   pinVAL(tabpin[evensal[i]],getbit8(bevenniv,i));    // señal local
                   if (LOG==1) logEve("",tabpin[evensal[i]],'L',(getbit8(bevenniv,i)==1));
                   }
                 else
                   if (getbit8(bevenENABLE[0],i) == 1)
                     {
                     sendEmail(i, 0, 0);
                     setbit8(bevenENABLE[0],i,0);  
                     if (LOG==1) logEve("",tabpin[evensal[i]],'M',(getbit8(bevenniv,i)==1));
                     }  
               else                          // señal de salida remota
                 if (evensal[i] < 30)
                   {
                   pinVal433Cond(evensal[i],bevenniv[i]);    // señal remota 433
                   if (LOG==1) logEve("",tabpin[evensal[i]],'4',(getbit8(bevenniv,i)==1));
                   }
                 else 
                   {
                   pinValrCond(evensal[i]-51,bevenniv[i]);    // señal remota 2.4
                   if (LOG==1) logEve("",tabpin[evensal[i]],'R',(getbit8(bevenniv,i)==1));
                   }
              }
            else        // baja la temperatura hay que ver si baja de (tcomp-thist)
              {
              if (tact < tcomp-thist)    // volver la salida a us estado de reposo
                {
                if (evensal[i] < despMail)   // si la salida es DIGITAL
                  {
                  if (getbit8(bevesaltipo,i)==0)   // señal de salida local
                    {
                    pinVAL(tabpin[evensal[i]],reversepin[getbit8(bevenniv,i)]);    // señal local
                    if (LOG==1) logEve("",tabpin[evensal[i]],'L',(reversepin[bevenniv[i]]==1));
                    }
                  else
                    if (evensal[i] < 30)  
                     {
                      pinVal433Cond(evensal[i],reversepin[bevenniv[i]]);    // señal remota 433
                      if (LOG==1) logEve("",tabpin[evensal[i]],'4',(reversepin[bevenniv[i]]==1));
                     }
                    else 
                      {
                      pinValrCond(evensal[i]-51,reversepin[bevenniv[i]]);    // señal remota 2.4
                      if (LOG==1) logEve("",tabpin[evensal[i]],'R',(reversepin[bevenniv[i]]==1));
                      }
                   }
                else                   // si es enviar correo
                  {    
                   // de momento no se hace nada, no se envía correo para avisar que ya ha vuelto a normal
                  }
                }
              }
            }
          else      // comparador menor o igual
             {
             if (tact <= tcomp)     // si valor pin <= valor en evento
              {
               if (getbit8(bevesaltipo,i)==0)   // señal de salida local
                 {
                 if (evensal[i] < despMail)   // si la salida es DIGITAL
                   {
                   pinVAL(tabpin[evensal[i]],getbit8(bevenniv,i));    // señal local
                   if (LOG==1) logEve("",tabpin[evensal[i]],'L',(getbit8(bevenniv,i)==1));
                   }
                 else
                   {
                   if (getbit8(bevenENABLE[0],i) == 1) 
                     {
                     sendEmail(i, 0, 0);
                     setbit8(bevenENABLE[0],i,0);  
                     if (LOG==1) logEve("",tabpin[evensal[i]],'M',(getbit8(bevenniv,i)==1));
                     }
                   }
                 }
               else
                 {
                 if (evensal[i] < 30)  
                   {
                   pinVal433Cond(evensal[i],bevenniv[i]);    // señal remota 433
                   if (LOG==1) logEve("",tabpin[evensal[i]],'4',getbit8(bevenniv,i)==1);
                   }
                 else 
                   {
                   pinValrCond(evensal[i]-51,bevenniv[i]);    // señal remota 2.4
                   if (LOG==1) logEve("",tabpin[evensal[i]],'R',(getbit8(bevenniv,i)==1));
                   }
                 }
              }
            else    //  valor pin > valor en evento+histeresis
              {
              if (tact > tcomp+thist)
                {
                 if (evensal[i] < despMail)   // si la salida es DIGITAL
                   {
                   if (getbit8(bevesaltipo,i)==0)   // señal de salida local
                     {
                     pinVAL(tabpin[evensal[i]],reversepin[getbit8(bevenniv,i)]);    // señal local
                     if (LOG==1) logEve("",tabpin[evensal[i]],'L',(reversepin[getbit8(bevenniv,i)]==1));
                     }
                   else    // señal de salida remota
                     if (evensal[i] < 30)  
                       {
                       pinVal433Cond(evensal[i],reversepin[bevenniv[i]]);    // señal remota 433
                       if (LOG==1) logEve("",tabpin[evensal[i]],'4',(reversepin[getbit8(bevenniv,i)]==1));
                       }
                     else 
                       {
                       pinValrCond(evensal[i]-51,reversepin[bevenniv[i]]);    // señal remota 2.4
                       if (LOG==1) logEve("",tabpin[evensal[i]],'R',(reversepin[getbit8(bevenniv,i)]==1));
                       }
                     }
                  else                   // si es enviar correo
                    {    
                     // de momento no se hace nada, no se envía correo para avisar que ya ha vuelto a normal
                    }
                  }
                }
              }
             }
//**************************************************
         }
     else            // activadora remota
       {
       if ((evenact[i] <=29) || (evenact[i]>=71) && (evenact[i]<=170))    // activadora remota digital
         {
           boolean coincide=false;
           if (evenact[i] <=29)
             coincide=(getbit8(Mb.C8,evenact[i]+168) == bevenniv[i]);
           else
             coincide = (getbit8(Mb.C8,evenact[i]-7) == bevenniv[i]);
            if (coincide) 
              {
               if (getbit8(bevesaltipo,i)==0)   // señal de salida local
                 {
                 pinVAL(tabpin[evensal[i]],getbit8(bevenniv,i));    // señal local
                 if (LOG==1) logEve("",tabpin[evensal[i]],'L',(getbit8(bevenniv,i)==1));
                 }
               else
                 if (evensal[i] < 30)  
                   {
                   pinVal433Cond(evensal[i],bevenniv[i]);    // señal remota 433
                   if (LOG==1) logEve("",tabpin[evensal[i]],'4',(getbit8(bevenniv,i)==1));
                   }
                 else 
                   if (evensal[i] < despMail)
                     {
                     pinValrCond(evensal[i]-51,bevenniv[i]);    // señal remota 2.4
                     if (LOG==1) logEve("",tabpin[evensal[i]],'R',(getbit8(bevenniv,i)==1));
                     }
                   else
                     if (getbit8(bevenENABLE[0],i) == 1) 
                       {
                       sendEmail(i, 0, 0);
                       setbit8(bevenENABLE[0],i,0);  
                       if (LOG==1) logEve("",tabpin[evensal[i]],'M',(getbit8(bevenniv,i)==1));
                       }
              }
         }
       else        // activadora remota analógica
         {
          float tact;
          if (evenact[i] <=70)   // activadora entrada analógica local (33-48 EA)   
            tact = float(Mb.R[evenact[i]+45])*factorArem[evenact[i]-51];   // Ent. analógica remota en modbus
          else
            tact = float(Mb.R[evenact[i]-171])/100;   // temperatura remota en modbus
          float tcomp = float(evenvalA[i])/100;
          float thist = float(evenhis[i])/100;
          if (evencomp[i]==0)    // comparador mayor o igual
            {
            if (tact >= tcomp)     // si valor pin >= valor en evento
              {
              if (evensal[i] < despMail)   // si la salida es DIGITAL
               if (getbit8(bevesaltipo,i)==0)   // señal de salida local
                 {
                 pinVAL(tabpin[evensal[i]],getbit8(bevenniv,i));    // señal local
                 if (LOG==1) logEve("",tabpin[evensal[i]],'L',(getbit8(bevenniv,i)==1));
                 }
               else                          // señal de salida remota
                 if (evensal[i] < 30)        
                   {
                   pinVal433Cond(evensal[i],bevenniv[i]);    // señal remota 433
                   if (LOG==1) logEve("",tabpin[evensal[i]],'4',(getbit8(bevenniv,i)==1));
                   }
                 else 
                   {
                   pinValrCond(evensal[i]-51,bevenniv[i]);    // señal remota 2.4
                   if (LOG==1) logEve("",tabpin[evensal[i]],'R',(getbit8(bevenniv,i)==1));
                   }
              else                   // si es enviar correo
                {    
                if (getbit8(bevenENABLE[0],i) == 1) 
                  {
                  sendEmail(i, 0, 0);
                  setbit8(bevenENABLE[0],i,0); 
                  if (LOG==1) logEve("",tabpin[evensal[i]],'M',(getbit8(bevenniv,i)==1));
                  }                  
                 }
              }
            else        // baja la temperatura hay que ver si baja de (tcomp-thist)
              {
              if (tact < tcomp-thist)    // volver la salida a us estado de reposo
                {
                if (evensal[i] < despMail)   // si la salida es DIGITAL
                  {
                  if (getbit8(bevesaltipo,i)==0)   // señal de salida local
                    {
                    pinVAL(tabpin[evensal[i]],reversepin[getbit8(bevenniv,i)]);    // señal local
                    if (LOG==1) logEve("",tabpin[evensal[i]],'L',(reversepin[getbit8(bevenniv,i)]==1));
                    }
                  else
                    if (evensal[i] < 30)  
                      {
                      pinVal433Cond(evensal[i],reversepin[bevenniv[i]]);    // señal remota 433
                      if (LOG==1) logEve("",tabpin[evensal[i]],'4',(reversepin[getbit8(bevenniv,i)]==1));
                      }
                    else 
                      {
                      pinValrCond(evensal[i]-51,reversepin[bevenniv[i]]);    // señal remota 2.4
                      if (LOG==1) logEve("",tabpin[evensal[i]],'R',(reversepin[getbit8(bevenniv,i)]==1));
                      }
                   }
                else                   // si es enviar correo
                  {    
                   // de momento no se hace nada, no se envía correo para avisar que ya ha vuelto a normal
                  }
                }
              }
            }
          else      // comparador menor o igual
            {
            if (tact <= tcomp)     // si valor pin <= valor en evento
              {
              if (evensal[i] < despMail)   // si la salida es DIGITAL
                if (getbit8(bevesaltipo,i)==0)   // señal de salida local
                  {
                  pinVAL(tabpin[evensal[i]],getbit8(bevenniv,i));    // señal local
                  if (LOG==1) logEve("",tabpin[evensal[i]],'L',(getbit8(bevenniv,i)==1));
                  }
                else
                  if (evensal[i] < 30)  
                    {
                    pinVal433Cond(evensal[i],evenvalD[i]);    // señal remota 433
                    if (LOG==1) logEve("",tabpin[evensal[i]],'4',(getbit8(bevenniv,i)==1));
                    }
                  else 
                    {
                    pinValrCond(evensal[i]-51,bevenniv[i]);    // señal remota 2.4
                    if (LOG==1) logEve("",tabpin[evensal[i]],'R',(getbit8(bevenniv,i)==1));
                    }
              else                   // si es enviar correo
                {    
                if (getbit8(bevenENABLE[0],i) == 1) 
                  {
                  sendEmail(i, 0, 0);
                  setbit8(bevenENABLE[0],i,0);  
                  if (LOG==1) logEve("",tabpin[evensal[i]],'M',(getbit8(bevenniv,i)==1));
                  }
                 }
              }
            else
              {
              if (tact > tcomp+thist)
                {
                if (evensal[i] < despMail)   // si la salida es DIGITAL
                 if (getbit8(bevesaltipo,i)==0)   // señal de salida local
                   {
                   pinVAL(tabpin[evensal[i]],reversepin[getbit8(bevenniv,i)]);    // señal local
                   if (LOG==1) logEve("",tabpin[evensal[i]],'L',(reversepin[getbit8(bevenniv,i)]==1));
                   }
                 else
                   if (evensal[i] < 30)  
                     {
                     pinVal433Cond(evensal[i],reversepin[bevenniv[i]]);    // señal remota 433
                     if (LOG==1) logEve("",tabpin[evensal[i]],'4',(reversepin[getbit8(bevenniv,i)]==1));
                     }
                   else 
                     {
                     pinValrCond(evensal[i]-51,reversepin[bevenniv[i]]);    // señal remota 2.4
                     if (LOG==1) logEve("",tabpin[evensal[i]],'R',(reversepin[getbit8(bevenniv,i)]==1));
                     }
                else                   // si es enviar correo
                  {    
                   // de momento no se hace nada, no se envía correo para avisar que ya ha vuelto a normal
                  }
                }
              }
            }
         }
       }
    }
    
  }
  

void procesaSemanal()
 {
   // bPRGsem aqui se guarda la asociacion
     RTC.read(tm);      // mirar hora
     for (byte i=0;i<nPRG;i++)
      if (((actPrg[0]==1) && (getbit8(bPRGsem[0],i)==1)) ||
          ((actPrg[1]==1) && (getbit8(bPRGsem[1],i)==1)) || 
          ((actPrg[2]==1) && (getbit8(bPRGsem[2],i)==1)) || 
          ((actPrg[3]==1) && (getbit8(bPRGsem[3],i)==1)) || 
          ((actPrg[4]==1) && (getbit8(bPRGsem[4],i)==1)) )
       {
       if (prgdia[i]!=0)    // hay algún día marcado
        {
         if (getbit(prgdia[i], tm.Wday) == 1)    // coincide el día de la semana
           {
           if (prghor[i] == tm.Hour)               // coincide la hora
             {
             if (prgmin[i] == tm.Minute)  
               {        // coincide el minuto
               if (getbit8(bprgtipo,i)==0)   // señal de salida local
                 {
                 pinVAL(tabpin[prgsal[i]],getbit8(bprgval,i));    // señal local
                 if (LOG==1) logAcc("",tabpin[prgsal[i]],'L',(getbit8(bprgval,i)==1));
                 }
               else
                 if (prgsal[i] < 30)  
                   {
                   pinVal433(prgsal[i],getbit8(bprgval,i));    // señal remota 433
                   if (LOG==1) logAcc("",tabpin[prgsal[i]],'4',(getbit8(bprgval,i)==1));
                   }
                 else 
                   {
                   pinValr(prgsal[i]-51,getbit8(bprgval,i));    // señal remota 2.4
                   if (LOG==1) logAcc("",tabpin[prgsal[i]-51],'R',(getbit8(bprgval,i)==1));
                   }
               }
             }
           }
        }
     }
 }
 
void procesaFechas()
 {
   RTC.read(tm);
   for (byte i=0;i<nPRG;i++)
     if (getbit8(bactfec,i)==1)
       if (fecano[i]+2000 == tmYearToCalendar(tm.Year))
         if (fecmes[i] == tm.Month+1)
           if (fecdia[i] == tm.Day)
             if (fechor[i] == tm.Hour)
               if (fecmin[i] == tm.Minute)  
                 if (getbit8(bprgtipo,i)==0)   // señal de salida local
                   {
                   pinVAL(tabpin[fecsal[i]],getbit8(bfecval,i));    // señal local
                   if (LOG==1) logAcc("",tabpin[fecsal[i]],'L',(getbit8(bfecval,i)==1));
                   }
                 else
                   if (prgsal[i] < 30)  
                     {
                     pinVal433(fecsal[i],getbit8(bfecval,i));    // señal remota 433
                     if (LOG==1) logAcc("",tabpin[fecsal[i]],'4',(getbit8(bfecval,i)==1));
                     }
                   else
                     { 
                     pinValr(fecsal[i]-51,getbit8(bfecval,i));    // señal remota 2.4
                     if (LOG==1) logAcc("",tabpin[fecsal[i]],'R',(getbit8(bfecval,i)==1));
                     }
 }
 
 
void horalcd(boolean exacto)
  {
  lcd.setCursor(0,1);
  if (RTC.read(tm)) 
    { 
    if (tm.Day < 10) printLCD(cero); lcd.print (tm.Day); printLCD(barra);
    if (tm.Month < 9) lcd.print(0); lcd.print (tm.Month+1); printLCD(barra);
    lcd.print (tmYearToCalendar(tm.Year));  printLCD(b);
    if (tm.Hour < 10) lcd.print(0); lcd.print (tm.Hour); 
    printLCD(dospuntos);
    if (tm.Minute < 10) printLCD(cero); lcd.print (tm.Minute); 
    if ((!exacto) || (tm.Second == 0))
      {
      printLCD(dospuntos);
      if (tm.Second < 10) lcd.print(0); 
      lcd.print (tm.Second);
      }
    }
  else    
    {
    if (RTC.chipPresent())  
      printLCD(rtcparado);    
    else 
      printLCD(rtcnodisponible);
    }
  }

///////////////////////////////////////////////////////////////////////////////////////////////////

int mifreeRam()
  {int auxmem;
  auxmem=freeRam();
  if (auxmem < minRam) minRam = auxmem;
  return auxmem;
    
  }
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

OneWire ds1(owPin1);  // on pin 43 (a 4.7K resistor is necessary)
DallasTemperature sensors1(&ds1);
OneWire ds2(owPin2);  // on pin 46 (a 4.7K resistor is necessary)
DallasTemperature sensors2(&ds2);

void leevaloresOW()
  { 
  sensors1.requestTemperatures();
  for (int i=0; i<nTemp1; i++)
    {
    Mb.R[i]=sensors1.getTempC(addr1Wire[i])*100;
    }
  }

void webclientHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  if (!autOK(server))  {server.httpUnauthorized(); return; }
}

void testEEPROM()
  {
    boolean ok=true;
    printS(ttestEEPROM);
    for (int i=0; i<16;i++)  // escribe todo ceros
      {
        writeEEPROM(EEPROM2dir,i,0);
        ok=(ok && (readEEPROM(EEPROM2dir,i)==0));
      }
    for (int i=0; i<16;i++)  // escribe valores
       {
        writeEEPROM(EEPROM2dir,i,i);
        ok=(ok && (readEEPROM(EEPROM2dir,i)==i));
      }
    printlnS(ok?blancoOK:blancoERROR);
  }

void procesa433()
  {
    if (pendiente433Rec>0)
      {
      pendiente433RecMostrar=pendiente433Rec;
      noInterrupts();
      if (modo433Rec==0)    // NewRemote
        {
        if (unit433Rec==15)    // recibe estado
          {
           byte auxnodo=address433Rec & 255;
           byte auxestado=(address433Rec>>8) & 255;
           printS(nodo433);
           for (int i=0; i<max433; i++)
             {
             byte auxunit= (unit433[i] & 15);
             if (addr433[i]==auxnodo)
               {
               if (auxunit==0) setbit8(Mb.C8, i+168, getbit(auxestado,auxunit));
               if (auxunit==1) setbit8(Mb.C8, i+168, getbit(auxestado,auxunit));
               if (auxunit==2) setbit8(Mb.C8, i+168, getbit(auxestado,auxunit));
               if (auxunit==3) setbit8(Mb.C8, i+168, getbit(auxestado,auxunit));
               if (auxunit==4) setbit8(Mb.C8, i+168, getbit(auxestado,auxunit));
               if (auxunit==5) setbit8(Mb.C8, i+168, getbit(auxestado,auxunit));
               }
             }
          }
        else
          {
          for (int i=0; i<max433; i++)
            {
            if ((addr433[i]==(address433Rec & 255)) && ((byte(unit433[i]-48)==unit433Rec)))
              setbit8(Mb.C8,i+168,switchType433Rec);
            }
          }
        }
      if (modo433Rec==1)    // Remote
        {
        for (int i=0; i<max433; i++)
          {
          if (addr433[i]==pendiente433Rec) setbit8(Mb.C8,i+168,1);
          else
            if (code433Off[i]==pendiente433Rec)
              setbit8(Mb.C8,i+168,0);
          }
        }
      pendiente433Rec=0;
      interrupts();
      }
    interrupts();
  }

void leervaloresAna()
  {
    for(int i=0; i<16;i++)      
      Mb.R[i+baseAna]=analogRead(i+54);
  }

void actualizasegunModbus()    // actualiza salidas seg´`un estado Modbus
  {
    for (int i=17; i<41; i++) // Salidas digitales locales
      pinVAL(tabpin[i], getbit8(Mb.C8, tabpin[i]));
//    for (int i=40; i<120; i++) // Salidas digitales remotas / relés remotos
//      if (getbit8(rbshowpin, i+44)==1)
//        pinValr(i, getbit8(Mb.C8, i+44));
  }
  
void actualizaDDNS()
  {
    
//  delay(1000);
//  P(connecting)="Connecting...";
//  P(conectado)="Connected";
//  P(confailed)="Connection failed";
//  printS(connecting);
//  
//  char server[]="www.google.es";
////  char server[]="http://dynupdate.no-ip.com";
//  if (EthClient.connect(server, 80)) 
//    {
//    printlnS(conectado);
////    P(host)="Host: www.google.com";
////    EthClient.println("GET http://ramonlh:pac2ram@dynupdate.no-ip.com/nic/update?hostname=elconucom.zapto.org HTTP/1.1");
//    EthClient.println("GET dynupdate.no-ip.com/nic/update?hostname=elconucom.zapto.org HTTP/1.1");
//    EthClient.println("Host: dynupdate.no-ip.com");
////    EthClient.println("Connection: close");
////    EthClient.println();
//    }
//  else 
//    {
//    printlnS(confailed);
//    }
//    
//    delay(5000);
//    while (EthClient.available()) {
//      char c = EthClient.read();
//      Serial.print(c);
//    }
    
    
    
    
//  
//      char server[]="https://dynupdate.no-ip.com";
//    char dyndnsHost[]="elconucom.no-ip.org";
//    if (dyndnsServ==0)
//      {
//      if(EthClient.connect("https://dynupdate.no-ip.com", 80))
//        {
//        Serial.println("Conectado a NO-IP");
////        EthClient.print("GET /nic/update?hostname=");
////        EthClient.print(dyndnsHost);
////          EthClient.println(" HTTP/1.0");
//        EthClient.println("GET /nic/update?hostname=elconucom.no-ip.org HTTP/1.0");
//        EthClient.println("Host: dynupdate.no-ip.com");
//        EthClient.println("Authorization: user:pass");
//        EthClient.println("User-Agent: Conuco WebServer ramonlh@telefonica.net");
//        EthClient.println();
//
//        //Wait for response
////        delay(5000);
////        Serial.print("Characters available: ");
////        Serial.println(EthClient.available());
//      
//        while(!EthClient.available())
//          {    
////          Serial.println("NO available");
//          while(EthClient.available())
//            {
//            Serial.println("available");
//            char read_char = EthClient.read(); 
//            Serial.write(read_char);
//            }
//          }
//        EthClient.stop();     
//        }
//      }
  }
  
byte backupEE()
{
//  SdFile file(namelogSys, O_WRITE | O_CREAT);
  if (SD.exists(namefilebackup)) 
    SD.remove(namefilebackup); 
  ofstream sdlog(namefilebackup, ios::out | ios::app);  
  if (sdlog)
    {
    for (long i=0; i<65536; i++)
      {
       byte leido=EEread(i);
       sdlog << leido;
       }
     sdlog.close();
     return 0;
    }
  else
    return 1;
}

byte restoreEE()
{
  ifstream sdlog(namefilebackup);
  ArduinoOutStream cout(Serial);
  int c;
  long cont=0;
  if (sdlog.is_open()) 
    {
    while (((c = sdlog.get()) >= 0) && (cont<65536))
      {
      EEwrite(cont,c);
      cont++;
      }
    sdlog.close();
    return 0;
    }
  else
    return 1;
}

byte buscaNodoenTab(unsigned int nodo)
  {
    boolean encontrado=false;
    byte i=0;
    while ((!encontrado) && (i<maxNodos))
      {
        encontrado=(tnodosact[i]==nodo);
        if (encontrado)
          return i;
        else
          i++;
      }
    return 99;
  }

boolean pideDatosRem(int i, int tipo, int nretry, int timeout)
  {
    boolean ok=false;
    boolean ret=false;
    int cont=nretry;
    long mact=0;
    headerT.type=tipo;
    headerT.to_node=tnodosact[i];
    while ((!ok) && (cont>0))
      {
      cont = cont-1;
      bool ok = network.write(headerT,&radmsgT,sizeof(radmsgT));
      if (ok) 
        {
        mact=millis();
        while ((!network.available()) && (millis() < (mact+timeout)))
          {
          network.update();
          }
        while (network.available())
          {
            
          network.read(headerR,&radmsgR,sizeof(radmsgR));
          if ((headerR.type==tipo) && (headerR.from_node==tnodosact[i]))
            {
            ret=true;
            procesaradmsg();
            tlastnodo[i]=0;
            }
          else
            ret=false;
          }
        }
      }
    return ret;
  }

void pidedatosremotos()
  {
  printlnS(avisoINI);
  for (int i=0;i<numNodos;i++)
    if (tnodosact[i]!=0)
      {
      printS(pideDatosRem(i, 2, 10, 100)? OK: NO);
      printS(pideDatosRem(i, 3, 10, 100)? OK: NO);
      printS(pideDatosRem(i, 6, 10, 100)? OK: NO);
      }
  printlnS(avisoFIN);
  }
    
///////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
//  for (int i=0; i<maxRem; i++) antigNod[i]=0;
  
  wdt_disable();   // DESACTIVAR WDT
  SPI.begin(); 
  if (tipoLCD==1) lcd.begin (20,4); 
  if (tipoLCD==2) lcd.begin (40,4); 
  Wire.begin(); 
    
  byte xx=1;
  lcd.print (xx);  leerDatosRed(); xx++;
  lcd.print (xx);  leerDatosDevice(); xx++;
  numPines = numIA+numID+numOD;
  for (byte i = 0; i < 18; ++i)
    if (freepines[i] == 0) {
      pinMode(i, INPUT);  
      digitalWrite(i, INPUT_PULLUP);  
      }
  for (byte i = 22; i < 54; ++i)
    if (freepines[i] == 0) {
      pinMode(i, OUTPUT);  
      digitalWrite(i, valorpin[getbit8(Mb.C8,i)]);  
      }
  pinMode(rstFab,INPUT);  digitalWrite(rstFab,HIGH);    // reset fábrica
  Serial.begin(115200);
  //////////////////////////////////////////////////////////
  if (digitalRead(rstFab)==0)
    {
    printS(espere);
    logSys("Fab+Reset",0,'S',false);
    iniciavalores();
    }
  //////////////////////////////////////////////////////////
//
  lcd.print (xx);  leerDatosRad(); xx++;
    tareason=(pmactRem>0);
  lcd.print (xx);  leerDatosRem();  xx++;
  lcd.print (xx);  leerDatos433();  xx++;
  lcd.print (xx);  leerDatosrTemp();  xx++;
  lcd.print (xx);  leerDatosEventos(); xx++;
  lcd.print (xx);  leerDatosSemanal();  xx++;
  lcd.print (xx);  leerDatosFechas();  xx++;
  lcd.print (xx);  leerEstado();  xx++;
  
  webserver.m_server.chport(EEwebPort);
  lcd.print (xx);  Ethernet.begin(EEmac, EEip);  xx++;  
  lcd.print (xx);  webserver.begin(); 

  for (byte i=0; i<8; i++) { bevenENABLE[0][i] = 255; bevenENABLE[1][i] = 255; }

  webserver.setDefaultCommand(&indexHTML);
  // Máximo 10 comandos. Para ampliar modificar variable m_commands[] en webserver.h
  webserver.addCommand("index.html", &indexHTML);
  webserver.addCommand("sd.html", &setupDevHTML);
  webserver.addCommand("st.html", &setupSondHTML);
  webserver.addCommand("sea.html", &setupEAnagHTML);
  webserver.addCommand("sed.html", &setupEdigHTML);
  webserver.addCommand("ssd.html", &setupSdigHTML);
  webserver.addCommand("sr.html", &setupRemEAnagHTML);
  webserver.addCommand("sred.html", &setupRemEDigHTML);
  webserver.addCommand("srsd.html", &setupRemSDigHTML);
  webserver.addCommand("srsr.html", &setupRemReleHTML);
  webserver.addCommand("sr4.html", &setupRem4HTML);
  webserver.addCommand("sr4code.html", &setupRem4codeHTML);
  webserver.addCommand("srt.html", &setuprTempHTML);
  webserver.addCommand("sse.html", &setupSemHTML);
  webserver.addCommand("sa.html", &setupAvaHTML);
  webserver.addCommand("sv.html", &setupEveHTML);
  webserver.addCommand("sclima.html", &setupClimaHTML);
  webserver.addCommand("sNe.html", &setupNetHTML);
  webserver.addCommand("ss.html", &setupSegHTML);
  webserver.addCommand("srl.html", &setupRelHTML);
  webserver.addCommand("sRa.html", &setupRadHTML);
  webserver.addCommand("sRa4.html", &setupRad4HTML);
  webserver.addCommand("sn.html", &setupNodHTML);
  webserver.addCommand("rst.html", &resetHTML);
  webserver.addCommand("rn.html", &resetNodHTML);
  webserver.addCommand("sn2.html", &setupNod2HTML);
  webserver.addCommand("sprem.html", &setupPerRemHTML);
  webserver.addCommand("sf.html", &setupFecHTML);
  webserver.addCommand("sesc.html", &setupEscHTML);
  webserver.addCommand("sprg.html", &setupPrgHTML);
  webserver.addCommand("slinks.html", &setupLinksHTML);
  webserver.addCommand("sy.html", &systemHTML);
  webserver.addCommand("wc.html", &webclientHTML);
  webserver.addCommand("slogs.html",&ShowLogCmd );   // muestra ficheros log
  webserver.addCommand("sfiles.html",&ShowFilesSD );   // muestra ficheros en SD
  webserver.addCommand("on",&onCmd);         // pone a ON salida digital  sintaxis: /on?nn
  webserver.addCommand("off",&offCmd);       // pone a OFF salida digital  sintaxis: /off?nn   
  webserver.addCommand("onr",&onrCmd);       // pone a ON salida digital remota  sintaxis: /onr?nn
  webserver.addCommand("offr",&offrCmd);     // pone a OFF salida digital remota  sintaxis: /offr?nn   
  webserver.addCommand("on4",&on4Cmd);       // pone a ON salida digital remota  sintaxis: /on4?nn
  webserver.addCommand("off4",&off4Cmd);     // pone a OFF salida digital remota  sintaxis: /off4?nn   
  webserver.addCommand("sf",&ShowFileCmd);   // borra fichero de la SD     sintaxis: /sf?namefile.txt
  webserver.addCommand("df",&DeleFileCmd);   // borra fichero de la SD     sintaxis: /df?namefile.txt
  webserver.addCommand("json",&jsonCmd);
  webserver.addCommand("jsonnames",&jsonnamesCmd);

// 
  lcd.clear();
  printLCD(OK);
  for (byte i=0; i<3;i++) {lcd.print(EEip[i]); printLCD(punto);} lcd.print (EEip[3]);
  printLCD(dospuntos); lcd.print(EEwebPort); 
  lcd.setCursor(0,2); 
  RTC.read(tmInicio);
  horalcd(false);
//
  sensors1.begin();
  nTemp1=sensors1.getDeviceCount();
//  sensors2.begin();
//  nTemp2=sensors2.getDeviceCount();
//
    printS(conucoweb); Serial.print(miversion);
    printS(punto); Serial.println(misubver);
    serialmem(" "); Serial.println();
    
    printS(lanip);
    for (byte i=0; i<4;i++) { Serial.print(EEip[i]); if (i<3) printS(punto); }
    printS(dospuntos);
    Serial.print(EEwebPort);
    printS(blancogw);
    for (byte i=0; i<4;i++) { Serial.print(EEgw[i]); if (i<3) printS(punto); }
    Serial.println();

    printS(rf24vel); Serial.print(radio.getDataRate());
    printS(blancocanal); Serial.print(radCan[0]);
    printS(blancopsize); Serial.print(radio.getPayloadSize());
    printS(blancocrcl); Serial.print(radio.getCRCLength());
    printS(blancolevel); Serial.println(radio.getPALevel()); 
    printS(blnacorfnet); Serial.println(radNOD,OCT);
    creatablanodos();
    printS(nodosusados); Serial.println(numNodos);
    for (int i=0; i<numNodos; i++)
      {
       printS(nodooct); printS(dospuntos);printS(b); Serial.println(tnodosact[i],OCT);
      }
    printS(blanco1wp1); Serial.print(nTemp1, DEC);
    printS(blancoSondas);
    printS(blancomodo);
    if (sensors1.isParasitePowerMode()) printS(parasite); else printS(power);
    Serial.println();
    for (int i=0; i<nTemp1; i++)   {
      if (sensors1.getAddress(addr1Wire[i], i))     
        {
        sensors1.setResolution(prectemp);
        printS(blancos4);
        for (uint8_t j = 0; j < 8; j++)     
          {
          if (addr1Wire[i][j] < 16) printS(cero);
          Serial.print(addr1Wire[i][j], HEX);   
          }
        Serial.println();
        }
      }
    printS(blanco1wp2); Serial.print(nTemp2, DEC);
    printS(blancoSondas);
    printS(blancomodo);
    if (sensors2.isParasitePowerMode()) printS(parasite); else printS(power);
    Serial.println();
     // aquí falta sacar la lista de sondas

    leevaloresOW();
  ////////////////////////////////////////////////////
  // configura radio 433
  NewRemoteReceiver::init(-1, 2, NewISR4);
  RemoteReceiver::init(-1, 2, ISR4);
  InterruptChain::setMode(4, HIGH);

  InterruptChain::addInterruptCallback(4, RemoteReceiver::interruptHandler);
  InterruptChain::addInterruptCallback(4, NewRemoteReceiver::interruptHandler);

  ////////////////////////////////////////////////////  
  
// inicia tarjeta SD
     pinMode(ethSEL,OUTPUT);
     sdHay=SD.begin(sdSEL,SPI_HALF_SPEED);
     printS(tSD);
     printlnS(sdHay?Ok:NO);
     if (!SD.begin(sdSEL, SPI_HALF_SPEED)) SD.initErrorHalt();

  // Configura radio NRF24L01
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(radCan[0]);
  network.begin(radCan[0], radNOD);
  
//  digitalWrite(ethSEL, LOW);    // pin 10
  EEPROM_readAnything (dirEEfactorA, factorA);
  EEPROM_readAnything (dirEEoffsetA, offsetA);
  EEPROM_readAnything (dirEEoffsetA, bsumatA);
  EEPROM_readAnything (dirEEesc, bescena);
  mact1 =millis();
  mact10 =millis();
  mact60 =millis();
  mactVar =millis();
  mactRem =millis();  logSys("Reset",0,'S',false);
  testEEPROM();
  printlnS(ready); 
  for (byte i=0; i<maxNodos; i++) tlastnodo[i]=0;
  for (int i=0; i<16; i++) 
    {
      vanaTED[i]=0;
      vanaHED[i]=0;
    }
  if (tareason) pidedatosremotos();
// actualizaDDNS(); 
}

void loop()    // el bucle se ejecuta unas 75 veces/segundo
{
  char buff[buffLEN];
  int len = buffLEN;  
  contador++;
  if ((millis() > (mact1 + 1000)))    // tareas que se hacen cada segundo
    {
      mact1=millis();
      velocidad=contador;
      contador=0;
      leervaloresAna();
      if ((actPrg[0]==1) || (actPrg[1]==1) ||(actPrg[2]==1) ||
          (actPrg[3]==1) || (actPrg[4]==1))   // algún programa activo 
        {
         procesaSemanal();
         procesaEventos();
        }
       procesaFechas();
    }
  if ((millis() > (mact10 + 10000)))    // tareas que se hacen cada 10 segundos
    {
      mact10=millis();
//      velocidad=contador/10;
//      contador=0;
//        actualizasegunModbus();
      for (int i=0; i<16; i++)       // leer valores de sondas DHTxx
        {
        if (tipoED[i]==1)            // DHT11
          {
          dht.begin(tabpin[i],DHT11);
          vanaTED[i]=dht.readTemperature(false); 
          vanaHED[i]=dht.readHumidity(); 
          }
        if (tipoED[i]==2)            // DHT21
          {
          dht.begin(tabpin[i],DHT21);
          vanaTED[i]=dht.readTemperature(false); 
          vanaHED[i]=dht.readHumidity(); 
          }
        if (tipoED[i]==3)            // DHT22
          {
          dht.begin(tabpin[i],DHT22);
          vanaTED[i]=dht.readTemperature(false); 
          vanaHED[i]=dht.readHumidity(); 
          }
        }
      leevaloresOW();  
    }
  if ((millis() > (mact60 + 60000)))    // tareas que se hacen cada 60 segundos
    {
     mact60=millis();
     for (byte i=0; i<maxNodos; i++) if (tlastnodo[i] < 255) tlastnodo[i]++;
//     actualizaDDNS();
    }
  if ((millis() > (mactVar + (pmactVar*1000))))    // tareas que se hacen cada pmactVar segundos
    {
      mactVar=millis();
//      if (mactVar>0) logTemp();
    }
  if (tareason)
    {
    if ((millis() > (mactRem + (pmactRem*1000))))    // tareas que se hacen cada pmactRem segundos
      {
      mactRem=millis();
      pidedatosremotos();
      }
    }
//  
  for (byte i=0; i<8;i++) //  actualiza contadores
    {     
     if (Mb.C8[i+32]!=digitalRead(tabpin[i]))
       {
       if (digitalRead(tabpin[i])==1)
         {
         contadores[i]++;
         EEPROM_writeAnything(dirEEcontadores+(4*i), contadores[i]);
         }
       Mb.C8[i+32]=digitalRead(tabpin[i]);
       }
    }

  Mb.Run();    // actualizar Modbus
  webserver.processConnection(buff, &len);
     
  if (arrancando) arrancando = false;
  
  if (radACT) 
    {
    network.update();
    if (network.available())
      {
      network.read(headerR,&radmsgR,sizeof(radmsgR));
      procesaradmsg();
      byte n=buscaNodoenTab(headerR.from_node);
      if (n!=99) 
        tlastnodo[n]=0;
      }
    }
  procesa433();
 
}
/////////////// end loop ////////////////////////////

void NewISR4(NewRemoteCode receivedCode) 
  {
  if (pendiente433Rec!=0) return;
  ninter++;
  modo433Rec=0;
  pendiente433Rec=receivedCode.address;
  address433Rec=receivedCode.address;
  unit433Rec=receivedCode.unit;
  switchType433Rec=receivedCode.switchType;
  }
  
void ISR4(unsigned long receivedCode, unsigned int period) 
  {
  if (pendiente433Rec!=0) return;
  ninter++;
  modo433Rec=1;
  pendiente433Rec=receivedCode;
  address433Rec=0;
  unit433Rec=0;
  switchType433Rec=0;
  }
  
