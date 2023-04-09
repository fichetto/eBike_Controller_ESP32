/*
DA IMPLEMENTARE:
- Lettura cardiofrequenzimetro da BLE
- Comando del servomotore e implementazione algoritmo per la cambiata automatica
- verificare se necessario il sensore MEMS
- Agganciare al WiFi e inviare i dati tramite MQTT




Implementati:
Rev17 - 20220226 - Mappatura dei livelli di assistenza, rimozione errore della divisione per zero, utilizzo dell'acceleratore per aumentare il livello di assistenza a comando (solo x papà con scheda modificata, il piedino 18 è stato spostato al 27)

rev16
	-Valori mappatura Wh batteria aggiornati

rev15
	-Mappatura tensione batteria a vuoto su curva Wh batteria e percentuale di carica
	-Blocco erogazione al di sotto della tensione soglia batteria.

rev14
	-Controllo tramite potenziometro con rotazione ridotta e pulsante multifunzione (boost, bypass sensore, cambio pagina, calibrazione potenziometro, reset)
	-Visualizza tensione batt quando scompare la prima tacca
	-Pagina di avviso cambio modalit� pulsante
	-Utilizzo dell'UART0 anzich� 1
	-Baudrate seriale a 9600
	-Frequenza CPU portata ad 80Mhz (anzich� 240)
	-Ottimizzazione elementi grafici display della pagina principale
	-Creazione procedura di calibrazione del potenziometro con visualizzazioni display e memorizzazione valori in eeprom
	-SensoreON memorizzato in eeprom
	-Visualizzazione numerica dei valori corrente e potenza all'interno dei Gauge

rev13
	-Controllo potenza tramite pulsanti

rev12
	-Introduzione bypass sensore pedalata tramite jumper sulla schedina da rimuovere in caso di avaria del sensore pedalata

rev11
	-Livello batteria rilevato solo a bassa corrente
	-Timeout rpmpedalata dinamico inversamente proporzionale agli rpm. Questo consente di avere un tempo di timeout + lungo quando si pedala lentamente e pi� reattivo ad alti regimi.
	-Ulteriore eliminazione di parti di codice per sensore forza non pi� utilizzate

rev10
	-Software dual core per miglioramento prestazioni ed affidabilit�
	-Eliminazione parti di programma non pi� utilizzate (sensore di forza)
	-Interfaccia display migliorata
	-Modifiche varie sostanziali

rev9
	-Cambio pagina display tramite potenziometro: fondo scala - 3/4 - fondo scala
	-Correzioni varie alla visualizzazione display
	-Filtro tensione letta dalla Vesc
	-Ottimizzazione e cambio parametri rampa di erogazione corrente

rev8
	-Implementazione rilevamento verso di pedalata tramite doppio sensore hall connessi agli ex pinrivoluzionepuleggia e pinrivoluzioneruota con relativo cambio di denominazione
	-Eliminazione pulsanti
	-Miglioramento gestione rampa di accelerazione (eliminazione overshooting)

rev7
	-Implementazione rampa di accelerazione positiva
	-Introduzione filtro media mobile potenziometro
	-Implementazione assistenza proporzionale alla frequenza di pedalata + pagina di configurazione

rev6
	-Implementazione potenziometro per controllo senza sensore di forza. L'abilitazione avviene selezionando il livello di assistenza 0

rev5
	-Implementazione comunicazione UART con Vesc e visualizzazione valori su display
	-Implementazione output DAC
	-Modifica gpio SCK cella di carico

rev4
	-Introduzione livelli di assistenza + pagina di configurazione dedicata
	-Modifiche varie interfaccia di visualizzazione
	-Range di rilevamento marce modificati (dimetro puleggia variato)

rev3
	-Reintroduzione media mobile con prevalenza dato istantaneo positivo (superiore al medio)
	-Azzeramento per timeout degli rpm ruota e pedalata
	-Spinta attiva solo se rpm ruota > 0
	-disattivazione interrupt durante salvataggio dati
	-Introduzione possibilit� di taratura cella in relaiva pagina tramite pressione simultanea tasti + e -

rev2
	-Software modificato per ESP32
		Libreria modificata rispetto alla versione per arduino
		Simulazione EEPROM
		Utilizzo libreria ESP32_Servo per l'output Vesc
		Riassegnazione pin
	-Lettura cella di carico invece di FSR
	-Modifiche minori per la lettura della frequenza pedalata

rev1
	-prima versione collaudata
*/




#include <Adafruit_SH1106.h>	//NB: La libreria utilizzata per la esp32 � diversa da quella per arduino 
#include <TimeUtility.h>
#include <Adafruit_GFX.h>
#include <VescUart.h>
#include <EEPROM.h>
#include <HardwareSerial.h>

/*
#define TORQUE_SENSOR_EN 1
1: abilitato
0: disabilitato
*/
#define TORQUE_SENSOR_EN 1

#define OLED_ADDR		0x3C
#define OLED_SDA		16
#define OLED_SCL		17

#define pinTorque				34			//Pin input sensore coppia movimento centrale
#define pinPAS_mvc				32			//Pin ing. sensore pas del movimento centrale
#define pinBtn					12			//Pin pulsante per sistema senza potenziometro (sensore coppia)
#define pinPot					12
#define pinThrottle				27			//Pin acceleratore
#define pinBtnUP				14
#define pinBypassPed			21
#define NumeroMagneti			6			//Numero magneti installati su corona
#define Hall1					32
#define Hall2					34
#define SampleMediaADC			100			//Campioni media mobile filtro ADC potenziometro/coppia

#define CurrentMax				20.0f		//Massima corrente erogabile al motore tramite comandi su UART
#define TimeBaseAccel			100			//Tempo base (in ms) per incremento output per la gestione dell'accelerazione. Definisce la frequenza di aggiornamento del valore durante l'accelerazione 
#define AccelerationValue		15.0f		//Ampere al secondo
#define StartPage				8
#define ADCdivider				3			//Divisore del valore adc per corsa potenziometro limitata
#define maxValueADC				4095		//valore max ADC (4095 per ESP32)
#define OffsetADC				180			//Valore di offset ADC dovuto all'inserimento della resistenza in serie al gnd del potenziometro
#define remapMaxValueADC		(maxValueADC - OffsetADC)				
#define EEPROM_SIZE				100
#define CUTOFF_VOLTAGE			30.f		//Tensione soglia per il blocco dell'erogazione
#define TORQUE_TH				25			//valore di coppia oltre la quale si attiva la trazione nella partenza da fermo (rpmPedalata = 0)
#define MIN_TORQUE_ADC_VAL		1700		//Valore minimo dell'adc del sensore di coppia
#define MAX_TORQUE_ADC_VAL		4095		//Valore massimo dell'adc del sensore di coppia
#define MIN_THROTTLE_ADC_VAL	1700		//Valore minimo dell'adc dell'acceleratore
#define MAX_THROTTLE_ADC_VAL	4095		//Valore massimo dell'adc dell'acceleratore
#define QTY_ASSIST_LEVEL		5.f			//Numero di livelli di assistanza per la modalit� sensore di coppia (senza potenziometro)
#define TICK_X_REVOLUTION_PAS   32			//Impulsi per rivoluzione del sensore PAS

Adafruit_SH1106 display(16, 17);	//NB: La libreria utilizzata per la esp32 � diversa da quella modificata  
TimeTrigger timerAccel;				//temporizzatore per la gestione dell'accelerazione
VescUart UART;
Timer timerButton;					//timer rilevamento pressione prolungata 
Timer timerSpecialBttFunction;		//timeout per azzerare conteggio pressioni pulsante per selezionare funzioni speciali (bypass sensore pedalata, calibrazione pot e reset
Timer timerPotCalibration;			//Timer per la durata della procedura di calibrazione del potenziometro
TaskHandle_t Task1;
HardwareSerial SerialVesc(1);

volatile unsigned long prevmillisPed = 0;	//per pedivella
volatile byte RpmPedaling = 0;		//Rpm pedivella
volatile int  RpmPedVariation = 0;  // Derivata degli RPM della pedivella
byte pagina = StartPage;			//pagina display
volatile float current = 0.0f;		//Corrente mappata su potenziometro 
volatile float sendCurrent;			//valore di corrente calcolato secondo le rampe di accelerazione e decelerazione della funzione 
unsigned long tempo = 0;
volatile unsigned long intervallo;
volatile bool AbilitaTrazione = false;
volatile int counterPAS = 0;		//Conta i segnali del sensore di pedalata
volatile bool flagHall;				//utilizzo questa variabile per abilitare la trazione solo dopo un secondo segnale di pedalata, viene resettata dal timeout di pedalata		
bool prevValueHall;					//flag che memorizza lo stato precedente del flagHall
long prevmillisHall;				//
volatile float outputUartCurrent;	//Corrente impartita alla Vesc
volatile int FilteredADCvalue;		//Valore ADC filtrato potenziometro
int timeout;						//Timeout rpmpedalata e trazione attiva
bool flagSecondoSegnale = false;	//questo flag consente di abilitare la trazione solo al secondo segnale di giro pedivella
bool SensorON = true;				//flag per attivazione/spegnimento sensore pedalata
volatile int ADCrawValue;
volatile int ADC_Throttle;
volatile int MinPot;
volatile int MaxPot;
volatile int testmspassed = 0;
volatile byte Drive_0_100;			//volore potenziometro/coppia mappato su range 0-100
volatile bool flagStop = false;		//flag di stop erogazione a batteria scarica
byte AssistLevel = 0;				//livello di assistenza selezionato
volatile float AssistLevelFLoat[6] = { 0.0f,1.0f,1.5f,2.0f,3.0f, 5.0f };
volatile float AssistComposed;


void setup()
{
	Serial.begin(115200);
	setCpuFrequencyMhz(80);

	xTaskCreatePinnedToCore(
		Task1code, /* Function to implement the task */
		"Task1", /* Name of the task */
		10000,  /* Stack size in words */
		NULL,  /* Task input parameter */
		0,  /* Priority of the task */
		&Task1,  /* Task handle. */
		0); /* Core where the task should run */

#if TORQUE_SENSOR_EN == 1
	pinMode(pinPAS_mvc, INPUT);
	pinMode(pinBtn, INPUT_PULLDOWN);
	attachInterrupt(digitalPinToInterrupt(pinPAS_mvc), RilevaPedalata, FALLING);
#else
	pinMode(Hall1, INPUT_PULLUP);
	pinMode(Hall2, INPUT_PULLUP);
	pinMode(pinBypassPed, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(Hall2), RilevaPedalata, FALLING);
#endif 

	timerAccel.setInterval(TimeBaseAccel);
	timerButton.setTimer(400);
	timerSpecialBttFunction.setTimer(10000);
	timerPotCalibration.setTimer(20000);

	EEPROM.begin(EEPROM_SIZE);
	MinPot = EEPROM.readInt(0);
	MaxPot = EEPROM.readInt(4);
	SensorON = EEPROM.readBool(8);
}


int testcount;
bool flagLongPressedBtt = false;
bool Button = 0;
bool flagPressedBtt = 0;
int CountBttSpecialFunction;
bool ManualThrottle = false;		//flag che attiva il motore anche in assenza di pedalata, tramite pressione prolungata del tasto (tranne in modalit� torque sensor)

void loop()
{

#if TORQUE_SENSOR_EN == 0
	ADCrawValue = analogRead(pinPot);

	if (ADCrawValue <= MaxPot + 200)	//Aggiorna FilteredADC (utilizzata per l'output vesc) solo se il pulsante non � premuto, diversamente ritiene il valore precedente.
	{
		FilteredADCvalue = FilterADC(ADCrawValue);
		percentPot = constrain((map(FilteredADCvalue, MinPot, MaxPot - 100, 0, 100)), 0, 100);
		Button = false;		//Quando il pulsante viene rilasciato, resetto il suo stato
	}
	else if (ADCrawValue > maxValueADC - 200)
	{
		Button = true;		//Pulsante premuto
		delay(50);			//delay come filtro per evitare ulteriori avanzamenti di pagina indesiderati
	}


	//-----------------BUTTON---------------------------------------
	if (Button == true)
	{
		flagPressedBtt = true;
		timerButton.startTimer();
		if (timerButton.checkTimer() == true)		//Pressione prolungata
		{
			ManualThrottle = true;
			timerButton.resetTimer();
			flagLongPressedBtt = true;
		}
	}

	if (Button == false && flagPressedBtt == true)
	{
		flagPressedBtt = false;
		if (flagLongPressedBtt == false)			//pressionebreve: se non � avvenuta la pressione prolungata allora rilevo quella breve
		{
			CountBttSpecialFunction++;
			pagina++;
			timerSpecialBttFunction.startTimer();		//timeout per conteggio pressioni pulsante per selezionare la modalit� di bypass pedalata
			//premendo 10 volte spegne o attiva il sensore di pedalata:
			//quando � OFF, la pressione lunga sul tasto attiva la trazione con valore di corrente in base al potenziometro
			//quando � ON, la pressione prolungata attiva la trazione con valore massimo (boost)
			if (CountBttSpecialFunction == 10)
			{
				SensorON = !SensorON;
				EEPROM.writeBool(8, SensorON);
				EEPROM.commit();
				pagina = 20;
			}
			if (CountBttSpecialFunction == 25)
			{
				pagina = 31;					//pagina di setup min max potenziometro
				//flagPotCalibration = true;		//flag che attiva funzione di memorizzazione min max
				potCalibrationRoutine();

			}
			if (CountBttSpecialFunction > 35)
			{
				ESP.restart();
			}
		}
		timerButton.resetTimer();
		ManualThrottle = false;
		flagLongPressedBtt = false;
	}

	if (timerSpecialBttFunction.checkTimer() == true)
	{
		CountBttSpecialFunction = 0;
		timerSpecialBttFunction.resetTimer();
	}


	/*Rileva pedalata e calcolo dell'output
	Vengono eseguiti solo se ADCvalue � superiore a 0 ovvero quando il potenziometro non � a 0 oppure viene premuto il pulsante
	*/
	if (Drive_0_100 > 5 || ManualThrottle == true)
	{
		if (SensorON == true)		//Se il sensore pedalata non � spento, utilizza la logica di funzionamento normale. 
		{
			if (flagHall != prevValueHall)
			{
				unsigned long deltamillis = millis() - prevmillisHall;
				RpmPedaling = 60000 / (deltamillis * NumeroMagneti);
				prevmillisHall = millis();
				prevValueHall = flagHall;
				timeout = deltamillis * 2;
				if (flagSecondoSegnale == true)
				{
					AbilitaTrazione = true;
				}
				flagSecondoSegnale = true;
			}
			else if (RpmPedaling != 0 && flagHall == prevValueHall && millis() > (prevmillisHall + timeout))
			{
				AbilitaTrazione = false;
				RpmPedaling = 0;
				flagSecondoSegnale = false;
			}
			else if (RpmPedaling != 0 && flagHall == prevValueHall && millis() > (prevmillisHall + 1500))
			{
				AbilitaTrazione = false;
				RpmPedaling = 0;
				flagSecondoSegnale = false;
			}
		}
		else								//Se il sensore � spento utilizzo l'accelerazione tramite pulsante.
		{
			if (ManualThrottle == true)
			{
				AbilitaTrazione = true;
			}
			else
			{
				AbilitaTrazione = false;
			}
		}


		// --- Calcolo output ---
		if (AbilitaTrazione == true)
		{
			current = map(Drive_0_100, 0, 100, 0, CurrentMax * 1000.0f) / 1000.0f;
			if (current >= CurrentMax) //constrain
			{
				current = CurrentMax;
			}
		}
		else
		{
			current = 0.0f;
		}
		if (SensorON == true && ManualThrottle == true) //se il sensore pedalata � attivo e viene premuto il pulsante SX attivo il boost sovrascrivendo il valore
		{
			current = CurrentMax;
		}

		if (flagStop == false)	//se il blocco per batteria scarica non � attivo
		{
			outputUartCurrent = SmoothCurrent(current);
		}
		else
		{
			outputUartCurrent = 0.f;
		}

	}
	else
	{
		outputUartCurrent = 0.f;
	}

#else

	// --- Lettura coppia --- 
	ADCrawValue = analogRead(pinTorque);
	ADC_Throttle = analogRead(pinThrottle);
	Serial.println(ADCrawValue);
	FilteredADCvalue = FilterADC(ADCrawValue);
	Drive_0_100 = constrain((map(FilteredADCvalue, MIN_TORQUE_ADC_VAL, MAX_TORQUE_ADC_VAL - 100, 0, 100)), 0, 100);
	AssistComposed = map(ADC_Throttle, 1500, 4095, 0.0, 5.0);
	if (AssistComposed < 0) {
		AssistComposed = 0;
	}
	//voltage = map(voltage, 0, V_REF / 1000.0, 0, 5);
	// --- Calcolo Rpm pedalata --- 
	static int prevCounterPAS = 0;
	unsigned long deltamillis = 0;
	if (counterPAS > prevCounterPAS)
	{
		deltamillis = millis() - prevmillisHall;
		if (deltamillis != 0) {
			RpmPedaling = 60000 / (deltamillis * TICK_X_REVOLUTION_PAS);
		}
		prevmillisHall = millis();
		timeout = deltamillis * 5;	//tempo di timeout che porta a 0 la cadenza 
		prevCounterPAS = counterPAS;
	}
	else if (millis() > prevmillisHall + timeout)
	{
		RpmPedaling = 0;
	}

	// Calcolo la variazione di velocità della pedivella
	static int prevRPMpedal = 0;
	RpmPedVariation = RpmPedaling - prevRPMpedal;
	prevRPMpedal = RpmPedaling;


	// --- Calcolo output ---
	if ( RpmPedaling > 7 || Drive_0_100 > TORQUE_TH)
	//if (RpmPedVariation > 1 && Drive_0_100 > TORQUE_TH)
	{
		//current = (map(Drive_0_100, 0, 100, 0, CurrentMax * 1000.0f) / 1000.0f) * pow(AssistLevel, 1.2f) * 0.8; //(AssistLevel / QTY_ASSIST_LEVEL);
		
		
		if (AssistComposed < AssistLevelFLoat[AssistLevel]) {
			AssistComposed = AssistLevelFLoat[AssistLevel];
		}

		current = (map(Drive_0_100, 0, 100, 0, CurrentMax * 1000.0f) / 1000.0f) * AssistComposed * 0.8; //(AssistLevel / QTY_ASSIST_LEVEL);

		if (current >= CurrentMax) //constrain
		{
			current = CurrentMax;
		}
	}
	else
	{
		current = 0.0f;
	}
	outputUartCurrent = SmoothCurrent(current);

	// --- Gestione pulsante ---
	Button = digitalRead(pinBtn);
	if (Button == true)
	{
		delay(50);	//filtro per evitare avanzamenti indesiderati di pagina
		flagPressedBtt = true;
		timerButton.startTimer();
		if (timerButton.checkTimer() == true)		//Pressione prolungata
		{
			pagina++;
			timerButton.resetTimer();
			flagLongPressedBtt = true;
		}
	}

	if (Button == false && flagPressedBtt == true)
	{
		flagPressedBtt = false;
		if (flagLongPressedBtt == false)			//pressionebreve: se non � avvenuta la pressione prolungata allora rilevo quella breve
		{
			AssistLevel++;
			if (AssistLevel > QTY_ASSIST_LEVEL)
			{
				AssistLevel = 0;
			}
			CountBttSpecialFunction++;
			timerSpecialBttFunction.startTimer();		//timeout per conteggio pressioni pulsante per selezionare la modalit� di bypass pedalata
			if (CountBttSpecialFunction > 35)
			{
				ESP.restart();
			}
		}
		timerButton.resetTimer();
		flagLongPressedBtt = false;
	}

	if (timerSpecialBttFunction.checkTimer() == true)
	{
		CountBttSpecialFunction = 0;
		timerSpecialBttFunction.resetTimer();
	}
#endif
}


float incrementValue = AccelerationValue / (1000.0f / TimeBaseAccel);
/*Gestisce la rampa di accelerazione per drive con potenziometro o di decelerazione per drive con sensore di coppia.
Nel caso di sensore di coppia, questa funzione garantisce un'erogazione pi� uniforme in quanto durante la pedalata.
Infatti, durante la pedalata, la coppia letta dal sensore ha andamento sinusoidale che risulterebbe fastidiosa
se l'erogazione al motore seguisse esattamente il suo andamento. Pertanto nella fase di spinta sul pedale (coppia crescente)
viene copiato il valore di corrente istantaneo calcolato a partire dal sensore;
quando invece la coppia decresce viene applicata una rampa che mantiene parte dell'erogazione fino alla successiva spinta in pedalata.
Gestisce anche gli eventuali valori fuori soglia*/
float SmoothCurrent(float Value) {
#if TORQUE_SENSOR_EN == 0
	//Gestione rampa
	if (Value > sendCurrent)
	{
		if (timerAccel.checkTime() == 1)
		{
			if (UART.data.avgMotorCurrent > 6.0f)		//se la corrente del motore � superiore ad un certo valore, evito la rampa di accelerazione
			{
				sendCurrent = Value;
			}
			else
			{
				float tmpsendCurrent = sendCurrent + incrementValue;	//questa variabile memorizza temporaneamente il valore incrementato che viene successivamente copiato nella variabile definitiva
				if (tmpsendCurrent >= Value)
				{
					sendCurrent = Value;
				}
				else
				{
					sendCurrent = tmpsendCurrent;
				}
			}

		}
	}
	else if (Value < sendCurrent)
	{
		sendCurrent = Value;
	}

	//Gestione valori fuori soglia
	if (sendCurrent > CurrentMax)
	{
		sendCurrent = CurrentMax;
	}
	else if (sendCurrent < 0.0f)
	{
		sendCurrent = 0.0f;
	}

	return sendCurrent;

#else	
	float decrementValue = RpmPedaling / 70.f;		//proporzionale alla cadenza: aumentare il divisore per rallentare la rampa di diminuzione
	//Gestione rampa 
	if ((RpmPedaling < 7 || RpmPedVariation<-35) && Drive_0_100 < TORQUE_TH)
	//if (RpmPedVariation < -20 && Drive_0_100 < TORQUE_TH)
	{
		sendCurrent = 0.0f;
		decrementValue = 2.0;	//quando si spinge sul pedale mentre si � fermi, RpmPedaling � 0 pertanto anche decrementValue lo �: ponendolo ad un certo valore si fa in modo che il decremento (e quindi la rampa di diminuzione spinta) avvenga a rpmPedaling=0
	}
	else if (Value < sendCurrent)
	{
		if (timerAccel.checkTime() == 1)
		{
			sendCurrent = sendCurrent - decrementValue;
		}
	}
	else
	{
		sendCurrent = Value;
	}

	//Gestione valori fuori soglia
	if (sendCurrent > CurrentMax)
	{
		sendCurrent = CurrentMax;
	}
	else if (sendCurrent < 0.0f)
	{
		sendCurrent = 0.0f;
	}

	return sendCurrent;
#endif // TORQUE_SENSOR_EN == 0
}

unsigned long sumADC;
byte indx1 = 0;
unsigned int ptrSamplesADC[SampleMediaADC];
/*Filtra l'ingresso ADC */
unsigned int FilterADC(unsigned int Valore) {
	sumADC -= ptrSamplesADC[indx1];
	ptrSamplesADC[indx1] = Valore;
	sumADC += ptrSamplesADC[indx1];
	unsigned int avg = sumADC / (SampleMediaADC);
	if (indx1 < (SampleMediaADC - 1))
	{
		indx1++;
	}
	else { indx1 = 0; }
	return avg;
}

/*Interfaccia display + output vesc*/
void Task1code(void* pvParameter) {

	SerialVesc.begin(9600, SERIAL_8N1, 19, 21);
	while (!SerialVesc) { ; }
	UART.setSerialPort(&SerialVesc);

	display.begin(SH1106_SWITCHCAPVCC, 0x3C);
	display.setRotation(2);
	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(40, 10);

	display.print("rev16");
	display.display();

	int WhBatteryMap[] = { 0,25,50,75,100,200,400,530,670,820,940,1170,1200 };	//da 30V a 42V
	byte percentBatt;

	for (;;) {

		UART.setCurrent(outputUartCurrent);
		UART.getVescValues();
		//float testbatt = map(percentPot, 0, 100, 280, 420) / 10.f;
		float TensioneBatt = ((TensioneBatt * 10.0f) - TensioneBatt + UART.data.inpVoltage) / 10.0f;		//filtro tensione 
		//Serial.println(TensioneBatt);
		if (TensioneBatt < CUTOFF_VOLTAGE)
		{
			flagStop == true;
		}
		if (TensioneBatt >= 30.f)
		{
			byte pos = TensioneBatt - 30;
			byte decimal = (TensioneBatt - 30.f - pos) * 10.f;
			int remainingWhBatt = map(decimal, 0, 10, WhBatteryMap[pos], WhBatteryMap[pos + 1]);
			percentBatt = map(remainingWhBatt, 0, 1200, 0, 100);
		}
		else
		{
			percentBatt = 0;
		}


		if (pagina == 5)
		{
			display.clearDisplay();

			byte line1 = map(outputUartCurrent * 1000, 0, CurrentMax * 1000, 0, 127);
			display.fillRect(0, 0, line1, 3, WHITE);

			display.setTextSize(1);
			display.setCursor(0, 6);
			display.print("Ah: ");
			display.print(UART.data.ampHours);

			display.setCursor(0, 16);
			display.print("Inp Curr: ");
			display.print(UART.data.avgInputCurrent);

			display.setCursor(0, 26);
			display.print("Mot Curr: ");
			display.print(UART.data.avgMotorCurrent);

			display.setCursor(0, 36);
			display.print("WrtCurr: ");
			display.print(current);

			display.setCursor(0, 46);
			display.print("V batt: ");
			display.print(UART.data.inpVoltage);

			display.setCursor(0, 56);
			display.print("Duty: ");
			display.print(UART.data.dutyCycleNow);

			display.setCursor(64, 6);
			display.print("Thr: ");
			display.print(ADC_Throttle);

			display.display();
		}
		else if (pagina == 6)
		{
			display.clearDisplay();

			byte line1 = map(outputUartCurrent * 1000, 0, CurrentMax * 1000, 0, 127);
			byte line2 = map(Drive_0_100, 0, 100, 0, 127);

			display.fillRect(0, 0, line1, 3, WHITE);
			display.fillRect(0, 5, line2, 3, WHITE);

			display.setTextSize(1);
#if TORQUE_SENSOR_EN == 0
			display.setCursor(0, 10);
			display.print("Count Hall: ");
			display.print(counterPAS);
			display.setCursor(120, 10);
			display.print(flagHall);

			display.setCursor(0, 19);
			display.print("RPM Ped: ");
			display.print(RpmPedaling);

			display.setCursor(0, 28);
			display.print("WriteCurr: ");
			display.print(outputUartCurrent);

			display.setCursor(0, 37);
			display.print("Motor Curr: ");
			display.print(UART.data.avgMotorCurrent);

			display.setCursor(0, 46);
			display.print("Pot %: ");
			display.print(Drive_0_100);

			display.setCursor(0, 55);
			display.print("ADCraw: ");
			display.print(ADCrawValue);

			display.setCursor(80, 55);
			display.print(MinPot);

			display.setCursor(102, 55);
			display.print(MaxPot);

			display.display();
#else

			display.setCursor(0, 10);
			display.print("Count PAS: ");
			display.print(counterPAS);

			display.setCursor(0, 19);
			display.print("RPM Ped: ");
			display.print(RpmPedaling);

			display.setCursor(0, 28);
			display.print("WriteCurr: ");
			display.print(outputUartCurrent);

			display.setCursor(0, 37);
			display.print("Motor Curr: ");
			display.print(UART.data.avgMotorCurrent);

			display.setCursor(0, 46);
			display.print("Drive %: ");
			display.print(Drive_0_100);

			display.setCursor(0, 55);
			display.print("ADCraw: ");
			display.print(ADCrawValue);

			display.setCursor(80, 55);
			display.print("Lvl: ");
			display.print(AssistLevel);

			display.display();
#endif
		}
		else if (pagina == 7)
		{
			display.clearDisplay();

			byte line1 = map(outputUartCurrent * 1000, 0, CurrentMax * 1000, 0, 127);
			byte line2 = map(Drive_0_100, 0, 100, 0, 127);

			display.fillRect(0, 0, line1, 3, WHITE);
			display.fillRect(0, 5, line2, 3, WHITE);

			display.setTextSize(1);

			display.setCursor(64, 36);
			display.print("WATT:");
			display.setTextSize(2);
			display.setCursor(64, 44);
			int watt = TensioneBatt * UART.data.avgInputCurrent;
			display.print(watt);

			display.setTextSize(1);
			display.setCursor(64, 10);
			display.print("AMP MOT:");
			display.setTextSize(2);
			display.setCursor(64, 18);
			display.printf("%3.1f", UART.data.avgMotorCurrent);

			display.setTextSize(1);
			display.setCursor(0, 10);
			display.print("AMP IN:");
			display.setTextSize(2);
			display.setCursor(0, 18);
			display.printf("%3.1f", UART.data.avgInputCurrent);

			display.setTextSize(1);
			display.setCursor(0, 36);
			display.print("VOLT IN:");
			display.setTextSize(2);
			display.setCursor(0, 44);
			display.printf("%3.1f", TensioneBatt);

			display.display();

		}
		else if (pagina == 8)
		{
			display.clearDisplay();
			display.setTextSize(0);

			//Barre batteria
			byte LivelloBatt;
			if (UART.data.avgInputCurrent < 1.0f)
			{
				LivelloBatt = (percentBatt) / 16; //(TensioneBatt - 30.0f + 2.4f) / 2.4f;		//map del valore da 30 a 42v su 5 barre (12/5=2.4)
			}
			display.drawFastVLine(0, 0, 32, WHITE);
			switch (LivelloBatt)
			{
			case 6:
				display.fillRect(0, 0, 6, 5, WHITE);
				display.setCursor(8, 0);
				display.print(percentBatt);
				display.fillRect(21, 0, 50, 5, WHITE);
				display.fillRect(0, 8, 30, 5, WHITE);
				display.fillRect(0, 16, 20, 5, WHITE);
				display.fillRect(0, 24, 20, 5, WHITE);
				display.fillRect(0, 32, 10, 5, WHITE);
				break;

			case 5:
				display.fillRect(0, 0, 6, 5, WHITE);
				display.setCursor(8, 0);
				display.print(percentBatt);
				display.fillRect(21, 0, 50, 5, WHITE);
				display.fillRect(0, 8, 30, 5, WHITE);
				display.fillRect(0, 16, 20, 5, WHITE);
				display.fillRect(0, 24, 20, 5, WHITE);
				display.fillRect(0, 32, 10, 5, WHITE);
				break;

			case 4:
				display.setCursor(8, 0);
				display.print(percentBatt);
				display.print("%");
				display.fillRect(0, 8, 30, 5, WHITE);
				display.fillRect(0, 16, 20, 5, WHITE);
				display.fillRect(0, 24, 20, 5, WHITE);
				display.fillRect(0, 32, 10, 5, WHITE);
				break;

			case 3:
				display.setCursor(8, 0);
				display.print(percentBatt);
				display.print("%");
				display.fillRect(0, 16, 20, 5, WHITE);
				display.fillRect(0, 24, 20, 5, WHITE);
				display.fillRect(0, 32, 10, 5, WHITE);
				break;

			case 2:
				display.setCursor(8, 0);
				display.print(percentBatt);
				display.print("%");
				display.fillRect(0, 24, 20, 5, WHITE);
				display.fillRect(0, 32, 10, 5, WHITE);
				break;

			case 1:
				display.setCursor(8, 0);
				display.print(percentBatt);
				display.print("%");
				display.fillRect(0, 32, 10, 5, WHITE);
				break;

			case 0:
				display.setCursor(8, 0);
				display.print(percentBatt);
				display.print("%");
				//display.printf("%3.1f", TensioneBatt);
				break;
			default:
				break;
			}
			display.fillCircle(63, 63, 66, BLACK);

			int Watt = UART.data.inpVoltage * UART.data.avgInputCurrent;

			//Elementi statici
			display.drawCircle(63, 63, 61, WHITE);
			display.drawCircle(63, 63, 63, WHITE);
			display.drawFastVLine(63, 1, 5, WHITE);
			display.drawCircle(63, 63, 28, WHITE);
			display.drawFastVLine(63, 36, 3, WHITE);
			display.setCursor(6, 56);
			display.print("0");
			display.setCursor(110, 56);
			display.print("15");
			display.setCursor(54, 16);
			display.print("Amp");
			display.setCursor(37, 56);
			display.print("0");
			display.setCursor(84, 56);
			display.print("5");
			display.setCursor(57, 45);
			display.print("hW");
#if TORQUE_SENSOR_EN == 0
			display.setCursor(92, 0);
			display.print("SENSOR");
			display.setCursor(110, 8);
			if (SensorON == true)
			{
				display.print("ON");
			}
			else
			{
				display.print("OFF");
			}
#else
			display.setCursor(110, 0);
			display.setTextSize(2);
			display.print(AssistLevel);
#endif

			//INDICATORI
			//Potenziometro
#if TORQUE_SENSOR_EN == 0
			float radP = map(Drive_0_100, 0, 100, 0, 314);
			if (radP > 314) { radP = 314; }
			radP = radP / 100.0f;
			drawIndicator(radP, 62, 52, 0.07);
			//#else
			//			float radP = map(AssistLevel, 0, QTY_ASSIST_LEVEL, 0, 314);
			//			if (radP > 314) { radP = 314; }
			//			radP = radP / 100.0f;
			//			drawIndicator(radP, 62, 52, 0.07);
#endif // TORQUE_SENSOR_EN == 0

			display.setTextSize(0);
			//Ampere
			float radA = map((UART.data.avgMotorCurrent * 100.0f), 0, (CurrentMax * 100.0f), 0, 314); //map(percentPot, 0, 100, 0, 314);
			radA = radA / 100.0f;
			drawIndicator(radA, 62, 34, 0.05);

			//Potenza
			float radPwr = map((Watt), 0, 500, 0, 314);
			radPwr = radPwr / 100.0f;
			drawIndicator(radPwr, 26, 2, 2.2);

			display.setTextSize(0);
			if (Watt > 0)
			{
				display.fillRect(55, 45, 20, 10, BLACK);
				display.setCursor(56, 46);
				display.print(Watt);
			}

			if (UART.data.avgMotorCurrent > 1)
			{
				display.fillRect(54, 15, 20, 10, BLACK);
				display.setCursor(55, 16);
				display.printf("%3.1f", UART.data.avgMotorCurrent);
			}

			display.display();
		}
		else if (pagina == 20) {
			display.clearDisplay();
			display.setTextSize(2);
			display.setCursor(20, 0);
			display.print("SENSORE");
			display.setCursor(50, 30);
			if (SensorON == true)
			{
				display.print("ON");
			}
			else
			{
				display.print("OFF");
			}
			display.display();
			delay(500);
			pagina = 8;
		}
		else if (pagina == 31) {

			display.clearDisplay();
			display.setTextSize(0);
			display.setCursor(20, 0);
			display.print("CALIBRAZIONE");

			display.setCursor(0, 15);
			display.print("MIN");
			display.setCursor(0, 25);
			display.print(MinPot);

			display.setCursor(40, 15);
			display.print("ATTUALE");
			display.setCursor(40, 25);
			display.print(FilteredADCvalue);

			display.setCursor(100, 15);
			display.print("MAX");
			display.setCursor(100, 25);
			display.print(MaxPot);

			if (timerPotCalibration.msPassed < 10000)
			{
				display.setCursor(8, 40);
				display.print("Porta il pot al");
				display.setCursor(5, 50);
				display.print("MINIMO entro ");
				display.print(10 - (timerPotCalibration.msPassed / 1000));
				display.print(" sec");
			}
			else
			{

				display.setCursor(8, 40);
				display.print("Porta il pot al");
				display.setCursor(5, 50);
				display.print("MASSIMO entro ");
				display.print(20 - (timerPotCalibration.msPassed / 1000));
				display.print(" sec");
			}
			display.display();


		}
		else if (pagina > 8 && pagina < 20) {	//da 20 in poi le pagine sono speciali
			pagina = 5;
		}
	}
}

void drawIndicator(float radP, byte a, byte a1, float P1) {

	//Vertice indicatore C
	float cC = a * sin(radP);
	float bC = a * cos(radP);
	byte xC = 63 - bC;
	byte yC = 63 - cC;

	//Vertice base A
	float cA = a1 * sin(radP + P1);
	float bA = a1 * cos(radP + P1);
	byte xA = 63 - bA;
	byte yA = 63 - cA;

	//Vertice base B
	float cB = a1 * sin(radP - P1);
	float bB = a1 * cos(radP - P1);
	byte xB = 63 - bB;
	byte yB = 63 - cB;

	display.fillTriangle(xC, yC, xA, yA, xB, yB, WHITE);	//triangolino indicatore
}

void potCalibrationRoutine(void) {
	delay(500);
	timerPotCalibration.startTimer();
	int temp = analogRead(pinPot);
	int tempMinPot;
	int tempMaxPot;
	bool flagbttx = false;
	while (!(timerPotCalibration.checkTimer()) && !flagbttx)
	{
		temp = analogRead(pinPot);	//controllo se � stato premuttoo il pulsante, in caso affermativo esco dal ciclo while
		if (temp > maxValueADC - 500)
		{
			flagbttx = true;
		}
		FilteredADCvalue = FilterADC(temp);
		if (timerPotCalibration.msPassed < 10000)
		{
			tempMinPot = FilteredADCvalue;
		}

		if (timerPotCalibration.msPassed > 10000)
		{
			MinPot = tempMinPot;	//trascorsi 10 sec, aggiorno gi� Minpot in modo che possa gi� essere visualizzato su display
			tempMaxPot = FilteredADCvalue;
		}
	}
	flagbttx = false;
	if (timerPotCalibration.checkTimer() == true)
	{
		MaxPot = tempMaxPot;
		EEPROM.writeInt(0, MinPot);
		EEPROM.writeInt(4, MaxPot);
		EEPROM.commit();
		delay(1500);
	}

	//Memorizzazione in eeprom
	timerPotCalibration.resetTimer();
	pagina = 8;
}

/*La seguente funzione rileva il verso e la frequenza della pedalata.
Il verso viene rilevato controllando il fonte di salita del sensore hall 2: se quest'ultimo avviene e il sensore 1 � gi� attivo, vuol dire che
il senso di pedalata � orario e quindi in trazione*/
void IRAM_ATTR RilevaPedalata() {
	counterPAS++;
#if TORQUE_SENSOR_EN == 0
	if (digitalRead(Hall1) == false) {
		flagHall = !flagHall;
	}
#endif

}
