#include <Adafruit_NeoPixel.h>

#define N_PIXELS  120
#define LED_PIN   7
#define MAX_FREQ  1000.0

Adafruit_NeoPixel
	strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

const byte COLOR_DECAY_RATE = 20;
const byte COLOR_BIN_OFFSET = 7; // Start at this bin
const byte COLOR_BIN_COUNT = 28; // Bins to convert to color vlaues
byte colorArray[COLOR_BIN_COUNT];

#include <avr/wdt.h>

// Sample rate in samples per second.
// This will be rounded so the sample interval
// is a multiple of the ADC clock period.
const uint32_t SAMPLE_RATE = 5000;
// Desired sample interval in CPU cycles (will be adjusted to ADC/timer1 period)
const uint32_t SAMPLE_INTERVAL = F_CPU/SAMPLE_RATE;
// Minimum ADC clock cycles per sample interval
// const uint16_t MIN_ADC_CYCLES = 15;
const uint8_t ANALOG_PIN = 0;
// Reference voltage
// Zero - use External Reference AREF
// (1 << REFS0) - Use Vcc
// (1 << REFS1) | (1 << REFS0) - use Internal 1.1V Reference
uint8_t const ADC_REF_AVCC = 0;

// bin values less than this return 0
const uint8_t VOLUME_MIN_CUT_OFF = 80.0;
// bin values more than this are clipped
const uint8_t VOLUME_MAX_CUT_OFF = 600.0;

void adcInit(uint32_t ticks, uint8_t pin, uint8_t ref);
void adcStart();

uint16_t acquire();
uint16_t findMax(uint8_t arr[], int n);
uint16_t findMax(uint16_t arr[], int n);


#define LIN_OUT 1 // use the log output function
#define FHT_N 256 // set to 256 point fht

#include <FHT.h> // include the library

unsigned long StartAcq = 0;
unsigned long EndAcq = 0;
unsigned long FftCount = 0;
unsigned long SampFreqAvg = 0;
unsigned long AvgFreq = 0;
unsigned long NumFreq = 0;

int BufPtr = 0;


boolean TogState = false;
boolean ErrorCond = false;

void setup()
{
	wdt_reset();
	Serial.begin(115200); // use the serial port
	adcInit(SAMPLE_INTERVAL, ANALOG_PIN, ADC_REF_AVCC);
	wdt_enable(WDTO_8S);

	strip.begin();

}


void loop()
{
	if (ErrorCond)
	{
		Serial.println("Error");
		return;
	}

	uint16_t freq = acquire();
	NumFreq++;
	AvgFreq = (AvgFreq * (NumFreq - 1) + freq) / NumFreq;

	if ((FftCount / 50) % 2 != TogState)
	{
		TogState = (FftCount / 50) % 2;
		NumFreq = 0;
	}

	for(byte i = 0; i < N_PIXELS; i++) {
		// Need to fix the last set of four pixels
		strip.setPixelColor(i, strip.Color(0, colorArray[i/4]/.5, colorArray[i/4]));
	}

	strip.show(); // Update strip

	wdt_reset();

	Serial.print("\tAverage frequency: ");
	Serial.println(AvgFreq);

}



uint16_t acquire()
{
	uint16_t fs, binNum, freq;

	adcStart();
	BufPtr = 0;

	StartAcq = micros();
	while(BufPtr < FHT_N)
	{
		millis();
	}
	EndAcq = micros();

	FftCount++;
	SampFreqAvg = (SampFreqAvg * (FftCount - 1) + (EndAcq - StartAcq)) / FftCount;

	fs = 1.0 / ((double)(EndAcq - StartAcq) / 1000000.0 / FHT_N);

	// process data
	fht_window(); // window the data for better frequency response
	fht_reorder(); // reorder the data before doing the fht
	fht_run(); // process the data in the fht
	fht_mag_lin(); // take the output of the fht


	// find the loudest bin number
	binNum = findMax(fht_lin_out, FHT_N/2);

	int currentColor;

	for(byte x = 0; x < COLOR_BIN_COUNT; x++) {

		// convert bin volumn to byte for rgb
		currentColor = 255 * (fht_lin_out[x + COLOR_BIN_OFFSET] / VOLUME_MAX_CUT_OFF);

		// If we detect a peak update the color otherwise decay
		if(currentColor > colorArray[x]) {
			colorArray[x] = currentColor;
		} else if ((colorArray[x] - COLOR_DECAY_RATE) > COLOR_DECAY_RATE) {
			colorArray[x] = colorArray[x] - COLOR_DECAY_RATE;
		} else {
			colorArray[x] = 0;
		}

		// Serial.print(x + COLOR_BIN_OFFSET);
		// Serial.print(": ");
		// // Serial.print(fht_lin_out[x + COLOR_BIN_OFFSET]);
		// Serial.print(colorArray[x]);
		// Serial.print("\t");
	}


	Serial.print("max bin: ");
	Serial.print(binNum);

	// If the volume is too low return 0
	if(fht_lin_out[binNum] > VOLUME_MIN_CUT_OFF){
		freq = binNum * (fs / FHT_N);
	} else {
		freq = 0;
	}

	return freq;
}

uint16_t findMax(uint8_t arr[], int n)
{
	uint16_t m = 0;
	uint8_t val = 0;
	for (int i = 0; i < n; i++)
	{
		if (arr[i] > val)
		{
			m = i;
			val = arr[i];
		}
	}
	return m;
}

uint16_t findMax(uint16_t arr[], int n)
{
	uint16_t m = 0;
	uint16_t val = 0;
	for (int i = 0; i < n; i++)
	{
		if (arr[i] > val)
		{
			m = i;
			val = arr[i];
		}
	}
	return m;
}

void adcInit(uint32_t ticks, uint8_t pin, uint8_t ref) {
	if (ref & ~((1 << REFS0) | (1 << REFS1))) {
		//error("Invalid ADC reference bits");
		ErrorCond = true;
		return;
	}
	// Set ADC reference and low three bits of analog pin number
	ADMUX = ref | (pin & 7);
	#if RECORD_EIGHT_BITS
		// Left adjust ADC result to allow easy 8 bit reading
		ADMUX |= (1 << ADLAR);
	#endif  // RECORD_EIGHT_BITS

 	// trigger on timer/counter 1 compare match B
	ADCSRB = (1 << ADTS2) | (1 << ADTS0);

	// not a Mega disable Digital input buffer
	if (pin < 6) DIDR0 |= 1 << pin;

	#if ADPS0 != 0 || ADPS1 != 1 || ADPS2 != 2
		#error unexpected ADC prescaler bits
	#endif

	uint8_t adps;  // prescaler bits for ADCSRA
	// for (adps = 7; adps > 0; adps--) {
	//  if (ticks >= (MIN_ADC_CYCLES << adps)) break;
	// }

	// 2 is too low, 8 is too high
	adps = 3;

	if (adps < 3)
	{
		Serial.println("Sample Rate Too High");
		ErrorCond = true;
		return;
	}

	Serial.print("ADC clock MHz: ");
	Serial.println((F_CPU >> adps)*1.0e-6, 3);

	// set ADC prescaler
	ADCSRA = adps;

	// round so interval is multiple of ADC clock
	ticks >>= adps;
	ticks <<= adps;

	// Setup timer1
	// no pwm
	TCCR1A = 0;

	uint8_t tshift;
	Serial.print("Ticks: ");
	Serial.println(ticks);
	// if (ticks < 0X10000) {

	// no prescale, CTC mode
		TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
		tshift = 0;

	// } else if (ticks < 0X10000*8) {
	//   // prescale 8, CTC mode
		// TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
		// tshift = 3;
	// } else if (ticks < 0X10000*64) {
	//   // prescale 64, CTC mode
	//   TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
	//   tshift = 6;
	// } else if (ticks < 0X10000*256) {
	//   // prescale 256, CTC mode
	//   TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);
	//   tshift = 8;
	// } else if (ticks < 0X10000*1024) {
	//   // prescale 1024, CTC mode
	//   TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12) | (1 << CS10);
	//   tshift = 10;
	// } else {
	//   Serial.println("Sample Rate Too Slow");
	//   ErrorCond = true;
	//   return;
	// }


	// divide by prescaler
	ticks >>= tshift;
	// set TOP for timer reset
	ICR1 = ticks - 1;
	// compare for ADC start
	OCR1B = 0;

	// multiply by prescaler
	ticks <<= tshift;
	Serial.print("Sample interval usec: ");
	Serial.println(ticks*1000000.0/F_CPU);
	Serial.print("Sample Rate: ");
	Serial.println((float)F_CPU/ticks);
}


void adcStart() {
	// Enable ADC, Auto trigger mode, Enable ADC Interrupt, Start A2D Conversions
	ADCSRA |= (1 << ADATE)  | (1 << ADEN) | (1 << ADIE) | (1 << ADSC) ;
	// enable timer1 interrupts
	TIMSK1 = (1 <<OCIE1B);
	TCNT1 = 0;
}




// ADC done interrupt
ISR(ADC_vect) {
	//Serial.println("interupt!");
	// read ADC
#if RECORD_EIGHT_BITS
	uint8_t d = ADCH;
#else  // RECORD_EIGHT_BITS
	uint8_t low = ADCL;
	uint8_t high = ADCH;
	uint16_t d = (high << 8) | low;
#endif  // RECORD_EIGHT_BITS

	int k = d - 0x0200; // form into a signed int
	k <<= 6; // form into a 16b signed int

	// Only write to the buffer if it's not full.
	if (BufPtr < FHT_N)
	{
		fht_input[BufPtr] = k;
	}

	BufPtr++;
}

ISR(TIMER1_COMPB_vect) {}


// Input a value 0 to 255 to get a color value.
// The colors are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
	if(WheelPos < 85) {
		return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
	} else if(WheelPos < 170) {
		WheelPos -= 85;
	 	return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
	} else {
	 	WheelPos -= 170;
	 	return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
	}
}