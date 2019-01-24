#include "MIDIUSB.h"

#define PIN_FADER9 A6

//pin WR del MUX, latchea los selectores para extrar el valor de esa entrada por el drain
#define LATCH_PORT_BIT 7

//número de combinaciones para el MUX
#define NUM_MUX_INPUTS 32

//número de inputs analógicos
#define NUM_ANALOG_INPUTS 33

//número de inputs digitales
#define NUM_DIGITAL_INPUTS 4

//numero de muestras que se toman para hacer el promedio
#define NUM_FILTER_SAMPLES 4

//pin de entrada de la señal del MUX
#define MUX_DRAIN A0

//tiempo para el debounce de los botones
#define DEBOUNCE_TIME 5

//notas C(do) en MIDI
#define NOTE_C0 0x18 // 24

//tipo de señal de control midi a utilizar(14-31 son reconocidas como asignables por el estándar MIDI)
#define MIDI_CC_START 13

//máscara para el PORTF
#define PORTF_MASK B10001100

//sentencia NOP para saltar una instrucción del microcontrolador
#define NOP __asm__("nop")

//combinaciones posibles de los selectores del MUX
uint8_t muxInput[NUM_MUX_INPUTS] = {B00100011, B00100010, B00100001, B00100000, B00010011, B00010010, B00010001, B00010000,
                                    B00110011, B00110010, B00110001, B00110000, B00000011, B00000010, B00000001, B00000000,
                                    B01110011, B01110010, B01110001, B01110000, B01000011, B01000010, B01000001, B01000000,
                                    B01100011, B01100010, B01100001, B01100000, B01010011, B01010010, B01010001, B01010000};

//canales a asignar a cada potenciómetro del MUX
uint8_t channel[NUM_MUX_INPUTS] = {0, 1, 2, 3, 4, 5, 6, 7,
                                   0, 1, 2, 3, 4, 5, 6, 7,
                                   0, 1, 2, 3, 4, 5, 6, 7,
                                   0, 1, 2, 3, 4, 5, 6, 7};
                                  
//tipo de mensaje MIDI a asignar a cada potenciómetro del MUX
uint8_t control[NUM_MUX_INPUTS] = {13, 13, 13, 13, 13, 13, 13, 13,
                                   14, 14, 14, 14, 14, 14, 14, 14,
                                   15, 15, 15, 15, 15, 15, 15, 15,
                                   16, 16, 16, 16, 16, 16, 16, 16};

uint8_t buttonBitNumber[NUM_DIGITAL_INPUTS] = {1, 2, 3, 5};

/*
  estructura creada para guardar el valor actual de un control y sus valores previos.
  se usará para saber si el usuario movió el potenciómetro
  también almacenará que canal usará y que tipo de mensaje enviará cada control
*/
struct pot_t {
  uint8_t lastValue;
  uint8_t channel;
  uint8_t control;
} analogInputs[NUM_ANALOG_INPUTS];

/*
  estructura creada para guardar el ultimo valor de un boton
  también almacenará que bit del puerto del microcontrolador utiliza
  servirá para saber cuando el usuario presiona los botones
*/
struct button_t{
  bool value;
  uint8_t portBit;
} digitalInputs[NUM_DIGITAL_INPUTS];


//metodos de la librería
void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}

void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

void updateAnalogValues() {
  static int i = 1;
  int16_t actualValue;

  if (i >= NUM_ANALOG_INPUTS) {
    i = 1;

  }

  //condición para poder enviar el estado del fader que no pasa por el MUX
  if (i == 32) {
    actualValue = analogRead(PIN_FADER9) >> 3;

    if ((average - actualValue) > 1 || (average - actualValue) < -1) {
      controlChange(8, 14, 127 - actualValue);
      analogInputs[i].lastValue = actualValue;
    }

    i++;

    return;
  }


  PORTC = ~(1 << LATCH_PORT_BIT) & PORTC;   //setea el pin WR del MUX en LOW
  NOP;

  PORTF = muxInput[i];  //busca la combinación del MUX y la selecciona

  NOP;
  PORTC = (1 << LATCH_PORT_BIT) | PORTC;    //setea el pin WR del MUX en HIGH
  

  actualValue = (analogRead(MUX_DRAIN) >> 3);   //lee el valor del potenciómetro seleccionado y lo reduce a 7 bits

  //si la muestra obtenida es diferente se envía un mensaje MIDI con el nuevo valor y se almacena el mismo como último valor guardado

  if((analogInputs.lastValue - actualValue) > 1 || (analogInputs.lastValue - actualValue) < -1){
    controlChange(analogInputs[i].channel, analogInputs[i].control, 127 - actualValue);
    analogInputs[i].lastValue = actualValue;
  }

  // if (average != actualValue) {
  //   controlChange(analogInputs[i].channel, analogInputs[i].control, 127 - average);
  //   analogInputs[i].lastValues[j] = actualValue;
  // }

  i++;
}

void updateDigitalValues() {
  static int i = 0;
  static unsigned long counter = 0;
  static unsigned int waitTime = 0;

  if (i >= NUM_DIGITAL_INPUTS){
    i = 0;
  }

  if (millis() - counter < waitTime){
    return;
  }

  counter = millis();  

  bool buttonState = (PIND & (1 << digitalInputs[i].portBit)) >> digitalInputs[i].portBit;    //lee valor actual del pin del pulsador

  if (digitalInputs[i].value != buttonState) {
    
    digitalInputs[i].value = buttonState;

    if(digitalInputs[i].value == 1){
      noteOff(i, NOTE_C0 + i, 127);
    }else if(digitalInputs[i].value == 0){
      noteOn(i, NOTE_C0 + i, 127); 
    }     
    
    waitTime = DEBOUNCE_TIME;
  }

  i++;
}

void setup() {

  DDRF = B01110011;   //PORTF utilizado para el MUX

  DDRD = B00000000;   //PORTD utilizado para los pulsadores y F9
  PORTD = B11000001;  //setea pullups internos a pines no utilizados para evitar ruido

  DDRB = B00000000;   //MISO, MOSI, CLK (bit 3, 2 y 1 respectivamente)
  PORTB = B11111111;

  DDRE = B00000000;
  PORTE = B01000100;

  DDRC = B10000000;   //MUX WR(bit 7)
  PORTC = B01000000;

  //asigna por única vez los canales y mensajes de cada potenciómetro
  for (int i = 0; i < NUM_MUX_INPUTS; i++) {
    analogInputs[i].channel = channel[i];
    analogInputs[i].control = control[i];
  }

  for (int i = 0; i < NUM_DIGITAL_INPUTS; i++) {
    digitalInputs[i].portBit = buttonBitNumber[i];
  }

}

void loop() {
  updateAnalogValues();
  MidiUSB.flush();            //fuerza a enviar el mensaje MIDI de inmediato
  updateDigitalValues();
  MidiUSB.flush();
}
