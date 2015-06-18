const uint16_t MAXCAP=2048;

/*
T=0 - 1855072 Hz
T=1 -  618357 Hz
T=2 -  380386 Hz
T=3 -  274604 Hz
T=4 -  214720 Hz
T=5 -  176369 Hz
*/
const uint16_t T=1;
uint8_t data[MAXCAP];

void setup() {
  // Paint memory
  mem_paint();
  
  // Disable ADC
  ADCSRA &= ~(_BV(ADEN));
  // Disable Analog Comparator
  ADCSRB &= ~(_BV(ACME));

  // Set PF[7-4] as input with pullups
  DDRF  &= 0x0F;
  PORTF |= 0xF0;
  
  // Initialize data
  for(int i=0; i<MAXCAP; i++)
    data[i]=0;
  
  // Open serial port
  Serial.begin(9600);
}

void loop() {
  menu_loop();
}

//#############
//# Mem Trace #
//#############
extern uint8_t __data_start;
extern uint8_t __data_end;
extern uint8_t __bss_start;
extern uint8_t __bss_end;
extern uint8_t __heap_start;
extern uint8_t *__brkval;

const uint8_t STACK_CANARY=0xAA;

inline void mem_paint(void) {
  uint8_t *p = __brkval==0 ? &__heap_start : __brkval;
  
  while((uint16_t)p <= SP)
  {
      *p = STACK_CANARY;
      p++;
  }
}

uint16_t mem_unused() {
  uint8_t *p = __brkval==0 ? &__heap_start : __brkval;
  
  uint16_t c = 0;
  uint16_t maxc = 0;
  // Search max contiguous bytes with value STACK_CANARY
  while(p <= (uint8_t*)SP) {
    if(*p == STACK_CANARY) {
      c++;
    } else if(c > maxc) {
      maxc = c;
    }
    p++;
  }
  
  return maxc;
}

//########
//# Menu #
//########
inline void menu_loop() {
  while(Serial.available()) {
    char cmd = Serial.read();
    switch(cmd) {
      case 'h':
        menu_help();
        break;
      case 'm':
        menu_mem();
        break;
      case 'c':
        menu_capture();
        break;
      case 's':
        menu_show();
        break;
      default:
        Serial.print(F("Unknown command: "));
        Serial.println(cmd);
        break;
    }
  }
}

inline void menu_help() {
  Serial.println();
  Serial.println(F("SimpleProtocolAnalyzer 1.0"));
  Serial.println();
  Serial.println(F("Commands:"));
  Serial.println(F(" h Show this help"));
  Serial.println(F(" m Show mem stats"));
  Serial.println(F(" c Start capture data"));
  Serial.println(F(" s Show captured data"));
  Serial.println();
}

inline void menu_mem() {
  uint16_t total_size = RAMEND-((uint16_t)&__data_start)+1;
  uint16_t data_size  = ((uint16_t)&__data_end)-((uint16_t)&__data_start);
  uint16_t bss_size   = ((uint16_t)&__bss_end)-((uint16_t)&__bss_start);
  uint16_t heap_size  = ((uint16_t)__brkval) == 0 ? 0 : (((uint16_t)__brkval)-((uint16_t)&__heap_start));
  uint16_t stack_size = RAMEND-SP;
  uint16_t total_used = data_size+bss_size+heap_size+stack_size;
  Serial.print(F("Total:  "));
  Serial.println(total_size);
  Serial.print(F("Data:   "));
  Serial.println(data_size);
  Serial.print(F("BSS:    "));
  Serial.println(bss_size);
  Serial.print(F("Heap:   "));
  Serial.println(heap_size);
  Serial.print(F("Stack:  "));
  Serial.println(stack_size);
  Serial.print(F("Free:   "));
  Serial.println(RAMEND-total_used);
  Serial.print(F("Unused: "));
  Serial.println(mem_unused());
  Serial.println();
}

inline void menu_capture() {
  uint8_t ini = (PINF & 0xF0);
  // Wait some change
  while(ini == (PINF & 0xF0));
  
  unsigned long start = micros();
  for(int i=0;i<MAXCAP;i++) {
    data[i]  = 0;
    // Read Low nibble
    data[i] |= (PINF & 0xF0) >> 4;
    _delay_us(T);
    // Read High nibble
    data[i] |= (PINF & 0xF0);
     _delay_us(T);
  }
  unsigned long end = micros();
  Serial.print(F("Buffer filled in: "));
  Serial.print(end-start);
  Serial.println(F(" us"));
  Serial.print(F("Freq: "));
  Serial.print((4096000000)/(end-start));
  Serial.println(F(" Hz"));
}

const char GRAPH[] = {'.','+'};
const uint16_t LINESIZE = 64;

inline void menu_show() {
  int i=0;
  while(i<MAXCAP) {
    // Signal0
    for(uint8_t s=0;s<4;s++) {
      for(int j=0;j<LINESIZE;j++) {
        Serial.print(GRAPH[(data[i+j]&(0x01 << s)) == 0 ? 0 : 1]);
        Serial.print(GRAPH[(data[i+j]&(0x10 << s)) == 0 ? 0 : 1]);
      }
      Serial.println();
    }
    i += LINESIZE;
    Serial.println();
  }
}
