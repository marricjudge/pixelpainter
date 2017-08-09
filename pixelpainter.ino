//This code is based on the Adafruit Neopixel Painter Tutorial https://learn.adafruit.com/neopixel-painter?view=all
//(Get the code here: https://github.com/adafruit/NeoPixel_Painter/archive/master.zip)
//and has been adapted and developed by Marius Richter during the creation process. A single seven segment display and two
//pushbuttons has been added to the sketch.
//Video: https://youtu.be/94cXyfNd3o4

//Last updated 09th August 2017 by Marius Richter

#include <arduino.h>
#include <SdFat.h> //SD-Library https://github.com/greiman/SdFat
#include <avr/pgmspace.h>
#include "./gamma.h"

// CONFIGURABLE STUFF --------------------------------------------------------

#define N_LEDS       170 // Number of LEDs in one row
#define CARD_SELECT   10 // SD card select pin
#define LED_PIN       10 // PIN where Neopixels are connected to the Arduino
#define SPEED         A0 // Speed-setting dial
#define BRIGHTNESS    A0 // Brightness-setting dial
#define BUTTONREPEATPIN  13 // Button to play and repeat the current image
#define BUTTONNEXTPIN    A1 // Button for next image
#define BUTTONPREVPIN    A2 // Button for previous image
#define CURRENT_MAX 3500 // Max current from power supply (mA)

int a = 2;  //For displaying segment "a"
int h = 3;  //For displaying segment "h" Segement B is called H because the variable b is for brightness
int c = 4;  //For displaying segment "c"
int d = 5;  //For displaying segment "d"
int e = 6;  //For displaying segment "e"
int f = 7;  //For displaying segment "f"
int g = 8;  //For displaying segment "g"

#define loadingDelay 100 // speed of the loading cycle of the display while running setup loop


// NON-CONFIGURABLE STUFF ----------------------------------------------------

#define OVERHEAD 150 // Extra microseconds for loop processing, etc.

uint8_t           sdBuf[512],  // One SD block (also for NeoPixel color data)
                  pinMask;     // NeoPixel pin bitmask
uint16_t          maxLPS,      // Max playback lines/sec
                  nFrames = 0, // Total # of image files
                  frame   = 0; // Current image # being painted
uint32_t          firstBlock,  // First block # in temp working file
                  nBlocks;     // Number of blocks in file
SdFat             sd;          // SD filesystem
volatile uint8_t *port;        // NeoPixel PORT register

boolean bmpProcess(char *inName, char *outName, uint8_t *brightness);

// INITIALIZATION ------------------------------------------------------------

void setup() {
  uint8_t  b, buttonRepeat, buttonNext, buttonPrev, minBrightness;
  char     infile[13], outfile[13];
  boolean  found;
  uint16_t i, n;
  SdFile   tmp;
  uint32_t lastBlock;

  pinMode(a, OUTPUT);  //A
  pinMode(h, OUTPUT);  //Segement B is called H because the variable b is used to relate to the brightness
  pinMode(c, OUTPUT);  //C
  pinMode(d, OUTPUT);  //D
  pinMode(e, OUTPUT);  //E
  pinMode(f, OUTPUT);  //F
  pinMode(g, OUTPUT);  //G

  displayCycle(); //show the cycling on the seven-segment-display

  pinMode(BUTTONREPEATPIN, INPUT);
  pinMode(BUTTONNEXTPIN, INPUT);
  pinMode(BUTTONPREVPIN, INPUT);

  digitalWrite(BUTTONREPEATPIN, LOW);           // Enable pullup on BUTTONREPEAT button
  buttonRepeat = digitalRead(BUTTONREPEATPIN); // Poll startup BUTTONREPEAT ASAP

  digitalWrite(BUTTONNEXTPIN, LOW);           // Button for next image
  buttonNext = digitalRead(BUTTONNEXTPIN); 

  digitalWrite(BUTTONPREVPIN, LOW);           // Button for prev image
  buttonPrev = digitalRead(BUTTONPREVPIN);

  Serial.begin(57600);
  pinMode(LED_PIN, OUTPUT);              // Enable NeoPixel output
  digitalWrite(LED_PIN, LOW);            // Default logic state = low
  port    = portOutputRegister(digitalPinToPort(LED_PIN));
  pinMask = digitalPinToBitMask(LED_PIN);
  memset(sdBuf, 0, N_LEDS * 3);          // Clear LED buffer
  show();                                // Init LEDs to 'off' state
#ifdef ENCODERSTEPS
  digitalWrite(5, HIGH);                 // Enable pullup on encoder pin
#endif

  Serial.print(F("Initializing SD card..."));
  if(!sd.begin(CARD_SELECT, SPI_FULL_SPEED)) {
    error(F("failed. Things to check:\n"
            "* is a card is inserted?\n"
            "* Is your wiring correct?\n"
            "* did you edit CARD_SELECT to match the SD shield or module?"));
  }
  Serial.println(F("OK"));

  displayCycle();
  
#if 0
  if(!volume.init(&card)) {
    error(F("Could not find FAT16/FAT32 partition.\n"
            "Make sure the card is formatted."));
  }
  root.openRoot(&volume);
#endif

  // This simple application always reads the files 'frameNNN.bmp' in
  // the root directory

  if(buttonRepeat == LOW) { // No button press

    // Two passes are made over the input images.  First pass counts the
    // files and estimates the max brightness level that the power supply
    // can sustain...
    
    minBrightness = 255; 
    do {
      sprintf(infile, "frame%03d.bmp", nFrames);
      b = 255; // Assume frame at full brightness to start...
      // ...it's then modified by the bmpProcess() function here to the
      // actual brightness limit the UBEC can sustain for *this image*.
      if(found = bmpProcess(infile, NULL, &b)) {
        nFrames++;
        // Keep track of the minimum 'b' value returned for *all images*.
        // This will later be applied to all, in order that multi-frame
        // animations can have consistent brightness throughout.
        if(b < minBrightness) minBrightness = b;
      }
    } while(found && (nFrames < 1000));
    
    displayCycle();
    Serial.print(nFrames);
    Serial.print(" frames\nbrightness = ");
    Serial.println(minBrightness);

    // Read dial, setting brightness between 1 (almost but not quite off)
    // and the previously-estimated safe max.
    minBrightness = map(analogRead(BRIGHTNESS), 0, 1023, 1, minBrightness);
  
    // Second pass now applies brightness adjustment while converting
    // the image(s) from BMP to a raw representation of NeoPixel data
    // (this outputs the file(s) 'frameNNN.tmp'
    for(i=0; i<nFrames; i++) {
      sprintf(infile , "frame%03d.bmp", i);
      sprintf(outfile, "frame%03d.tmp", i);
      b = minBrightness; // Reset b to safe limit on each loop iteration
      bmpProcess(infile, outfile, &b);
    }
    while(digitalRead(BUTTONREPEATPIN) == HIGH); // Wait for button release

  } else { // Button held -- use existing data

    do { // Scan for files to get nFrames
      sprintf(infile, "frame%03d.tmp", nFrames);
      if(found = tmp.open(infile, O_RDONLY)) {
        if(tmp.contiguousRange(&firstBlock, &lastBlock)) {
          nFrames++;
        }
        tmp.close();
      }
    } while(found);

  } // end buttonStart test
  displayCycle();

#ifdef ENCODERSTEPS
  // To use a rotary encoder rather than timer, connect one output
  // of encoder to T1 pin (digital pin 5 on Arduino Uno).  A small
  // capacitor across the encoder pins may help for debouncing.
  TCCR1A = _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12) | _BV(CS11);
#else
  // Prepare for playback from file; make a full read pass through the
  // file to estimate block read time (+5% margin) and max playback
  // lines/sec.  Not all SD cards perform the same.  This makes sure a
  // reasonable speed limit is used.
  for(i=0; i<nFrames; i++) { // Scan all files
    sprintf(infile, "frame%03d.tmp", i);
    tmp.open(infile, O_RDONLY);
    tmp.contiguousRange(&firstBlock, &lastBlock);
    nBlocks = tmp.fileSize() / 512;
    tmp.close();
    n = (uint16_t)(1000000L /                         // 1 uSec /
      (((benchmark(firstBlock, nBlocks) * 21) / 20) + // time + 5% +
       (N_LEDS * 30L) + OVERHEAD));                   // 30 uSec/pixel
    if(n > maxLPS) maxLPS = n;
  }
  if(maxLPS > 400) maxLPS = 400; // NeoPixel PWM rate is ~400 Hz
  Serial.print(F("Max lines/sec: "));
  Serial.println(maxLPS);

  // Set up Timer1 for 64:1 prescale (250 KHz clock source),
  // fast PWM mode, no PWM out.
  TCCR1A = _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  // Timer1 interrupt is not used; instead, overflow bit is tightly
  // polled.  Very infrequently a block read may inexplicably take longer
  // than normal...using an interrupt, the code would clobber itself.
  // By polling instead, worst case is just a minor glitch where the
  // corresponding row will be a little thicker than normal.
#endif

  // Timer0 interrupt is disabled for smoother playback.
  // This means delay(), millis(), etc. won't work after this.
  TIMSK0 = 0;
}

// Startup error handler; doesn't return, doesn't run loop(), just stops.
static void error(const __FlashStringHelper *ptr) {
    Serial.println(ptr); // Show message
    for(;;);             // and hang
}

// PLAYBACK LOOP -------------------------------------------------------------

void loop() {
  
  displayDigit(frame);
  Serial.print("Number send to display: ");
  Serial.println(frame);
  
  uint32_t block    = 0;     // Current block # within file
  boolean  stopFlag = false; // If set, stop playback loop
  uint32_t lastBlock;
  char     infile[13];
  SdFile   tmp;

  // Get existing contiguous tempfile info
  sprintf(infile, "frame%03d.tmp", frame);
  if(!tmp.open(infile, O_RDONLY)) {
    error(F("Could not open NeoPixel tempfile for input"));
  }
  if(!tmp.contiguousRange(&firstBlock, &lastBlock)) {
    error(F("NeoPixel tempfile is not contiguous"));
  }
  // Number of blocks needs to be calculated from file size, not the
  // range values.  The contiguous file creation and range functions
  // work on cluster boundaries, not necessarily the actual file size.
  nBlocks = tmp.fileSize() / 512;

  tmp.close(); // File handle is no longer accessed, just block reads

  // Stage first block, but don't display yet -- the loop below
  // does that only when Timer1 overflows.
  sd.card()->readBlock(firstBlock, sdBuf);
  // readBlock is used rather than readStart/readData/readEnd as
  // the duration between block reads may exceed the SD timeout.

  while(true) {
    delay(1000);
    displayDigit(frame);

    Serial.print("Whileloop: ");
    Serial.println(frame); //print the current frame number to the serial monitor

    if(digitalRead(BUTTONREPEATPIN) == HIGH) {  //if button next to the handrail is pressed the image will be displayed
      Serial.println("PLAY/REPEAT!");
      displayTurnOff();
      #ifdef ENCODERSTEPS
        // Set up for rotary encoder
        TCNT1 = 0;
        OCR1A = ENCODERSTEPS;
      #else
        // Set up timer based on dial input
        uint32_t linesPerSec = map(analogRead(SPEED), 0, 1023, 10, maxLPS);
        // Serial.println(linesPerSec);
        OCR1A = (F_CPU / 64) / linesPerSec;          // Timer1 interval
      #endif
      
        for(;;) {
          while(!(TIFR1 & _BV(TOV1)));               // Wait for Timer1 overflow
          TIFR1 |= _BV(TOV1);                        // Clear overflow bit
      
          show();                                    // Display current line
          if(stopFlag) break;                        // Break when done
      
          if(++block >= nBlocks) {                   // Past last block?
            if(digitalRead(BUTTONREPEATPIN) == LOW) {       // BUTTONREPEAT released?
              memset(sdBuf, 0, N_LEDS * 3);          // LEDs off on next pass
              stopFlag = true;                       // Stop playback on next pass
              continue;
            }                                        // Else BUTTONREPEAT still held
            block = 0;                               // Loop back to start
          }
          sd.card()->readBlock(block + firstBlock, sdBuf); // Load next pixel row
        }
      
      break;
      
    } else if(digitalRead(BUTTONNEXTPIN) == HIGH) {
      Serial.println("NEXT!");
      delay(1000);
      if(++frame >= nFrames) frame = 0;
      displayDigit(frame);
      
    } else if(digitalRead(BUTTONPREVPIN) == HIGH) {
      Serial.print("PREV");  
      delay(1000);
      if(--frame >= nFrames) frame = nFrames - 1;
      displayDigit(frame);
    }
  }   
}

// BMP->NEOPIXEL FILE CONVERSION ---------------------------------------------

#define BMP_BLUE  0 // BMP and NeoPixels have R/G/B color
#define BMP_GREEN 1 // components in different orders.
#define BMP_RED   2 // (BMP = BGR, Neo = GRB)
#define NEO_GREEN 0
#define NEO_RED   1
#define NEO_BLUE  2

// Convert file from 24-bit Windows BMP format to raw NeoPixel datastream.
// Conversion is bottom-to-top (see notes below)...for horizontal light
// painting, the image is NOT rotated here (the per-pixel file seeking this
// requires takes FOREVER on the Arduino).  Instead, such images should be
// rotated counterclockwise (in Photoshop or other editor) prior to moving
// to SD card.  As currently written, both the input and output files need
// to be in the same directory.  Brightness is set during conversion; there
// aren't enough cycles to do this in realtime during playback.  To change
// brightness, re-process image file using new brightness value.
boolean bmpProcess(
  char    *inName,
  char    *outName,
  uint8_t *brightness) {

  SdFile    inFile,              // Windows BMP file for input
            outFile;             // NeoPixel temp file for output
  boolean   ok        = false,   // 'true' on valid BMP & output file
            flip      = false;   // 'true' if image stored top-to-bottom
  int       bmpWidth,            // BMP width in pixels
            bmpHeight,           // BMP height in pixels
            bmpStartCol,         // First BMP column to process (crop/center)
            columns,             // Number of columns to process (crop/center)
            row,                 // Current image row (Y)
            column;              // and column (X)
  uint8_t  *ditherRow,           // 16-element dither array for current row
            pixel[3],            // For reordering color data, BGR to GRB
            b = 0,               // 1 + *brightness
            d,                   // Dither value for row/column
            color,               // Color component index (R/G/B)
            raw,                 // 'Raw' R/G/B color value
            corr,                // Gamma-corrected R/G/B
           *ledPtr,              // Pointer into sdBuf (output)
           *ledStartPtr;         // First LED column to process (crop/center)
  uint16_t  b16;                 // 16-bit dup of b
  uint32_t  bmpImageoffset,      // Start of image data in BMP file
            lineMax   = 0L,      // Cumulative brightness of brightest line
            rowSize,             // BMP row size (bytes) w/32-bit alignment
            sum,                 // Sum of pixels in row
            startTime = millis();

  if(brightness)           b = 1 + *brightness; // Wraps around, fun with maths
  else if(NULL == outName) return false; // MUST pass brightness for power est.

  Serial.print(F("Reading file '"));
  Serial.print(inName);
  Serial.print(F("'..."));
  if(!inFile.open(inName, O_RDONLY)) {
    Serial.println(F("error"));
    return false;
  }

  if(inFile.read(sdBuf, 34)             &&    // Load header
    (*(uint16_t *)&sdBuf[ 0] == 0x4D42) &&    // BMP signature
    (*(uint16_t *)&sdBuf[26] == 1)      &&    // Planes: must be 1
    (*(uint16_t *)&sdBuf[28] == 24)     &&    // Bits per pixel: must be 24
    (*(uint32_t *)&sdBuf[30] == 0)) {         // Compression: must be 0 (none)
    // Supported BMP format -- proceed!
    bmpImageoffset = *(uint32_t *)&sdBuf[10]; // Start of image data
    bmpWidth       = *(uint32_t *)&sdBuf[18]; // Image dimensions
    bmpHeight      = *(uint32_t *)&sdBuf[22];
    // That's some nonportable, endian-dependent code right there.

    Serial.print(bmpWidth);
    Serial.write('x');
    Serial.print(bmpHeight);
    Serial.println(F(" pixels"));

    if(outName) { // Doing conversion?  Need outFile.
      // Delete existing outFile file (if any)
      (void)sd.remove(outName);
      Serial.print(F("Creating contiguous file..."));
      // NeoPixel working file is always 512 bytes (one SD block) per row
      if(outFile.createContiguous(sd.vwd(), outName, 512L * bmpHeight)) {
        uint32_t lastBlock;
        outFile.contiguousRange(&firstBlock, &lastBlock);
        // Once we have the first block index, the file handle
        // is no longer needed -- raw block writes are used.
        outFile.close();
        nBlocks = bmpHeight; // See note in setup() re: block calcs
        ok      = true;      // outFile is good; proceed
        Serial.println(F("OK"));
      } else {
        Serial.println(F("error"));
      }
    } else ok = true; // outFile not needed; proceed

    if(ok) { // Valid BMP and contig file (if needed) are ready
      Serial.print(F("Processing..."));

      rowSize = ((bmpWidth * 3) + 3) & ~3; // 32-bit line boundary
      b16     = (int)b;

      if(bmpHeight < 0) {       // If bmpHeight is negative,
        bmpHeight = -bmpHeight; // image is in top-down order.
        flip      = true;       // Rare, but happens.
      }

      if(bmpWidth >= N_LEDS) { // BMP matches LED bar width, or crop image
        bmpStartCol = (bmpWidth - N_LEDS) / 2;
        ledStartPtr = sdBuf;
        columns     = N_LEDS;
      } else {                 // Center narrow image within LED bar
        bmpStartCol = 0;
        ledStartPtr = &sdBuf[((N_LEDS - bmpWidth) / 2) * 3];
        columns     = bmpWidth;
        memset(sdBuf, 0, N_LEDS * 3); // Clear left/right pixels
      }

      for(row=0; row<bmpHeight; row++) { // For each row in image...
        Serial.write('.');
        // Image is converted from bottom to top.  This is on purpose!
        // The ground (physical ground, not the electrical kind) provides
        // a uniform point of reference for multi-frame vertical painting...
        // could then use something like a leaf switch to BUTTONREPEAT playback,
        // lifting the light bar like making giant soap bubbles.

        // Seek to first pixel to load for this row...
        inFile.seekSet(
          bmpImageoffset + (bmpStartCol * 3) + (rowSize * (flip ?
          (bmpHeight - 1 - row) : // Image is stored top-to-bottom
          row)));                 // Image stored bottom-to-top
        if(!inFile.read(ledStartPtr, columns * 3))  // Load row
          Serial.println(F("Read error"));

        sum       = 0L;
        ditherRow = (uint8_t *)&dither[row & 0x0F]; // Dither values for row
        ledPtr    = ledStartPtr;
        for(column=0; column<columns; column++) {   // For each column...
          if(b) { // Scale brightness, reorder R/G/B
            pixel[NEO_BLUE]  = (ledPtr[BMP_BLUE]  * b16) >> 8;
            pixel[NEO_GREEN] = (ledPtr[BMP_GREEN] * b16) >> 8;
            pixel[NEO_RED]   = (ledPtr[BMP_RED]   * b16) >> 8;
          } else { // Full brightness, reorder R/G/B
            pixel[NEO_BLUE]  = ledPtr[BMP_BLUE];
            pixel[NEO_GREEN] = ledPtr[BMP_GREEN];
            pixel[NEO_RED]   = ledPtr[BMP_RED];
          }

          d = pgm_read_byte(&ditherRow[column & 0x0F]); // Dither probability
          for(color=0; color<3; color++) {              // 3 color bytes...
            raw  = pixel[color];                        // 'Raw' G/R/B
            corr = pgm_read_byte(&gamma[raw]);          // Gamma-corrected
            if(pgm_read_byte(&bump[raw]) > d) corr++;   // Dither up?
            *ledPtr++ = corr;                           // Store back in sdBuf
            sum      += corr;                           // Total brightness
          } // Next color byte
        } // Next column

        if(outName) {
          if(!sd.card()->writeBlock(firstBlock + row, (uint8_t *)sdBuf))
            Serial.println(F("Write error"));
        }
        if(sum > lineMax) lineMax = sum;

      } // Next row
      Serial.println(F("OK"));

      if(brightness) {
        lineMax = (lineMax * 20) / 255; // Est current @ ~20 mA/LED
        if(lineMax > CURRENT_MAX) {
          // Estimate suitable brightness based on CURRENT_MAX
          *brightness = (*brightness * (uint32_t)CURRENT_MAX) / lineMax;
        } // Else no recommended change
      }

      Serial.print(F("Processed in "));
      Serial.print(millis() - startTime);
      Serial.println(F(" ms"));

    } // end 'ok' check
  } else { // end BMP header check
    Serial.println(F("BMP format not recognized."));
  }

  inFile.close();
  return ok; // 'false' on various file open/create errors
}

// MISC UTILITY FUNCTIONS ----------------------------------------------------

// Estimate maximum block-read time for card (microseconds)
static uint32_t benchmark(uint32_t block, uint32_t n) {
  uint32_t t, maxTime = 0L;

  do {
    t = micros();
    sd.card()->readBlock(block++, sdBuf);
    if((t = (micros() - t)) > maxTime) maxTime = t;
  } while(--n);

  return maxTime;
}

// NEOPIXEL FUNCTIONS --------------------------------------------------------

static void show(void) {
  volatile uint16_t
    i   = N_LEDS * 3; // Loop counter
  volatile uint8_t
   *ptr = sdBuf,      // Pointer to next byte
    b   = *ptr++,     // Current byte value
    hi,               // PORT w/output bit set high
    lo,               // PORT w/output bit set low
    next,
    bit = 8;

  noInterrupts();
  hi   = *port |  pinMask;
  lo   = *port & ~pinMask;
  next = lo;

  asm volatile(
   "head20_%=:"                "\n\t"
    "st   %a[port],  %[hi]"    "\n\t"
    "sbrc %[byte],  7"         "\n\t"
     "mov  %[next], %[hi]"     "\n\t"
    "dec  %[bit]"              "\n\t"
    "st   %a[port],  %[next]"  "\n\t"
    "mov  %[next] ,  %[lo]"    "\n\t"
    "breq nextbyte20_%="       "\n\t"
    "rol  %[byte]"             "\n\t"
    "rjmp .+0"                 "\n\t"
    "nop"                      "\n\t"
    "st   %a[port],  %[lo]"    "\n\t"
    "nop"                      "\n\t"
    "rjmp .+0"                 "\n\t"
    "rjmp head20_%="           "\n\t"
   "nextbyte20_%=:"            "\n\t"
    "ldi  %[bit]  ,  8"        "\n\t"
    "ld   %[byte] ,  %a[ptr]+" "\n\t"
    "st   %a[port], %[lo]"     "\n\t"
    "nop"                      "\n\t"
    "sbiw %[count], 1"         "\n\t"
     "brne head20_%="          "\n"
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [bit]   "+r" (bit),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));

  interrupts();
  // There's no explicit 50 uS delay here as with most NeoPixel code;
  // SD card block read provides ample time for latch!
}

// SEVEN SEGMENT DISPLAY  -----------------------------------------------------------------------------

void displayDigit(int digit) {

  digitalWrite(a,LOW);
  digitalWrite(h,LOW);
  digitalWrite(c,LOW);
  digitalWrite(d,LOW);
  digitalWrite(e,LOW);
  digitalWrite(f,LOW);
  digitalWrite(g,LOW);

//Conditions for displaying segment a
 if(digit!=1 && digit != 4)
 digitalWrite(a,HIGH);
 
 //Conditions for displaying segment h
 if(digit != 5 && digit != 6)
 digitalWrite(h,HIGH);
 
 //Conditions for displaying segment c
 if(digit !=2)
 digitalWrite(c,HIGH);
 
 //Conditions for displaying segment d
 if(digit != 1 && digit !=4 && digit !=7)
 digitalWrite(d,HIGH);
 
 //Conditions for displaying segment e 
 if(digit == 2 || digit ==6 || digit == 8 || digit==0)
 digitalWrite(e,HIGH);
 
 //Conditions for displaying segment f
 if(digit != 1 && digit !=2 && digit!=3 && digit !=7)
 digitalWrite(f,HIGH);

 //Conditions for displaying segment g
 if (digit!=0 && digit!=1 && digit !=7)
 digitalWrite(g,HIGH);
}

void displayTurnOff(){ //Turn the whole display off
    digitalWrite(a,LOW);
    digitalWrite(h,LOW);
    digitalWrite(c,LOW);
    digitalWrite(d,LOW);
    digitalWrite(e,LOW);
    digitalWrite(f,LOW);
    digitalWrite(g,LOW);
}


void displayCycle (){ //send the loading cycle to the display, the delaytime "loadingDelay" is set a the very top of this code
    digitalWrite(a,LOW);
    digitalWrite(h,LOW);
    digitalWrite(c,LOW);
    digitalWrite(d,LOW);
    digitalWrite(e,LOW);
    digitalWrite(f,LOW);
    digitalWrite(g,LOW);
  
    digitalWrite(a,HIGH);
    delay(loadingDelay);
    digitalWrite(h,HIGH);
    delay(loadingDelay);
    digitalWrite(a,LOW);
    digitalWrite(c,HIGH);
    delay(loadingDelay);
    digitalWrite(h,LOW);
    digitalWrite(d,HIGH);
    delay(loadingDelay);
    digitalWrite(c,LOW);
    digitalWrite(e,HIGH);
    delay(loadingDelay);
    digitalWrite(d,LOW);
    digitalWrite(f,HIGH);
    delay(loadingDelay);
    digitalWrite(e,LOW);
    delay(loadingDelay);
    digitalWrite(a,HIGH);
    digitalWrite(f,LOW);
    delay(loadingDelay);
}

