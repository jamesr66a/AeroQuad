#ifndef SD_WRITE_H
#define SD_WRITE_H

#include "SdFat.h"
#include "SdFatUtil.h"

//writes CR and LF to a file 
void writeCRLF(SdFile& f){
	f.write((uint8_t*)"\r\n", 2);
}

//writes and unsigned number to a file
void writeNumber(SdFile& f, uint32_t n){
  uint8_t buf[10];
  uint8_t i = 0;
  do {
    i++;
    buf[sizeof(buf) - i] = n%10 + '0';
    n /= 10;
  } while (n);
  f.write(&buf[sizeof(buf) - i], i);

}

//writes a string to a file 
void writeString(SdFile& f, char *str) {
  uint8_t n;
  for (n = 0; str[n]; n++);
  f.write((uint8_t *)str, n);
}

void appendInit(Sd2Card& card, SdVolume& volume, SdFile& root, SdFile& file){

  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  if (!card.init(SPI_HALF_SPEED)) Serial.print("card.init failed");

  // initialize a FAT volume
  if (!volume.init(&card)) Serial.print("volume.init failed");

  // open the root directory
  if (!root.openRoot(&volume)) Serial.print("openRoot failed");

  file.writeError=false;

  for (uint8_t i = 0; i < 100; i++) {
  // O_CREAT - create the file if it does not exist
  // O_APPEND - seek to the end of the file prior to each write
  // O_WRITE - open for write
  if (!file.open(&root, "file", O_CREAT | O_APPEND | O_WRITE)) {
    Serial.print("open failed");
  }

}

#endif // SD_WRITE_H
