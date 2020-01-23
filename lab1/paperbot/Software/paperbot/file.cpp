#include <Arduino.h>
#include "debug.h"

#include <FS.h>

/* This cpp file contain implemtation related to SPIFFS
 */

bool isFileSetup = false;

//initialize SPIFFS, a flash memory on the arduino
void setupFile() {
    DEBUG("Prepare file system");
    SPIFFS.begin();
    isFileSetup = true;
}

/* The function takes a file name as argument.
 * It opens the file and read all lines from the file and store them in text
 */
String loadFile(const char* filename) {
  // if not setup setup the file
    if (!isFileSetup)
        setupFile();

  // open the file
    File file = SPIFFS.open(filename, "r");
    if (!file) {
        DEBUG("File open failed");  
    } else {
        DEBUG("File open success");
    // read all lines from the file and store in text, read each line of the file and appened endline,
        String text = "";
        while (file.available()) {
            text += file.readStringUntil('\n');
            text += "\n";
        }
        file.close();
        return text;
    }
}
