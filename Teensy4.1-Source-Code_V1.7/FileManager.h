#ifndef FILEMANAGER_H_
#define FILEMANAGER_H_

#include <SD.h>
#include <SPI.h>
#include <time.h>

#define FORMAT_SPIFFS_IF_FAILED true
#define PATH_SIZE  32

class FileManagerClass {
private:
    bool verb = true;
    String curr_dir;
    String curr_file;
    int path_size = PATH_SIZE;
public:
    FileManagerClass();
    ~FileManagerClass();
    bool exist(const char * path);
    void listDir(File dir, uint8_t levels);
    void createDir(const char * path);
    void removeDir(const char * path);
    void readFile(const char * path);
    void createFile(const char * path);
    void writeFile(const char * path, String message);
    void appendFile(const char * path, String message);
    void renameFile(const char * path1, const char * path2);
    void deleteFile(const char * path);
    void testFileIO(const char * path);

    bool exist(String path);
    void listDir(String dirname, uint8_t levels);
    void createDir(String path);
    void removeDir(String path);
    void readFile(String path);
    void createFile(String path);
    void writeFile(String path, String message);
    void appendFile(String path, String message);
    void renameFile(String path1, String path2);
    void deleteFile(String path);
    void testFileIO(String path);

    void setVerboseMode(bool _verb);
    bool getVerboseMode();
    void setCurrentFile(String file);
    String getCurrentFile();
};

FileManagerClass::FileManagerClass() {}

FileManagerClass::~FileManagerClass() {}

void FileManagerClass::setVerboseMode(bool _verb) { verb = _verb; }
bool FileManagerClass::getVerboseMode() { return verb; }
void FileManagerClass::setCurrentFile(String file) { curr_file = file; }
String FileManagerClass::getCurrentFile() { return curr_file; }

bool FileManagerClass::exist(String path) {
    File root = SD.open("/");
    File file = root.openNextFile();
    while(file) {
        if (path == file.name())
            return true;
        file = root.openNextFile();
    }
    return false;
}

bool FileManagerClass::exist(const char * path) {
    File root = SD.open("/");
    File file = root.openNextFile();
    while(file) {
        if (!strcmp(file.name(), path))
            return true;
        file = root.openNextFile();
    }
    return false;
}

void FileManagerClass::listDir(String dirname, uint8_t levels) {
    listDir(dirname.c_str(), levels);
}

void FileManagerClass::listDir(File dir, uint8_t levels) {
    while (true) {
        File entry =  dir.openNextFile();
        if (! entry)    break;  // no hay mas archivos
        for (uint8_t i = 0; i < levels + 1; i++)    Serial.print('\t');
        Serial.print(entry.name());
        if (entry.isDirectory()) {
            Serial.println("/");
            listDir(entry, levels + 1);
        } else {    // los archivos tienen tamaño, los directorios no
            Serial.print("\t\t");
            Serial.println(entry.size(), DEC);
        }
        entry.close();
    }
}

void FileManagerClass::createDir(String path) {
    createDir(path.c_str());
}

void FileManagerClass::createDir(const char * path) {
    if (verb) Serial.printf("Creating Dir: %s\n", path);
    if(SD.mkdir(path)){
        if (verb) Serial.println("Dir created");
    } else {
        if (verb) Serial.println("mkdir failed");
    }
}

void FileManagerClass::removeDir(String path) {
    removeDir(path.c_str());
}

void FileManagerClass::removeDir(const char * path) {
    if (verb) Serial.printf("Removing Dir: %s\n", path);
    if(SD.rmdir(path)){
        if (verb) Serial.println("Dir removed");
    } else {
        if (verb) Serial.println("rmdir failed");
    }
}

void FileManagerClass::readFile(String path) {
    readFile(path.c_str());
}

void FileManagerClass::readFile(const char * path) {
    if (verb) Serial.printf("Reading file: %s\n", path);

    File file = SD.open(path, FILE_READ);
    if(!file){
        if (verb) Serial.println("Failed to open file for reading");
        return;
    }

    if (verb) Serial.println("Read from file: ");
    while (file.available()) {
        if (verb) Serial.write(file.read());
    }
    file.close();
}

void FileManagerClass::createFile(String path) {
    createFile(path.c_str());
}

void FileManagerClass::createFile(const char * path) {
    if (verb) Serial.printf("Writing file: %s\n", path);

    File file = SD.open(path, FILE_WRITE);
    if(!file){
        if (verb) Serial.println("Failed to open file");
        return;
    }
    if (verb) Serial.println("File created");
    file.close();
}

void FileManagerClass::writeFile(String path, String message) {
    writeFile(path.c_str(), message);
}

void FileManagerClass::writeFile(const char * path, String message) {
    if (verb) Serial.printf("Writing file: %s\n", path);

    File file = SD.open(path, FILE_WRITE);
    if(!file){
        if (verb) Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)) {
        if (verb) Serial.println("File written");
    } else {
        if (verb)
        if (message == "") Serial.println("File cleared");
        else Serial.println("Write failed");
    }
    file.close();
}

void FileManagerClass::appendFile(String path, String message) {
    appendFile(path.c_str(), message);
}

void FileManagerClass::appendFile(const char * path, String message) {
    if (verb) Serial.printf("Appending to file: %s\n", path);

    File file = SD.open(path, FILE_WRITE);
    if(!file){
        if (verb) Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        if (verb) Serial.println("Message appended");
    } else {
        if (verb) Serial.println("Append failed");
    }
    file.close();
}

void FileManagerClass::renameFile(String path1, String path2) {
    renameFile(path1.c_str(), path2.c_str());
}

void FileManagerClass::renameFile(const char * path1, const char * path2) {
    if (verb) Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (SD.rename(path1, path2)) {
        if (verb) Serial.println("File renamed");
    } else {
        if (verb) Serial.println("Rename failed");
    }
}

void FileManagerClass::deleteFile(String path) {
    deleteFile(path.c_str());
}

void FileManagerClass::deleteFile(const char * path) {
    if (verb) Serial.printf("Deleting file: %s\n", path);
    if(SD.remove(path)){
        if (verb) Serial.println("File deleted");
    } else {
        if (verb) Serial.println("Delete failed");
    }
}

void FileManagerClass::testFileIO(String path) {
    testFileIO(path.c_str());
}

void FileManagerClass::testFileIO(const char * path) {
    if (verb) Serial.printf("Testing file I/O with %s\r\n", path);

    static uint8_t buf[512];
    size_t len = 0;
    File file = SD.open(path, FILE_WRITE);
    if(!file){
        if (verb) Serial.println("- failed to open file for writing");
        return;
    }

    size_t i;
    if (verb) Serial.print("- writing" );
    uint32_t start = millis();
    for(i=0; i<2048; i++){
        if ((i & 0x001F) == 0x001F){
          if (verb) Serial.print(".");
        }
        file.write(buf, 512);
    }
    if (verb) Serial.println("");
    uint32_t end = millis() - start;
    if (verb) Serial.printf(" - %u bytes written in %u ms\r\n", 2048 * 512, end);
    file.close();

    file = SD.open(path);
    start = millis();
    end = start;
    i = 0;
    if(file && !file.isDirectory()){
        len = file.size();
        size_t flen = len;
        start = millis();
        if (verb) Serial.print("- reading" );
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            if ((i++ & 0x001F) == 0x001F){
              if (verb) Serial.print(".");
            }
            len -= toRead;
        }
        if (verb) Serial.println("");
        end = millis() - start;
        if (verb) Serial.printf("- %u bytes read in %u ms\r\n", flen, end);
        file.close();
    } else {
        if (verb) Serial.println("- failed to open file for reading");
    }
}


FileManagerClass FileManager;


#endif  /* FILEMANAGER_H_ */
