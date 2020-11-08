#include <cstddef>

struct Writer;
void writeHeader(Writer& buffer);
void writeCsvHeader(Writer& outBuffer);
size_t writeBlock(char* buffer);
size_t writeToothBlock(char* buffer);
