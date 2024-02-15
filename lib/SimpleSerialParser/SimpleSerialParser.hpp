#pragma once
#include "Arduino.h"
/**
 * @brief Simple serial parser
 *
 * @details This class is used to parse serial data from serial port.
 *          The serial data terminates with a '\n' char.
 *          The first char of the data is the command char.
 *          After that it is the data type, i is int, f is float, s is string.
 *          The data type is followed by a colon ':'.
 *          The data is the last part of the serial data, each of the data is seperated by a semi colon ';'.
 *          Example of serial data: "ciii:1;2;3;\n"
 *          The above serial data means that the command is 'c', the data type is 'iii', and the data is 1, 2, 3.
 *          Another example of serial data: "cfs:3.4;hello;\n"
 *          Can handle up to maximum of 10 data or 64 bytes maximum.
 */

struct ParsedData
{
    String rawData;
    char type;
    int intData;
    double floatData;
};

struct ParsedSerialData
{
    char command;
    int count = 0;
    ParsedData data[10];
};

class SimpleSerialParser
{
private:
    char buffer[64];
    int index;

public:
    SimpleSerialParser();
    bool serialCheck();
    ParsedSerialData parseSerialData();
};