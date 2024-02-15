#include "SimpleSerialParser.hpp"

SimpleSerialParser::SimpleSerialParser()
{
    int index = 0;
}

bool SimpleSerialParser::serialCheck()
{
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n')
        {
            buffer[index] = '\0';
            index = 0;
            return true;
        }
        else
        {
            buffer[index++] = c;
        }
    }
    return false;
}

ParsedSerialData SimpleSerialParser::parseSerialData()
{
    char dataType[10];
    char rawData[15];
    ParsedSerialData parsedSerialData;
    parsedSerialData.command = buffer[0];
    parsedSerialData.count = 0;
    int i = 1;
    int dataTypeIndex = 0;
    int rawDataIndex = 0;
    bool gettingDataType = true;
    while (buffer[i] != '\0')
    {
        if (gettingDataType)
        {
            if (buffer[i] == ':')
            {
                gettingDataType = false;
                dataTypeIndex = 0;
            }
            else
                dataType[dataTypeIndex++] = buffer[i];
        }
        else
        {
            if (buffer[i] == ';')
            {
                parsedSerialData.data[parsedSerialData.count].type = dataType[parsedSerialData.count];
                parsedSerialData.data[parsedSerialData.count].rawData = String(rawData);
                if (dataType[parsedSerialData.count] == 'i')
                    parsedSerialData.data[parsedSerialData.count].intData = atoi(rawData);
                else if (dataType[parsedSerialData.count] == 'f')
                    parsedSerialData.data[parsedSerialData.count].floatData = atof(rawData);
                rawDataIndex = 0;
                parsedSerialData.count++;
            }
            else
                rawData[rawDataIndex++] = buffer[i];
        }
        i++;
    }
    return parsedSerialData;
}