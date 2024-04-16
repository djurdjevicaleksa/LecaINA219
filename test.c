#include "lecaina219.h"
#include<stdio.h>
#include <unistd.h>

int main()
{
    LecaINA219 ina;
    ina219Init(&ina, 0x40, "/dev/i2c-1", 0.1, 2);
    inaConfigure(&ina, CONFIG_VOLTAGE_32V, CONFIG_GAIN_8, CONFIG_ADC_12BIT_4SAMPLES, CONFIG_ADC_12BIT_4SAMPLES, CONFIG_MODE_SHUNT_BUS_CONT);

    DATAPACK datapack;

    inaGetData(&ina, 0b00111111, &datapack);
    printDatapack(&datapack);

    /*
    float busVoltage_V;
    float busVoltage_mV;
    float current_A;
    float current_mA;
    float power_W;
    float power_mW;
    float shuntVoltage_mV;

    inaReadBusVoltage_V(&ina, &busVoltage_V);
    inaReadBusVoltage_mV(&ina, &busVoltage_mV);
    inaReadCurrent_A(&ina, &current_A);
    inaReadCurrent_mA(&ina, &current_mA);
    inaReadPower_W(&ina, &power_W);
    inaReadPower_mW(&ina, &power_mW);
    inaReadShuntVoltage_mV(&ina, &shuntVoltage_mV);

    fprintf(stdout, "Bus voltage: %.02f V\nBus voltage: %.02f mV\nCurrent: %.02f A\nCurrent: %.02f mA\nPower: %.02f W\nPower: %.02f mW\nShunt voltage: %.02f mV\n",
            busVoltage_V, busVoltage_mV, current_A, current_mA, power_W, power_mW, shuntVoltage_mV);
    
    */
    
    inaTerminate(&ina);

    return 0;
}