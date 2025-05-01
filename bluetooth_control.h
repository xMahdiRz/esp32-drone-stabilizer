#ifndef BLUETOOTH_CONTROL_H
#define BLUETOOTH_CONTROL_H

#include <BluetoothSerial.h>

// Function prototypes
void setupBluetooth();
void handleBluetooth();

// Bluetooth interface object
extern BluetoothSerial SerialBT;

#endif // BLUETOOTH_CONTROL_H
