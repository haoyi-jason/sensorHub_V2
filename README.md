16001 Sensor Hub project.
Interfacing sensors with STM32F205RB mcu, support sensor:
1. BMI160
2. ADXL355
3. AD7124(ADC)
4. HTUD21 temperature/humidity sensor
support interface
1. Bluetooth (ST)
2. WIFI(CC3100)

note:
1. in BT mode, BT module occupy UART2, the DMA channel(s) is conflit
with SPID3, so the sensor connect to P8(SPID3) should remap to use SPID1

# sensorhub_V2
