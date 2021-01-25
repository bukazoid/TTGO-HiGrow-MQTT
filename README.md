# TTGO-HiGrow-MQTT

This is my contribution to the many TTGO HiGrow implementations. 
Code is based on the TTGO HiGrow 19-8-22 v1.1 model with the BME280 sensor and without the DHT.

The mqtt topic is structured like: `plant/<mac-id>/<sensorkind>`. 

Sensors are:
- plant/id/temperature      *(float)*   
- plant/id/pressure         *(float)*
- plant/id/lumen            *(float)*
- plant/id/battery          *(float)*
- plant/id/soil             *(int)*
- plant/id/salt             *(int)*
- plant/id/meta             *(json)*

The meta data contains:
``` json    
{  
    "mac":"C4:4F:33:7F:FF:F5",  
    "ip":"192.168.0.190",       
    "bootcount":637,            
    "firmware":"1.0.3"          
}
```

## Building 

Use [Visual Studio Code](code.visualstudio.com) with the PlatformIO plugin. 
Connect with USB. 
From the PlatformIO plugin choose : Build, Upload and/or Monitor

## Configuration options

In the file `include\user-variables.h`, you can specify:

- a list of ssid's and their passwords
- static ip => (recommended => less time, less power)
- mqtt server settings
- duration of the deep sleep (frequentie of updates)

## DHT

If you do have a DHT11/12/22 attached, you wont have much troubles re-enabling this functionality.
Most code is commented. 

## Acknowledge

This code was based on the work of 

https://github.com/Xinyuan-LilyGO/TTGO-HiGrow

https://github.com/pesor/TTGO-T-HIGrow



