# AirMixer
Controller to monitor and distribute evenly warm air from stove in a countryside house. (in Russian)

ATMega8 with connected two DS18B20 measures temperature in a room. 
When measurements differ, it turns on a fan to mix air in the room, 
gradually increasing applied power (to produce less noise and consume
less power). 
