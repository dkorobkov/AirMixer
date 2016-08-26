/*
 * AirMixer.c
 *
 * Created: 19.08.2016 16:51:28
 *  Author: -
 
 Проект - смешиватель воздуха. Стоит 2 термометра
 DS18B20, контроллер сравнивает температуру и, если она 
 отличается больше чем на 1.5 градуса, включает вентилятор
 с ШИМ, сначала слабенько, потом на полную мощность.
 
 Вентилятор управляется полевым транзистором через опторазвязку,
 которую надо дёргать на землю для включения.
 
 Это версия 2, оказалось, что ШИМ работает отвратительно, интерферирует с 
 частотой сети, трещит и всё такое. ПОэтому сделаны 3 реле, одно отключает
 схему на время переключения, чтобы не искрило, два других подключают гасящие конденсаторы 
 или замыкают напрямую балласт вентилятора.
 
 Реле подключены к PD2, PD3, PD4:
 
 PD2, PD3 - электромагнитные РЭС-10.
 
 PD2=0, PD3=0 - максимальная скорость
 PD2=1, PD3=0 - минимальная скорость
 PD2=0, PD3=1 - вторая скорость
 PD2=1, PD3=1 - третья скорость
 
 
 PD4 - главное оптореле KSD203AC2, которое всё обесточивает. КОгда его расцепили (0), надо подождать 
 несколько секунд, чтобы остальные реле разрядились через резисторы и не искрили. Потом 
 можно переключить эти реле (РЭС10), и затем включить оптореле.
 */ 

#define F_CPU 4000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>


#include "OWIPolled.h"
#include "OWIHighLevelFunctions.h"
#include "OWIBitFunctions.h"
#include "OWIcrc.h"


// Defines used only in code example.
#define DS1820_FAMILY_ID                0x10 
#define DS18B20_FAMILY_ID                0x28
#define DS1820_START_CONVERSION         0x44
#define DS1820_READ_SCRATCHPAD          0xbe
#define DS1820_ERROR                    -1000   // Return code. Outside temperature range.

#define SEARCH_SUCCESSFUL               0x00
#define SEARCH_CRC_ERROR                0x01

#define FALSE       0
#define TRUE        1

#define MAX_DEVICES 8       //!< Max number of devices to search for.

#define BUSES   (OWI_PIN_3) //!< Buses to search.

/*! \brief  Data type used to hold information about slave devices.
 *  
 *  The OWI_device data type holds information about what bus each device
 *  is connected to, and its 64 bit identifier.
 */
typedef struct
{
    unsigned char bus;      //!< A bitmask of the bus the device is connected to.
    unsigned char id[8];    //!< The 64 bit identifier.
} OWI_device;


// Prototypes of functions used in exemples.
unsigned char SearchBuses(OWI_device * devices, unsigned char len, unsigned char buses);
OWI_device * FindFamily(unsigned char familyID, OWI_device * devices, unsigned char size, unsigned char OrderNum);
signed int DS1820_ReadTemperature(unsigned char bus, unsigned char * id);
//void DS2890_SetWiperPosition(unsigned char position, unsigned char bus, unsigned char * id);



/*
Светодиод между PC0, PC1
*/
#define LED_G    {PORTC = (PORTC & 0xfc) | 1;}
#define LED_R	 {PORTC = (PORTC & 0xfc) | 2;}
#define LED_OFF	{PORTC = (PORTC & 0xfc);}


#define R1_ON {PORTD |= (1<<PD2);}
#define R1_OFF {PORTD &= ~(1<<PD2);}
#define R2_ON {PORTD |= (1<<PD3);}
#define R2_OFF {PORTD &= ~(1<<PD3);}
#define R3_ON {PORTD |= (1<<PD4);}
#define R3_OFF {PORTD &= ~(1<<PD4);}

/*
Мигаем сотнями (если есть), десятками и единицами
Сначала мигаем зелёным номер параметра
Потом значение до запятой
Потом коротко зелёным
Потом значение после запятой
Потом пауза
*/

// ShowPoint - если надо показать запятую, тогда в конце не делаем паузу
void FlashDigit(unsigned char UseRed, unsigned char n, unsigned char ShowPoint)
{
	if(n == 0)
		n = 10; // 10 раз мигаем, чтобы показать ноль ("105")
	while(n > 0)
	{
		if(UseRed)
		{
			LED_R;
		}
		else
		{
			LED_G;
		}
		_delay_ms(300);
		LED_OFF;
		_delay_ms(300);
		wdt_reset();
		
		n--;
	}
	
	LED_OFF;
	// если показать точку, то, не прерывая последовательность 
	// красных, мигаем зелёным - а дальше сразу следует цифра 
	// после запятой
	if(ShowPoint)
	{
		LED_G;
		_delay_ms(300); 
		LED_OFF;		
		_delay_ms(300); 
	}
	else
		_delay_ms(1000); // Пауза между цифрами (+ 400 мс выше = 1 сек)
	
	wdt_reset();
}

void ShowValue(int orderNum, int intpart, int floatpart)
{
	unsigned char bShow = 0;
	
	FlashDigit(0, orderNum, 0);
	
	int n = 0;	
	while(intpart >= 100)
	{
		n++;
		intpart -= 100;
		bShow = 1;
	}
	
	if(bShow)
		FlashDigit(1, n, 0);

	n = 0;
	while(intpart >= 10)
	{
		n++;
		intpart -= 10;
		bShow = 1;
	}

	if(bShow)
		FlashDigit(1, n, 0);

	// Единицы покажем по любому, запятую - если будут десятые доли
	FlashDigit(1, intpart, floatpart); 

	if(floatpart)
		FlashDigit(1, floatpart, 0); 
}


void SetPower(unsigned char power)
{
	R3_OFF;
	wdt_reset();
	_delay_ms(1000);
	wdt_reset();
	_delay_ms(1000);

	switch(power)
	{
		case 0:
		default:
			return; // не влкючаем реле
		case 1:
			R1_ON; // 0.33 uF
			R2_OFF;
			break;
		case 2:
			R1_OFF; 
			R2_ON;  //0.44 uF
		break;
		case 3:
			R1_ON; // 0.33 uF
			R2_ON; // 0.44 uF
		break;
		case 4:
			R1_OFF; // напрямую
			R2_OFF;
		break;
	}	
	// Включаем главное реле
	R3_ON;
}

int main(void)
{
	// вотчдог
	//WDTCR = (1<<WDP2) | (1<<WDP1) | (1<<WDP0); // 2 seconds
	wdt_enable(WDTO_2S);
	
	//LEDs
	DDRC=3;// | (1<<DDC5); // LED & PWM
	PORTC=0;
	
	// Реле
	DDRD = (1<<DDD2) | (1<<DDD3) | (1<<DDD4);
	PORTD = 0;

	unsigned char Power = 0;

    static OWI_device devices[MAX_DEVICES];
    OWI_device * ds1820_1 = NULL;
    OWI_device * ds1820_2 = NULL;
    signed int temperature_1 = 0;
    signed int temperature_2 = 0;
	signed int delta = 0;

	wdt_reset();
	LED_R;
	_delay_ms(100);
	LED_G;
	_delay_ms(100);
	LED_R;
	_delay_ms(100);
	LED_G;
	_delay_ms(100);
	LED_R;
	_delay_ms(100);
	LED_G;
	_delay_ms(100);
	LED_OFF;
	_delay_ms(500);
	wdt_reset();

    OWI_Init(BUSES);
    
	unsigned char srch=0;
    // Do the bus search until all ids are read without crc error.    
    while(SearchBuses(devices, MAX_DEVICES, BUSES) != SEARCH_SUCCESSFUL)
    {
		ShowValue(4, 4, 4);
    }

	unsigned char nDevicesFound = 0;
	for(srch=0; srch < MAX_DEVICES; srch++)
		if(devices[srch].bus != 0x00)
			nDevicesFound++;

	{
		ShowValue(3, nDevicesFound, 0);
	}

    // See if there is a DS1820 or DS2890 on a bus.
    ds1820_1 = FindFamily(DS18B20_FAMILY_ID, devices, MAX_DEVICES, 0);
    ds1820_2 = FindFamily(DS18B20_FAMILY_ID, devices, MAX_DEVICES, 1);
    
	unsigned char JustChanged = 1; // 1 чтобы не лазить в проверку, пока не заполнился массив
	
	#define N_AVERAGE 10
	signed int arT1[N_AVERAGE] = {0,0,0,0,0,0,0,0,0,0};
	signed int arT2[N_AVERAGE] = {0,0,0,0,0,0,0,0,0,0};
	unsigned char idx = 0;	
	
    for (;;)
    {
        // If there is a DS1820 temperature sensor on a bus, read the
        // temperature.
        // The DS1820 must have Vdd pin connected for this code to work.
		if(JustChanged > 0) // ЧТобы не очень часто менять режим
			JustChanged++;
		if(JustChanged > N_AVERAGE) // максимальное JustChanged точно должно быть больше N_AVERAGE, чтобы правильно усреднять
			JustChanged = 0;
			
        if (ds1820_1 != NULL && ds1820_2 != NULL)
        {
			wdt_reset();
			
            temperature_1 = DS1820_ReadTemperature((*ds1820_1).bus, (*ds1820_1).id);
			wdt_reset();
            temperature_2 = DS1820_ReadTemperature((*ds1820_2).bus, (*ds1820_2).id);
			wdt_reset();

			// 12-битное значение + 15й бит знаковый

			if(temperature_1 < 0) temperature_1 = 0;
			if(temperature_2 < 0) temperature_2 = 0;

			//               On  -55.0°
			//               OFF 124.9°
			ShowValue(1, temperature_1/16, ((temperature_1 & 0x0f)*10) / 16);
			ShowValue(2, temperature_2/16, ((temperature_2 & 0x0f)*10) / 16);
			ShowValue(3, delta, 0);
			ShowValue(4, Power, 0);

			arT1[idx] = temperature_1;			
			arT2[idx] = temperature_2;
			idx++;
			if(idx >= N_AVERAGE)
				idx = 0;
			
			// Ещё бы гистерезис добавить.
			
			if(JustChanged == 0)
			{
				// Усредняем
				long t = 0;
				for(delta=0; delta < N_AVERAGE; delta++)
					t += arT1[delta];
				temperature_1 = t / N_AVERAGE;
				
				t = 0;
				for(delta=0; delta < N_AVERAGE; delta++)
					t += arT2[delta];
				temperature_2 = t / N_AVERAGE;
				
				delta = temperature_1 - temperature_2;
				if(delta < 0)
					delta = -delta;
			
				// В зависимости от дельты включаем ШИМ, дельта=16 это 1 градус
				unsigned char OldPower = Power;
				if(delta > 4*16)
					Power = 4;
				else if(delta > 3*16)
					Power = 3;
				else if(delta > 2*16)
					Power = 2;
				else if(delta > 2*16)
					Power = 1;
				else Power = 0;
			
				if(OldPower != Power)
				{
					JustChanged = 1;
					SetPower(Power);
				}
			}
			
        }        
		else
			ShowValue(5, 5, 5); 

		_delay_ms(1000);
		wdt_reset();
	}
}

/****  DS18B20 specific  ***********************************************************/
/*! \brief  Perform a 1-Wire search
 *
 *  This function shows how the OWI_SearchRom function can be used to 
 *  discover all slaves on the bus. It will also CRC check the 64 bit
 *  identifiers.
 *
 *  \param  devices Pointer to an array of type OWI_device. The discovered 
 *                  devices will be placed from the beginning of this array.
 *
 *  \param  len     The length of the device array. (Max. number of elements).
 *
 *  \param  buses   Bitmask of the buses to perform search on.
 *
 *  \retval SEARCH_SUCCESSFUL   Search completed successfully.
 *  \retval SEARCH_CRC_ERROR    A CRC error occured. Probably because of noise
 *                              during transmission.
 */
unsigned char SearchBuses(OWI_device * devices, unsigned char len, unsigned char buses)
{
    unsigned char i, j;
    unsigned char presence;
    unsigned char * newID;
    unsigned char * currentID;
    unsigned char currentBus;
    unsigned char lastDeviation;
    unsigned char numDevices;
    
    // Initialize all addresses as zero, on bus 0 (does not exist).
    // Do a search on the bus to discover all addresses.    
    for (i = 0; i < len; i++)
    {
        devices[i].bus = 0x00;
        for (j = 0; j < 8; j++)
        {
            devices[i].id[j] = 0x00;
        }
    }
    
    // Find the buses with slave devices.
    presence = OWI_DetectPresence(BUSES);
    
    numDevices = 0;
    newID = devices[0].id;
    
    // Go through all buses with slave devices.
	currentBus = OWI_PIN_3; // делаем только с одной шиной на ноге РC3
    {
        lastDeviation = 0;
        currentID = newID;
        if (currentBus & presence) // Devices available on this bus.
        {
            // Do slave search on each bus, and place identifiers and corresponding
            // bus "addresses" in the array.
            do  
            {
                memcpy(newID, currentID, 8);
                OWI_DetectPresence(currentBus);
                lastDeviation = OWI_SearchRom(newID, lastDeviation, currentBus);
                currentID = newID;
                devices[numDevices].bus = currentBus;
                numDevices++;
                newID=devices[numDevices].id;                
            }  while(lastDeviation != OWI_ROM_SEARCH_FINISHED);            
        }
    }

    // Go through all the devices and do CRC check.
    for (i = 0; i < numDevices; i++)
    {
        // If any id has a crc error, return error.
        if(OWI_CheckRomCRC(devices[i].id) != OWI_CRC_OK)
        {
            return SEARCH_CRC_ERROR;
        }
    }
    // Else, return Successful.
    return SEARCH_SUCCESSFUL;
}

/*! \brief  Find the first device of a family based on the family id
 *
 *  This function returns a pointer to a device in the device array
 *  that matches the specified family.
 *
 *  \param  familyID    The 8 bit family ID to search for.
 *
 *  \param  devices     An array of devices to search through.
 *
 *  \param  size        The size of the array 'devices'
 *
 *  \return A pointer to a device of the family.
 *  \retval NULL    if no device of the family was found.
 */
OWI_device * FindFamily(unsigned char familyID, OWI_device * devices, unsigned char size, unsigned char OrderNum)
{
    unsigned char i = 0;
    
    // Search through the array.
    while (i < size)
    {
        // Return the pointer if there is a family id match.
        if ((*devices).id[0] == familyID)
        {
			if(OrderNum == 0)
				return devices;
			else OrderNum--; // Ищем следующий
        }
        devices++;
        i++;
    }
    // Else, return NULL.
    return NULL;
}


/*! \brief  Read the temperature from a DS1820 temperature sensor.
 *
 *  This function will start a conversion and read back the temperature
 *  from a DS1820 temperature sensor.
 *
 *  \param  bus A bitmask of the bus where the DS1820 is located.
 *  
 *  \param  id  The 64 bit identifier of the DS1820.
 *
 *  \return The 16 bit signed temperature read from the DS1820.
 */
signed int DS1820_ReadTemperature(unsigned char bus, unsigned char * id)
{
    signed int temperature;
    
    // Reset, presence.
    if (!OWI_DetectPresence(bus))
    {
        return DS1820_ERROR; // Error
    }
    // Match the id found earlier.
    OWI_MatchRom(id, bus);
    // Send start conversion command.
    OWI_SendByte(DS1820_START_CONVERSION, bus);
    // Wait until conversion is finished.
    // Bus line is held low until conversion is finished.
	// ?? ????. ????? ?? ??? ???? ????????? 750 ?? ?? ????????, 
	// ?? ????? ????? ? ?? ????? ??????? ???????.
	
	_delay_ms(750);

    while (!OWI_ReadBit(bus))
    {
    
    }
    // Reset, presence.
    if(!OWI_DetectPresence(bus))
    {
        return -1000; // Error
    }
    // Match id again.
    OWI_MatchRom(id, bus);
    // Send READ SCRATCHPAD command.
    OWI_SendByte(DS1820_READ_SCRATCHPAD, bus);
    // Read only two first bytes (temperature low, temperature high)
    // and place them in the 16 bit temperature variable.
    temperature = OWI_ReceiveByte(bus);
    temperature |= (OWI_ReceiveByte(bus) << 8);
    
    return temperature;
}


