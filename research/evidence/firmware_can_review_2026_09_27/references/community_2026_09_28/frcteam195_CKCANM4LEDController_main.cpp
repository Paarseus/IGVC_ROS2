// source: https://github.com/frcteam195/CKCANM4LEDController/blob/f63fc34/src/main.cpp (fetched 2026-09-28)
#include <Arduino.h>
#include <CAN.h>
#include "frc_can_defines.hpp"
#include "main.hpp"
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SleepyDog.h>
#include "drivers/LEDDriverNeoPixel.hpp"
#include "LEDController.hpp"
#include "drivers/RGBColor.hpp"
#include "Globals.h"
#include "utils/Colors.hpp"

#define DEVICE_ID 1
#define PIN_NEOPIXEL_STRIP_1 24
#define PIN_NEOPIXEL_STRIP_2 14

#define FRC_HEARTBEAT_ID 0x01011840

uint32_t CK_CAN_ID = 0;
uint32_t CK_CAN_MASK = 0;

uint32_t counter = 0;
APIClass prevMode = APIClass::IDLE;
APIClass currMode = APIClass::IDLE;

Adafruit_NeoPixel mOnboardLED(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
LEDDriverNeoPixel mOnboardLEDDriver(mOnboardLED);
LEDController mOnboardLEDController(&mOnboardLEDDriver);

Adafruit_NeoPixel mNeoPixelStrip(60, PIN_NEOPIXEL_STRIP_1, NEO_GRBW + NEO_KHZ800);
LEDDriverNeoPixel mNeoPixelDriver(mNeoPixelStrip);
LEDController mLEDController(&mNeoPixelDriver);

#ifdef DEBUG
TimeoutTimer tt(1000);
#endif

#define DEFAULT_CAN_TIMEOUT 500
int32_t mValidCANTimeout = 0;

void setup()
{
	Watchdog.enable(2000);

#ifdef DEBUG
	Serial.begin(9600);
#endif	
	pinMode(PIN_CAN_STANDBY, OUTPUT);
	digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
	pinMode(PIN_CAN_BOOSTEN, OUTPUT);
	digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster

	pinMode(PIN_NEOPIXEL_POWER, OUTPUT);
	digitalWrite(PIN_NEOPIXEL_POWER, true);

	mOnboardLED.begin();
	mOnboardLEDController.init();
	mOnboardLEDController.configureDefaultState(LEDState::BLINK);
	mOnboardLEDController.configureBlink(1, 500);
	mOnboardLEDController.setColor(0x00FF0000);
	mOnboardLEDController.setBrightness(0xFF);
	mOnboardLEDController.setRequestedState(LEDState::BLINK);

	mNeoPixelStrip.begin();	//Init strip
	mLEDController.init();	//Init LED Controller
	mLEDController.configureDefaultState(LEDState::FIXED_ON);

#ifdef DEBUG
		Serial.println("Initialized LEDs!");
#endif

	// start the CAN bus at 1 Mbps
	if (!CAN.begin(1E6))
	{
#ifdef DEBUG
		Serial.println("Starting CAN failed!");
#endif
		//resetFunc();
	}

	CK_CAN_ID = (((uint32_t)DeviceType::IOBreakout << DEVICE_TYPE_OFFSET) & DEVICE_TYPE_MASK) |
				(((uint32_t)Manufacturer::TeamUse << MANUFACTURER_OFFSET) & MANUFACTURER_MASK) |
				(((uint32_t)APIClass::IDLE << API_CLASS_OFFSET) & API_CLASS_MASK) |
				(((uint32_t)APIIndex::DEFAULT << API_INDEX_OFFSET) & API_INDEX_MASK) |
				((DEVICE_ID << DEVICE_NUMBER_OFFSET) & DEVICE_NUMBER_MASK);

	CK_CAN_MASK = DEVICE_TYPE_MASK | MANUFACTURER_MASK | DEVICE_NUMBER_MASK;

	//CAN.filterExtended(CK_CAN_ID, CK_CAN_MASK & FRC_HEARTBEAT_ID);

	prevMode = APIClass::OFF;
	currMode = APIClass::COMM_LOSS;

	Watchdog.reset();
}

void loop()
{
	int packetSize = CAN.parsePacket();
	if (packetSize)
	{
		mValidCANTimeout = DEFAULT_CAN_TIMEOUT;

		uint8_t receivedData[packetSize];
		for (int i = 0; i < packetSize; i++) {
			receivedData[i] = CAN.read();
		}

		uint32_t packetID = (uint32_t)CAN.packetId();
#ifdef DEBUG
		// if (counter++ % 100 == 0)
		// {
			// Serial.printf("CAN Mask: %x\n", CK_CAN_MASK);
			if ((packetID & CK_CAN_MASK) == CK_CAN_ID)
			{
				Serial.printf("Got packet CAN ID: %x\n", CAN.packetId());
				Serial.printf("DevType: %x, MfgType: %x, APIClass: %x, APIIndex: %x, DevID: %x\n",
					(packetID & DEVICE_TYPE_MASK) >> DEVICE_TYPE_OFFSET,
					(packetID & MANUFACTURER_MASK) >> MANUFACTURER_OFFSET,
					(packetID & API_CLASS_MASK) >> API_CLASS_OFFSET,
					(packetID & API_INDEX_MASK) >> API_INDEX_OFFSET,
					(packetID) & DEVICE_NUMBER_MASK
				);
			}
		// }
#endif

		if ((packetID & CK_CAN_MASK) == CK_CAN_ID)
		{
			APIClass apiClass = (APIClass)((packetID & API_CLASS_MASK) >> API_CLASS_OFFSET);
			APIIndex apiIndex = (APIIndex)((packetID & API_INDEX_MASK) >> API_INDEX_OFFSET);
			handleCANPacket(receivedData, packetSize, apiClass, apiIndex);
		}
		else if (packetID == FRC_HEARTBEAT_ID)
		{
			if (packetSize >= 5)
			{
				bool robotEnabled = (receivedData[4] & 0x10) > 0;
				if (robotEnabled)
				{
					mOnboardLEDController.setColor(0x0000FF00);
					mOnboardLEDController.configureDefaultState(LEDState::FIXED_ON);
					mOnboardLEDController.setRequestedState(LEDState::FIXED_ON);
				}
				else
				{
					mOnboardLEDController.configureDefaultState(LEDState::BLINK);
					mOnboardLEDController.setRequestedState(LEDState::BLINK);
					mOnboardLEDController.setColor(0x00FFBF00);
				}
			}
		}
	}

	if (--mValidCANTimeout <= 0)
	{
		mOnboardLEDController.setColor(0x00FF0000);
		mOnboardLEDController.configureDefaultState(LEDState::BLINK);
		mOnboardLEDController.setRequestedState(LEDState::BLINK);
	}

#ifdef DEBUG
	if (tt.isTimedOut())
	{
		Serial.printf("Timeout 1s, Millis: %d \n", millis());
		tt.reset();
	}
#endif
	
	mLEDController.run();
	mOnboardLEDController.run();

	Watchdog.reset();
}

void handleCANPacket(uint8_t* data, int packetSize, APIClass apiClass, APIIndex apiIndex)
{
	switch (apiClass)
	{
		case APIClass::IDLE:
		{
			switch (apiIndex)
			{
				case APIIndex::SET_COLOR:
				{
					if (packetSize >= 4)
					{
						RGBColor rgb;
						rgb.white = data[0];
						rgb.red = data[1];
						rgb.green = data[2];
						rgb.blue = data[3];
#ifdef DEBUG
						// Serial.printf("W,R,G,B: %x,%x,%x,%x\n", rgb.white, rgb.red, rgb.green, rgb.blue);
						// Serial.printf("uint32_t: %d\n", rgb.getUInt32());
#endif
						mLEDController.setColor(rgb);
					}
					break;
				}
				case APIIndex::SET_BRIGHTNESS:
				{
					if (packetSize >= 1)
					{
						uint8_t brightness;
						brightness = data[0];

#ifdef DEBUG
						// Serial.printf("brightness: %d\n", brightness);
#endif
						mLEDController.setBrightness(brightness);
					}
					break;
				}
				case APIIndex::SET_BLINK:
				{
					if (packetSize >= 3)
					{
						uint16_t blinkRateMs;
						uint8_t blinkCount;

						blinkRateMs = data[0] | ((data[1] << 8) & 0xFF00);
						blinkCount = data[2];
						mLEDController.configureBlink(blinkCount, blinkRateMs);
					}
					break;
				}
				case APIIndex::SET_DEFAULT_STATE:
				{
					if (packetSize > 0)
					{
						LEDState ledState;
						ledState = (LEDState)data[0];
						mLEDController.configureDefaultState(ledState);
					}
					break;
				}
				default:
				{
					break;
				}
			}
			break;
		}
		case APIClass::OFF:
		{
			mLEDController.setOff();
			break;
		}
		case APIClass::FIXED_ON:
		{
#ifdef DEBUG
			// Serial.println("Got fixed on CAN packet!");
#endif
			mLEDController.setOn();
			mLEDController.setRequestedState(LEDState::FIXED_ON);
			switch (apiIndex)
			{
				case APIIndex::FLOAT_PIXEL:
				{
					if (packetSize >= 8)
					{
						RGBColor rgb;
						rgb.white = data[0];
						rgb.red = data[1];
						rgb.green = data[2];
						rgb.blue = data[3];
						uint16_t pixelCount = data[4] | ((data[5] << 8) & 0xFF00);
						uint16_t pixelRepeatSpacing = data[6] | ((data[7] << 8) & 0xFF00);
						(void)rgb;
						(void)pixelCount;
						(void)pixelRepeatSpacing;
						// mLEDController.setFloatPixel(rgb, pixelCount, pixelRepeatSpacing);
					}
					break;
				}
				default:
				{
					break;
				}
			}
			break;
		}
		case APIClass::BLINK:
		{
			mLEDController.setRequestedState(LEDState::BLINK);
			break;
		}
		case APIClass::COMM_LOSS:
		{
			mLEDController.setColorLock(true);
			mLEDController.setColor(RED, true);
			mLEDController.setBrightness(0xFF);
			mLEDController.configureDefaultState(LEDState::MORSE);
			mLEDController.configMessage("SOS", true);
			break;
		}
		case APIClass::COMM_RESTORED:
		{
			mLEDController.setBrightness(0xFF);
			mLEDController.setColor(GREEN, true);
			mLEDController.configureDefaultState(LEDState::FADE);
			mLEDController.setRequestedState(LEDState::BLINK);
			break;
		}
		case APIClass::FADE:
		{
#ifdef DEBUG
	Serial.println("Setting mode to fade!");
#endif
			mLEDController.setRequestedState(LEDState::FADE);
			break;
		}
		case APIClass::MORSE:
		{
			std::string msg;
			if (packetSize > 0)
			{
				for (uint8_t i = 0; i < packetSize; i++)
				{
					msg.push_back((char)data[i]);
				}

				switch (apiIndex)
				{
					case APIIndex::MORSE_MESSAGE_1:
					{

						mLEDController.configMessage(msg, true);
						break;
					}
					case APIIndex::MORSE_MESSAGE_2:
					case APIIndex::MORSE_MESSAGE_3:
					case APIIndex::MORSE_MESSAGE_4:
					{
						mLEDController.addToMessage(msg);
						break;
					}

					default:
						break;
				}
			}
			break;
		}
		case APIClass::DRIVER_SIGNAL:
		{
			break;
		}
		case APIClass::ENDGAME:
		{
			break;
		}
	}

	prevMode = currMode;
	currMode = apiClass;
}