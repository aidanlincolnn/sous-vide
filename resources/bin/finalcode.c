//Sous Vide Final Project
//Partner Codes: ZG,CU,6C
#include "maevarmGEN.h"
#include "maevarmUSART.h"
#include "maevarmADC.h"
#include "maevarmUSB.h"

//Pin assignment
#define potentiometer F1//potentiometer input connected to pin F1
#define menuButton B0 //menu button connected to pin B0
#define onButton  B1 // on button connected to pin B1
#define power B2 // output signal to power switch connected to digital pin B2 , connected to PowerSwitch 1 +in, 2-in connected to GND
#define RedLED  B6 //red led connected to digital pin B3
#define GreenLED  B5 //green led connected to digital pin B5
#define OW_PIN D0 //temp sensor connected to digital pin D0

#include "ds18s20.h"
#include <math.h>

int  counts = 0; 	//ADC count variable
double voltage = 0;	//store voltage after ADC conversion
double currentT = 0; 	//temp in Fahrenheit
double desiredT = 100; 	//reasonable starting temperature, meat will need to be cooked at a higher t
double difference = 0; 	//difference between temperatures
double old_diff = 0; 	//for PID
double deriv = 0; 	//Derivative for PID
double integ = 0; 	//Integral for PID
double duty = 0; 	//duty cycle as percent of PWM
double Kp = .515;
double Kd = -.17;
double Ki = .007;
double period = 20000; 	//set period of PWM (20 seconds)
int cookmode = 1; 	//1= pid 0= thermostat
int continuecook=0; 	//keep sous vide on
int temptype = 0;	// 0 =F, 1=C
int timerOn =0; 	//0 = off, 1=on
double totaltime = 0; 	//total time for timer
double currentTime = 0; //current time for timer

void setupsousvide()
{
	//Set up I/O
	IO_mode(OW_PIN,INPUT);
	IO_mode(onButton,INPUT);
	IO_mode(menuButton,INPUT);
	IO_mode(potentiometer,INPUT);
	IO_mode(power,OUTPUT);
	IO_mode(RedLED,OUTPUT);
	IO_mode(GreenLED,OUTPUT);
	
	//Initialize USART
	USART_init(9600,USART_ASYNC); //set baud rate to 9600
	USART_tx_enable();
	// turn display on with cursor off and no blink
	USART_tx(0x16);
	//form feed to completely clear the display
	USART_tx(0x0C);
	delay_ms(5);
	
	//set up ADC
	//set reference voltage to internal +5V
	ADC_set_AREF(AREF_Vcc);
	//set clock to 125 KHz. If CPU clock is 16 MHz, then
	//use the /128 prescaler A
	ADC_set_CLK(ADC_CLK128);
	ADC_set_channel(0);	//set ADC channel to F1
	ADC_enable();	//enable ADC module
	
	//Reset PID variables 
	deriv=0;	
	integ=0;
	duty=0;
	difference=0;
	old_diff=0;	
}

double GetTemp() //read from digital thermometer 
{
	if(!ds_present())//if thermometer is not present
	exit(0);//exit
	currentT = ds_get_temp();//read thermometer
	currentT = currentT/16;//divide by 16
	currentT = currentT;
	return currentT;//return temp in Celsius
}

double readpotentiometer(int value) //read from the potentiometer 
{
	//read voltage from potentiometer for voltage
	counts = ADC_read(); //possible 1023 counts 
	
	//for Fahrenheit 0 = 70 degrees, 1023 = 210 degrees, 140/1023 = 0.1368523949 degrees per step
	//for Celsius, 0=25, 1023=98, 73/1023 = .071359 degrees per step

	if (value == 0) //reading a temperature 
	{
		if (temptype ==0) //F
		{
			desiredT = (counts * 0.1368523949)+70;	//calculate desired T
			desiredT= floor(desiredT);
			return desiredT; //return desired Temp
		}
		else if (temptype ==1) //C
		{
			desiredT = (counts * 0.071359)+25;	//calculate desired T
			desiredT= floor(desiredT);
			return desiredT; //return desired Temp
		}
	}
	else //reading a time
	{
		totaltime = counts /5.6833; 			//timer goes from 0 min - 180 min
		totaltime= floor(totaltime);
		return totaltime;
	}
}


double getduty(double diff, double der, double intg) //return duty cycle as a percentage of period of PWM (can only be between 0 and 1)
{
	duty = Kp*diff + Kd*der + Ki*intg;
	if (duty>1)
		duty = 1;
	if (duty <0)
		duty = 0;
	return duty;
}

void powerout(double dutycycle)
{
	//control power output determined by duty cycle
	int delayhigh = (int) (dutycycle*period);		//high part of pwm
	int delaylow =(int) (period - (dutycycle*period));	//low part of pwm
	usb_printf("Duty = %f\n",duty);				//print statements for terminal
	usb_printf("Delayhigh = %d\n",delayhigh);
	usb_printf("Delaylow = %d\n",delaylow);
	usb_printf("Difference = %f\n", difference);
	usb_printf("Current T %.1f \n",currentT);
	IO_out(power,HIGH);					//turn heater on
	IO_out(RedLED,HIGH);					//turn red led on
	delay_ms(delayhigh); //delay for high part of PWM	//delay for high part of pwm
	IO_out(power,LOW);					//turn off heater
	IO_out(RedLED,LOW);					//turn off red led
	delay_ms(delaylow); //delay for low part of PWM		//delay for low part of pwm
}

int main()
{
	usb_initialize();			//initialize usb communication, reset goes to here
	setupsousvide();			//set up sous vide initialization
	while(1)				//loop forever
	{	
		IO_out(RedLED,HIGH); 		//turn on red led
		IO_out(GreenLED,HIGH); 		//turn on green led
		USART_tx(0x0C);
		delay_ms(250);
		while (IO_read(menuButton) ==0) //temperature menu, can choose F or C
		{
			USART_tx(0x80);		//move cursor to line 0 position 0
			USART_printf("Temp mode:");
			USART_tx(0x94);		//move cursor to line 1 position 0
			if (temptype ==0)
			USART_printf("Fahrenheit      ");
			else if (temptype ==1)
			USART_printf("Celsius         ");
			if (IO_read(onButton) ==1)
			{
				if (temptype ==1)
					temptype =0;
				else
					temptype =1;
				delay_ms(200); //contact bounce
			}
		}
		USART_tx(0x0C);
		delay_ms(250);

		while (IO_read(menuButton) ==0) //cook mode menu, can choose PID or Thermostat
		{
			USART_tx(0x80);//move cursor to line 0 position 0
			USART_printf("Cook Mode:");
			USART_tx(0x94);//move cursor to line 1 position 0

			if (cookmode == 0)
				USART_printf("Thermostat      ");
			else if (cookmode ==1)
				USART_printf("PID             ");
			if (IO_read(onButton) ==1)
			{
				if (cookmode ==1)
					cookmode =0;
				else
					cookmode =1;
				delay_ms(200); //contact bounce
			}
		}
		USART_tx(0x0C);
		delay_ms(250);

		while (IO_read(menuButton) ==0) //timer menu, can choose on or off
		{
			USART_tx(0x80);//move cursor to line 0 position 0
			USART_printf("Turn On Timer?");
			USART_tx(0x94);//move cursor to line 1 position 0
			if (timerOn == 1)
				USART_printf("Yes");
			else if (timerOn ==0)
				USART_printf("No ");
			if (IO_read(onButton ==1))
			{
				if (timerOn ==1)
					timerOn =0;
				else
					timerOn =1;
				delay_ms(200);//contact bounce
			}
		}
		USART_tx(0x0C);
		delay_ms(250);

		if (timerOn ==1) //timer set menu
		{
			delay_ms(500);
			USART_tx(0x0C);
			delay_ms(5);
			while (IO_read(menuButton) ==0)
			{
				USART_tx(0x80);//move cursor to line 0 position 0
				USART_printf("Set Timer");
				USART_tx(0x94);//move cursor to line 1 position 0
				totaltime = readpotentiometer(1);	//read potentiometer to get time for timer
				USART_printf("%.f",totaltime);
				USART_printf(" Minutes ");
			}
		}
		USART_tx(0x0C);
		delay_ms(250);
		
		while (IO_read(menuButton) ==0)	//set desired temperature menu
		{
			USART_tx(0x80);//move cursor to line 0 position 0
			USART_printf("Set Temperature");
			desiredT = readpotentiometer(0);
			USART_tx(0x94);//move cursor to line 1 position 0
			USART_printf("Desired T=%.f",desiredT);
			if (temptype ==0)
				USART_printf("F  ");
			else
				USART_printf("C  ");

			//start sous vide here
			if(IO_read(onButton) == 1) //start sous vide when button is pressed
			{
				IO_out(RedLED,LOW); //turn off red led
				IO_out(GreenLED,LOW); //turn off green led
				delay_ms(1000);//for button bounce

				currentTime = 0;//for timer
				continuecook =0;//to exit to main menu

				while(continuecook == 0) //run until button is pressed again
				{
					if(IO_read(onButton) ==1) 	//check if button has been pressed again
						continuecook = 1;
					currentT = GetTemp();		//get  temperature
					if (temptype == 0)
						currentT = ((currentT*9)/5) + 32;
					USART_tx(0x80);			//move cursor to line 0 position 0
					USART_printf("Current T=%.1f",currentT);
					if (temptype ==0)		//print current temperature
						USART_printf("F  ");
					else
						USART_printf("C  ");
					USART_tx(0x94);			//move cursor to line 1 position 0
					USART_printf("Desired T=%.f",desiredT);
					if (temptype ==0)
						USART_printf("F  ");
					else
						USART_printf("C  ");
					
					old_diff = difference;		//save old difference
					difference =  desiredT- currentT;//get new difference

					deriv = old_diff - difference; //calculate derivative
					integ += difference;		//calculate integral
					
					if (cookmode == 0)  //thermostat mode
					{
						if (difference <.5 && difference>-.5) //turn on green led when temp is close
						{
							IO_out(RedLED,LOW);	//sous vide off- red low
							IO_out(GreenLED,HIGH);	//reached reasonable temp - green high
						}
						if (difference >= 0)		//turn on power when difference is high
						{
							IO_out(power,HIGH);
							IO_out(GreenLED,LOW);
							IO_out(RedLED,HIGH);	//sous vide on- red high
						}
						else if(difference <0)		//turn off power when difference is low
						{
							IO_out(power,LOW);
							IO_out(GreenLED,LOW);
							IO_out(RedLED,LOW);	//sous vide off- red low
						}
						
						
					}
					else if (cookmode == 1)	//pid mode
					{
						
						if (difference <-2 || difference >2) //use thermostat control until within 2 degrees
						{
							IO_out(GreenLED,LOW);
							if (difference >= 0)	     //turn on power when difference is positive
							{
								IO_out(power,HIGH);
								IO_out(RedLED,HIGH); //sous vide on- red high
							}
							else if(difference <0)	     //turn off power when difference is negative
							{
								IO_out(power,LOW);
								IO_out(RedLED,LOW);	//sous vide off- red low
							}
							currentTime = currentTime + .02; //figure out loop time and put it into x 1.2 seconds per loop
							
						}
						else
						{
							currentTime = currentTime + .3533; //figure out loop time and put it into x 21.2 second per loop
							IO_out(GreenLED,HIGH);
							duty = getduty(difference,deriv,integ); //PID control
							powerout(duty); //output PWM wave based on PID result
						}

					}
					
					
					if (currentTime >= totaltime && timerOn==1 && cookmode==1) // if we reach the time , beep twice each loop, timing calculation no longer matters
					{
						USART_tx(0xD4); //1/4th note
						USART_tx(0xDA); //6th scale
						USART_tx(0xDF); //play C
						delay_ms(200);
						USART_tx(0xDF); //play C
					}
				}
				IO_out(power,LOW); //turn off sous vide if on button pressed
				delay_ms(750);
			}
			IO_out(power,LOW); //turn off sous vide if on button pressed
			delay_ms(200);
		}
		
	}
}