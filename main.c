#include "address_map_arm.h"

#define ADC_OFFSET 98

typedef struct
{
    double iState;      // Integrator state
    double iMax, iMin;  // Maximum and minimum allowable integrator state

    double iGain,	// integral gain
           pGain;	// proportional gain
} SPid;

void VGA_box(int, int, int, int, short);
void VGA_clr(void);
void VGA_text(char *, int, int);
void VGA_Update(int, float);
void Poll_Keyboard(void);
char ASCII_Convert(int);
int HEX_Lookup(char);
double Get_RPM(void);
double UpdatePID(SPid *, double);
void Set_PWM(unsigned int, float);
void put_jtag(char);

volatile int * JP2_GPIO_ptr = (int *) JP2_BASE;
volatile int * ADC_ptr = (int *) ADC_BASE;
volatile int * HPS_timer0_ptr = (int *) HPS_TIMER0_BASE;
volatile int * HPS_timer1_ptr = (int *) HPS_TIMER1_BASE;
volatile int * MPcore_private_timer_ptr = (int *) MPCORE_PRIV_TIMER;
volatile int * PS2_ptr = (int *) PS2_BASE;
volatile int * HEX3_HEX0_ptr = (int *) HEX3_HEX0_BASE;
volatile int * HEX5_HEX4_ptr = (int *) HEX5_HEX4_BASE;
volatile int * KEY_ptr = (int *) KEY_BASE;

int numVoltages = 0;
int sumVoltages = 0;
int numRPMs = 0;
double sumRPMs = 0.0;
int pulseCount = 0;
int pidLoopTime = 200;
int pidLoopCounter = 0;
int pwmOn = 0;
int pwmPeriod = 1000;	// Period in milliseconds
float pwmPercent;
int pwmTime;
SPid pidObj = { 0, 100, -100, 0.8, 1 };	 // iState, iMax, iMin, iGain, pGain
int RPM_input = 0;
double RPM_actual;
double motorCurrent;
double pulseDiff = 0;
int shouldUpdate = 1;
int motorIdle = 0;

int main(void)
{
	disable_A9_interrupts ();	// disable interrupts in the A9 processor
	set_A9_IRQ_stack ();			// initialize the stack pointer for IRQ mode
	config_GIC ();					// configure the general interrupt controller

	*(JP2_GPIO_ptr + 1) = 0x1; // configure JP2 D0 as output and D1 as input
	*(JP2_GPIO_ptr + 2) = 0x2; // set interrupt mask to enable only JP2 D1

	*(ADC_ptr + 1) = 1;			// Sets the ADC up to automatically perform conversions.

	*(HPS_timer0_ptr) = 100000; // timeout = 1/(100 MHz) x 100x10^3 = 1 msec
	*(HPS_timer0_ptr + 2) = 0b11; // mode = interrupt = 0, mode = 1, enable = 1
	*(HPS_timer1_ptr) = 100000000; // timeout = 1/(100 MHz) x 100x10^6 = 1 sec
	*(HPS_timer1_ptr + 2) = 0b111; // mode = interrupt = 1, mode = 1, enable = 1
	
	*(MPcore_private_timer_ptr) = 200000; // timeout = 1/(200 MHz) x 200x10^3 = 1 msec
	*(MPcore_private_timer_ptr + 2) = 0b111; // mode = interrupt = 1, auto = 1, enable = 1

	*(KEY_ptr + 2) = 0x1; 	// enable interrupts for only KEY 0

	*HEX3_HEX0_ptr = 0x3F;	// Display 0 for RPM input

	enable_A9_interrupts ();	// enable interrupts in the A9 processor
	VGA_clr();
	
	while (1)
	{
		Poll_Keyboard();

		if (shouldUpdate == 1)
		{
			shouldUpdate = 0;
			int avgRPM = (int)(sumRPMs / numRPMs + 0.5);
			sumRPMs = 0;
			numRPMs = 0;
			VGA_Update(avgRPM, motorCurrent);
		}
	}
}

char ASCII_Convert (int n)
{
	if (n == 0x70)      return '0';		//0
	else if (n == 0x69) return '1';		//1
	else if (n == 0x72) return '2';		//2
	else if (n == 0x7A) return '3';		//3
	else if (n == 0x6B) return '4';		//4
	else if (n == 0x73) return '5';		//5
	else if (n == 0x74) return '6';		//6
	else if (n == 0x6C) return '7';		//7
	else if (n == 0x75) return '8';		//8
	else if (n == 0x7D) return '9';		//9
	else if (n == 0x5A) return 'N';		//Enter
	else                return 'X';		//Error
}

int HEX_Lookup(char c)
{ 
	switch(c)
	{
		case '0':
			return 0x3F;
		case '1':
			return 0x06;
		case '2':
			return 0x5B;
		case '3':
			return 0x4F;
		case '4':
			return 0x66;
		case '5':
			return 0x6D;
		case '6':
			return 0x7D;
		case '7':
			return 0x07;
		case '8':
			return 0x7F;
		case '9':
			return 0x6F;
		default:
			return 0;
	}		
}

double Get_RPM(void)
{
	/*if (pulseCount == 0)
	{
		motorIdle++;
		if (motorIdle > (1000 / pidLoopTime))	// Return 0 RPM if no pulses read for 1 second
			return 0;
	}*/
	double fractionPulse = (100000000 - *(HPS_timer1_ptr + 1)) / 100000000.0;
	pulseDiff += fractionPulse;
	double pulsePerSecond = (1000.0 / pidLoopTime) * (pulseCount + pulseDiff);
	pulseDiff = -fractionPulse;
	return pulsePerSecond; // RPM = pulse/s * 60 / 3X
}

void Poll_Keyboard(void)
{
	static int committed = 0;
	static int ready;
	char ASCII;
	static int entries[4] = {0};
	int temp1 = *PS2_ptr;
	if ((temp1 & 0x8000) == 0x8000) // Indicates there are codes not read in yet
	{
		if ((temp1 & 0xFF) == 0xF0)  // Test for key break. Note some keys have 2 breaks.
		{
			ready = 1;
			committed = 0;
		}
		else if (ready == 1)   // Else if means a check that the input is not a break is completed.
		{
			ready = 0;
			ASCII = ASCII_Convert((int)(temp1 & 0xFF));
			if (ASCII == 'X') // Error
				;
			else if (ASCII == 'N' && entries[0] != 0) // Enter is pressed, there is at least 1 character.
			{
				committed = 1;
				int RPM_temp = 0;
				if (entries[3] != 0)
					RPM_temp += (entries[3] - 48) * 1000;
				if (entries[2] != 0)
					RPM_temp += (entries[2] - 48) * 100;
				if (entries[1] != 0)
					RPM_temp += (entries[1] - 48) * 10;
				RPM_input = RPM_temp + (entries[0] - 48);

				int i;
				for (i = 0; i < 4; i++)
					entries[i] = 0;   //Enter pressed, clear all entries

				*HEX5_HEX4_ptr = 0;
			}
			else if (ASCII != 'N')
			{
				*HEX5_HEX4_ptr = 0x7900;

				entries[3] = entries[2];
				entries[2] = entries[1];
				entries[1] = entries[0];
				entries[0] = ASCII;

				temp1 = HEX_Lookup(entries[3]); // Display lower digits
				temp1 = temp1 << 8;
				temp1 |= HEX_Lookup(entries[2]);
				temp1 = temp1 << 8;
				temp1 |= HEX_Lookup(entries[1]);
				temp1 = temp1 << 8;
				temp1 |= HEX_Lookup(entries[0]);
				*HEX3_HEX0_ptr = temp1;
			}
		}
	}
}

void Set_PWM(unsigned int period, float percent)
{
	if (!pwmOn) {
		pwmTime = (int)(period * percent + 0.5); // write to timer load register
	}
	else {
		pwmTime = (int)(period * (100 - percent) + 0.5);
	}

	if (percent == 0.0) {
		*(JP2_GPIO_ptr) &= 0xFFFFFFFE; // turn off GPIO pin
	}
	else if (percent == 100.0) {
		*(JP2_GPIO_ptr) |= 0x1; // turn on GPIO pin
	}
	else if (pwmOn) {
		*(JP2_GPIO_ptr) &= 0xFFFFFFFE; // turn off GPIO pin
	}
	else {
		*(JP2_GPIO_ptr) |= 0x1; // turn on GPIO pin
	}

	if (pwmTime > 0) {
		*(HPS_timer0_ptr + 2) &= 0b110;  // mode = interrupt = 1, mode = 1, enable = 0
		*(HPS_timer0_ptr) = pwmTime; // write to timer load register
		*(HPS_timer0_ptr + 2) |= 0x1;
	}

	pwmOn = !pwmOn;
}

double UpdatePID(SPid *pid, double error)
{
	double pTerm, iTerm;

	pTerm = pid->pGain * error;		// calculate the proportional term

	// calculate the integral state with appropriate limiting
	pid->iState += error;
	if (pid->iState > pid->iMax) pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) pid->iState = pid->iMin;

	iTerm = pid->iGain * pid->iState;	// calculate the integral term

	return pTerm + iTerm;
}

void VGA_Update(int avgRPM, float current)
{
	VGA_box(0, 319, 0, 239, 0); //Black background
	
	//Find the percent of RPM for Tach
	static char RPM_text[10] = {'R', 'P', 'M', ':', ' ', ' ', ' ', ' ', ' ', 0};
	static char Current_text[18] = {'C', 'u', 'r', 'r', 'e', 'n', 't', ':', ' ', ' ', ' ', ' ', ' ', '.', ' ', 'm', 'A', 0};
	float percent = (((float) avgRPM / 350.0f) * 100) + 0.05;
	
	int temp2 = avgRPM;
	RPM_text[5] = (temp2 / 1000) + 48;
	temp2 = temp2 % 1000;
	RPM_text[6] = (temp2 / 100) + 48;
	temp2 = temp2 % 100;
	RPM_text[7] = (temp2 / 10) + 48;
	temp2 = temp2 % 10;
	RPM_text[8] = temp2 + 48;

	short color = 255; //Blueish color

	if (RPM_text[5] == '0')
	{
		RPM_text[5] = ' ';
		if (RPM_text[6] == '0')
		{
			RPM_text[6] = ' ';
			if (RPM_text[7] == '0')
				RPM_text[7] = ' ';
		}
	}		

	int temp3 = (int)current;
	Current_text[9] = (temp3 / 1000) + 48;
	temp3 = temp3 % 1000;
	Current_text[10] = (temp3 / 100) + 48;
	temp3 = temp3 % 100;
	Current_text[11] = (temp3 / 10) + 48;
	temp3 = temp3 % 10;
	Current_text[12] = temp3 + 48;
	Current_text[14] = ((int)(current * 10) % 10) + 48;
	
	if (Current_text[9] == '0')
	{
		Current_text[9] = ' ';
		if (Current_text[10] == '0')
		{
			Current_text[10] = ' ';
			if (Current_text[11] == '0')
			{
				Current_text[11] = ' ';
			}
		}
	}
	
	int x1 = 60;
	int x2 = ((int) (percent * 2)) + 59;
	int y1 = 12;
	int y2 = 26;
	
	VGA_box(58, 262, 10, 28, (short)512); 
	if (x2 >= x1)
	{
		VGA_box(x1, x2, y1, y2, color);
	}
	VGA_text(RPM_text, 1, 1); //CHANGE X AND Y
	VGA_text(Current_text, 1, 9); //CHANGE X AND Y
}

void VGA_box(int x1, int x2, int y1, int y2, short pixel_color) //Display rectangle
{
	int pixel_ptr, row, column;
	for (row = y1; row <= y2; row++)
	{
		for (column = x1; column <= x2; ++column)
		{
			pixel_ptr = FPGA_ONCHIP_BASE + (row << 10) + (column << 1);
			*(short *)pixel_ptr = pixel_color;
		}
	}
}

void VGA_clr()
{
	// Clear the character buffer
	int *p;
	for (p = (int*)FPGA_CHAR_BASE; p < (int*)FPGA_CHAR_END; ++p)
		*p = 0;
}

void VGA_text(char * text_ptr, int x, int y) //Display text
{
	int offset;
	volatile char * character_buffer = (char *) FPGA_CHAR_BASE;

	offset = (y << 7) + x;
	while ( *(text_ptr))
	{
		*(character_buffer + offset) = *(text_ptr);
		++text_ptr;
		++offset;
	}
}

// ISR for MPCORE_PRIV_TIMER_IRQ (interrupt ID = 29)
void master_clock_ISR( void )
{
	pidLoopCounter++;
	sumVoltages += (*(ADC_ptr) & 0xFFF) - ADC_OFFSET;	// read 12 bits of data from ADC
	numVoltages++;

	if (pidLoopCounter == pidLoopTime)
	{
		RPM_actual = Get_RPM();
		pulseCount = 0;
		sumRPMs += RPM_actual;
		numRPMs++;

		double pidOutput = UpdatePID(&pidObj, (RPM_input - RPM_actual) * 100 / 350.0);
		if (pidOutput < 0.0)
			pwmPercent = 0.0;
		else if (pidOutput > 100.0)
			pwmPercent = 100.0;
		else
			pwmPercent = pidOutput;

		if (RPM_input > 250)		pidLoopTime = 30;
		else if (RPM_input > 100)	pidLoopTime = 40;
		else if (RPM_input > 50)	pidLoopTime = 100;
		else						pidLoopTime = 200;
		pidLoopCounter = 0;
	}

	if (numVoltages == 1000)	// if 1000 Voltages have been read
	{
		int avgVoltage = sumVoltages / 1000;	// average the values read over 1 second
		sumVoltages = 0;
		numVoltages = 0;

		motorCurrent = (avgVoltage * 1000) / (float)(2 * 3650); // V_O = I_S * R_S * R_L / 1k
		/*if (motorIdle > (1000 / pidLoopTime))		// Display 0 mA if no pulses read for 1 second
			motorCurrent = 0;*/
		shouldUpdate = 1;
	}

	*(MPcore_private_timer_ptr + 2) |= 0x1; // clear timer interrupt flag
	return;
}

// ISR for HPS_TIMER0_IRQ (interrupt ID = 199)
void pwm_timer_ISR( void )
{
	Set_PWM(pwmPeriod, pwmPercent);
	*(HPS_timer0_ptr + 3); // clear timer interrupt flag
	return;
}

// ISR for JP2_IRQ (interrupt ID = 84)
void gpio_ISR( void )
{
	pulseCount++;
	motorIdle = 0;

	*(JP2_GPIO_ptr + 3) |= 0x2;  // clear GPIO interrupt flag
	*(HPS_timer1_ptr + 2) |= 0x1;  // mode = interrupt = 1, mode = 0, enable = 1
	return;
}

// ISR for KEYS_IRQ (interrupt ID = 73)
void pushbutton_ISR( void )
{
	int press = *(KEY_ptr + 3);	// read the pushbutton interrupt register
	*(KEY_ptr + 3) = press;		// Clear the interrupt

	if (press & 0x1)			// KEY0 was pressed
	{
		RPM_input = 0;
		*HEX5_HEX4_ptr = 0;
		*HEX3_HEX0_ptr = 0x3F;	// Display 0 for RPM input
	}
	return;
}
