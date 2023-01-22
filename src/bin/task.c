/* Arduino Uno */

#define OFF 0
#define ON OFF + 1

#include <stdio.h>
#include <stdlib.h>

/*タイマーハンドラ用*/
#include <avr/io.h>
#include <avr/interrupt.h>

void TaskRelay(void *pvParameters);
void TaskPWM(void *pvParameters);
void TaskDisplay(void *pvParameters);

unsigned char i = 0;
#define R 47.0

/* タスクの定義 */
#define THRESHOLD 100
#define RELAY_DELAY 0
#define PWM_DELAY 0
#define fCoeff_P 0.3
#define fCoeff_I 0.4
#define fCoeff_D 2.8
int iTarget = 80;
int iPWM = 128;
float fP_error = 0.0;
float fI_error = 0.0;
float fD_error = 0.0;
float fP_error_previous = 0.0;
bool flg = true;
bool previous_flag = true;

void TaskRelay(void *pvParameters)
{ // Relay control
    (void)pvParameters;
    if (iPWM < THRESHOLD - 1)
        flg = true;
    if (iPWM > THRESHOLD)
        flg = false;
    switch (flg)
    {
    case true:
        digitalWrite(4, HIGH);
        if (previous_flag == false)
        {
            previous_flag = true;
            Serial.println("RELAY ON Control");
        }
        break;
    case false:
        digitalWrite(4, LOW);
        if (previous_flag == true)
        {
            previous_flag = false;
            Serial.println("RELAY OFF Control");
        }
        break;
    }
}
void TaskPWM(void *pvParameters)
{ // PWM control
    (void)pvParameters;
    int iMonitor = analogRead(A1);
    fP_error = fCoeff_P * (float)(iMonitor - iTarget) / 1.5;
    fI_error += fCoeff_I * fP_error;
    fD_error = fCoeff_D * (fP_error - fP_error_previous);
    fP_error_previous = fP_error;
    iPWM -= (int)(fP_error + fI_error + fD_error);
    if (iPWM > 255)
        iPWM = 255;
    if (iPWM < 0)
        iPWM = 0;
    analogWrite(3, iPWM);
    Serial.println("PWM Control");
}
void TaskDisplay(void *pvParameters)
{ // PWM control
    (void)pvParameters;
    Serial.println("D");
}
