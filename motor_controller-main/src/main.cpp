#include "SimpleCAN.h"
#include "CANHandler.h"
#include <Arduino.h>
#include <SimpleFOC.h>
#include <math.h>


#define ID 7
// The actual CAN bus class, which handles all communication.

CANHandler CANDevice(CreateCanLib(),ID);

// currentsensing 
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
//



// hall sensor instance
HallSensor sensor = HallSensor(A_HALL1, A_HALL2,A_HALL3,40);
BLDCMotor motor = BLDCMotor(40);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH,A_PHASE_VL,A_PHASE_WH,A_PHASE_WL);

// Interrupt routine intialisation
// channel A, B and C callbacks
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}



// funktion umwandlung von NTC zu temperatur wird weiter unten aufgerufen 

static float Ntc2TempV(float ADCVoltage) 
{
	// Formula: https://www.giangrandi.org/electronics/ntc/ntc.shtml
	const float ResistorBalance = 4700.0;
	const float Beta  = 3425.0F;
	const float RoomTempI = 1.0F/298.15F; //[K]
	const float Rt = ResistorBalance * ((3.3F / ADCVoltage)-1);
	const float R25 = 10000.0F;
	
	float T = 1.0F/((log(Rt/R25)/Beta)+RoomTempI);
	T = T - 273.15;

	return T;
}


void setup() 
{
  
  pinMode(PC6, OUTPUT);
  digitalWrite(PC6, HIGH);
	SimpleFOCDebug::enable();
	Serial.begin(115200);
	delay(1000);
    Serial.println("setup");
    sensor.init();
    sensor.enableInterrupts(doA,doB,doC);

	CANDevice.Init();

	// Set bus termination on/off (may not be available on all platforms).
	CANDevice.Can1->SetBusTermination(true);


	
  driver.voltage_power_supply = 24;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
//***************************************
  current_sense.linkDriver(&driver);
  // aligning voltage [V]
  motor.voltage_limit = 24;
  motor.velocity_limit =40;
  motor.voltage_sensor_align = 4;
 
  motor.linkSensor(&sensor);

 

  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_TARGET| _MON_VEL|_MON_ANGLE|_MON_CURR_D|_MON_CURR_Q|_MON_VOLT_D|_MON_VOLT_Q;
  motor.monitor_downsample = 500;


 // PID getuned für geschwindigkeit 
   motor.PID_velocity.P =0.2;
   motor.PID_velocity.I =10.0;
  //motor.PID_velocity.D =0;
   motor.LPF_velocity.Tf= 0.005;

  //PID getunt für winkel
    motor.LPF_angle.Tf =0.2;
    motor.P_angle.P=1;
    motor.P_angle.I=0;
    motor.P_angle.D=0;

  //ramping 
  motor.P_angle.output_ramp = 20; 
  
  // set motion control loop to be used
  motor.controller = MotionControlType::angle;
  motor.init();
  current_sense.init();
  current_sense.skip_align = true;
  motor.linkCurrentSense(&current_sense);
  motor.initFOC();
  for(int i=0;i<max_device_count;i++){
    for(int j=0;j<count_target_parameter;j++){
     CANDevice.target_values[i][j]=0;
    }
  }
  for(int i=0;i<max_device_count;i++){
    for(int j=0;j<count_motor_values;j++){
      CANDevice.motor_values[i][j]=0;
    }
  }
}


void loop()
{
// motor.monitor();
float Temp = _readADCVoltageInline(A_TEMPERATURE, current_sense.params);
float temp_degC = (Temp - 0.925) / 0.019+25;
PhaseCurrent_s currents = current_sense.getPhaseCurrents();
float current_magnitude = current_sense.getDCCurrent();
static uint32_t LastAction=millis();
motor.move();
if(ID==0){
  if(CANDevice.get_values_serial()){
    motor.move(CANDevice.target_values[0][0]);
    CANDevice.send_target_values();
  }
}else{
	if (LastAction+5000<millis() ){
    CANDevice.motor_values[ID][0]=motor.shaft_angle;
    CANDevice.motor_values[ID][1]=current_magnitude;
  	CANDevice.motor_values[ID][2]=motor.shaft_velocity;
    CANDevice.motor_values[ID][3]=temp_degC;
    CANDevice.send_motor_values();
	  LastAction=millis();
    }
}
motor.loopFOC();
if(CANDevice.full_instruction_recived){
  Serial.printf("Motor moved to:%f\n",CANDevice.target_values[ID][0]);
  motor.P_angle.limit =CANDevice.target_values[ID][1];
  motor.P_angle.output_ramp =CANDevice.target_values[ID][1];

  motor.move(CANDevice.target_values[ID][0]);
  CANDevice.full_instruction_recived=false;
}

CANDevice.Can1->Loop();
}