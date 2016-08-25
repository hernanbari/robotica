#include <Encoder.h>

#define SIMULACION 1

/* Definiciones de pines */
#define PIN_MOT_A 7
#define PIN_MOT_B 6
#define PIN_ENC_A 20
#define PIN_ENC_B 21

#if SIMULACION
#define analogWrite analogWriteSim
#else
Encoder enc(PIN_ENC_A, PIN_ENC_B);
#endif

/* Variables globales */
long prev_pos = 0;  // pos anterior del encoder
float tiempo_anterior_control_loop;

float I_error = 0;
double pid_input, pid_output, pid_setpoint, pid_last_input;

/*constantes globales pid */
#define PWM_MIN 50
#define PWM_MAX 205
#define PID_P  20
#define PID_I  20
#define PID_D  30
#define CICLO_CONTROL_TIEMPO 10 // milisegundos

void setup()
{ 
  // seteo serial
  Serial.begin(115200);
   
  pid_setpoint = 2;
  pid_last_input = 0;
  tiempo_anterior_control_loop = millis();
  


 // inicializo los pines del motor
  pinMode(PIN_MOT_A, OUTPUT);      // inicializo el pin A del motor como una salida
  pinMode(PIN_MOT_B, OUTPUT);      // inicializo el pin B del motor como una salida
  
  // seteo en 0 la velocidad del motor
  analogWrite(PIN_MOT_A,0);        
  analogWrite(PIN_MOT_B,0);
  
  Serial.print("setpoint: "); Serial.print(pid_setpoint); 
  Serial.print(" p: "); Serial.print(PID_P);
  Serial.print(" i: "); Serial.print(PID_I);
  Serial.print(" d: "); Serial.println(PID_D);
}

/****************************** CICLO PRINCIPAL ***************************/

void loop()
{
  #if SIMULACION
  actualizar_simulacion();
  #endif

  ciclo_control();  
}

/***************************** CICLO DE CONTROL **********************/

void ciclo_control()
{
  float tiempo_control = millis();
  int delta_t = tiempo_control  - tiempo_anterior_control_loop;
  if (delta_t >= CICLO_CONTROL_TIEMPO)
  {
    tiempo_anterior_control_loop = tiempo_control;

    float velocidad = calcular_velocidad(delta_t);
    pid_input=velocidad;
    /************* AGREGUE EL CODIGO AQUI **************/
    double P_error=pid_setpoint-pid_input;
    double D_error=pid_input-pid_last_input; //integral del error
    I_error=(I_error+P_error); // posible saturacion
    pid_output=PID_P*P_error+PID_I*I_error+PID_D*D_error;
    int pwm = pid_output; // escribir en esta variable el pwm que se manda al motor (como valor positivo/negativo)
    set_motor_pwm(pwm);
    pid_last_input=pid_input;
    Serial.print("vel: "); Serial.print(velocidad,5); Serial.print(" pwm: "); Serial.print(pwm); Serial.print(" t: "); Serial.print(tiempo_control,5);Serial.print(" delta_t: "); Serial.println(delta_t);
  }  
}


/************************* FUNCIONES AUXILIARES ***********************************/

float calcular_velocidad(double delta_t)
{
  /************* AGREGUE EL CODIGO AQUI **************/
  long dif_pos=0;
  long act_pos;
  float resultado=0;
  act_pos=encoder_position();
  dif_pos=act_pos-prev_pos;
  prev_pos=act_pos;
  resultado=(dif_pos*360)/480;
  
  return resultado/delta_t; // modificame
  //return 3.5;
}

void set_motor_pwm(int pwm)
{

  if(pwm>0&&pwm<PWM_MAX){
    analogWrite(PIN_MOT_B,pwm+PWM_MIN);
    analogWrite(PIN_MOT_A,0);
  }
  else if(pwm<0&&pwm>(-PWM_MAX)){
    analogWrite(PIN_MOT_B,0);
    analogWrite(PIN_MOT_A,-pwm+PWM_MIN);
  }
  else{
    analogWrite(PIN_MOT_A,0);
    analogWrite(PIN_MOT_B,0);
  }
}

long encoder_position(void)
{
  #if SIMULACION
  return enc_read_sim();
  #else
  return enc.read();
  #endif  
}

/********************************** funciones de simulacion ****************************/

#define SIM_VEL_MIN 0.1 // velocidad minima en la que el motor empieza a moverse
#define SIM_MAX_PWM_QUIETO 50  // pwm maximo para el cual no se mueve

/* Variables globales */
int sim_pwmA = 0;    // asociado al pin motA
int sim_pwmB = 0;    // asociado al pin motB
float sim_vel = 0;   // vel actual
float sim_pos = 0;   // pos actual
int sim_pwm = 0;
float sim_t = 0;
float sim_t_prev = 0;

void actualizar_simulacion()
{
  sim_t = micros() / 1000.00;
  
  sim_pwm = -sim_pwmA + sim_pwmB;
  unsigned long dtsim= sim_t - sim_t_prev;
  if(dtsim == 0) return;
  sim_t_prev = sim_t;
  float alpha1=0.01823;       
  float alpha2=-0.2583;      
  long tau=30*dtsim;
  float NI = 0;
  if(fabs(sim_vel) > SIM_VEL_MIN){  //velocidad dist de 0    
    NI=(sim_vel - alpha2*copysign(1, sim_vel))/tau;
  }
  else{ 
    if(sim_pwm >= SIM_MAX_PWM_QUIETO) NI = alpha1 * SIM_MAX_PWM_QUIETO/tau;
    else if (sim_pwm <= -SIM_MAX_PWM_QUIETO) NI = -alpha1 * SIM_MAX_PWM_QUIETO/tau;
    else{
      sim_vel = 0; 
      NI = alpha1*sim_pwm/tau;
    }
  }       
  //Euler
  float F = -NI+(alpha1*sim_pwm)/tau;
  sim_vel = sim_vel + float(F * dtsim);
  sim_pos = (sim_pos+sim_vel*dtsim);
}

long enc_read_sim()
{
  return sim_pos;
}

void analogWriteSim(int pin, int pwm)
{
  if(pin == PIN_MOT_A) sim_pwmA = pwm;
  if(pin == PIN_MOT_B) sim_pwmB = pwm;  
}


