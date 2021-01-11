/* Sofrware de emulação do sistema mecânico de hidrogeração
 *  Este software possui a função de emular a dinâmica das turbinas
 *  Realizar o controle de potênica do motor
 *  Relaizar a emulação do sistema das válvulas que realizam a abeturura do distribuidor
 *  
 *  Por: Nei Junior Da Silva Farias
*/
#include "pwm_lib.h"
#include <DueTimer.h>

//Configurando a porta do PWM
  using namespace arduino_due::pwm_lib;
  pwm<pwm_pin::PWMH0_PC3> pwm_pin35;
  pwm<pwm_pin::PWMH1_PA19>pwm_pin42;// Configura o canal 1 de pwm para sair na porta 42
  pwm<pwm_pin::PWMH2_PB14>pwm_pin53;// Configura o canal 1 de pwm para sair na porta 53

//Tempo de interrupção
  float Ts = 0.0001;
//Definindo as portas analógicas dos sensores
 
  volatile int sensorPinVc1   = A3;//Entrada do banco de capacitores do campo
  volatile int sensorPinVc2   = A4;//Entrada do banco de capaciotres da Armadura
  volatile int sensorPina12   = A0;//Entrdada da corrente de armadura
  volatile int sensorpinA5    = A2;//Entrada da corrente de campo
  volatile int sensorpinVab   = A6;//Entrada da tensão Vab
  volatile int sensorpinVbc   = A7;//Entrada da tensão Vbc
  volatile int sensorpinIa    = A8;//Entrada da corrente Ia
  volatile int sensorpinIb    = A9;//Entrada da corrente Ib
  volatile int velan          = A6;//Entrada da velocidade
  volatile int exgs           = A10;//Entrada da velocidade

//Valores das tensões nos capacitores

volatile float Vcam  = 0;//Variável que armazena a tensão de campo
volatile float Vcarm = 0;//Variável que armazena a tensão de armadura
static   float Vminarm = 300;//Tensão mínima do banco de capacitores da armadura, para finalizar o carregamento dos capacitores
static   float Vmincam = 160;//Tensão mínima do banco de capacitores do campo, para finalizar o carregamento dos capacitores
  
// Definindo as variaveis que refebecerão os dados da porta analógica

    int Ia     = 0;// Corrente de armadura
    int Ie     = 0;// Corrente de campo
    int Vc1    = 0;// Capacitor de campo
    int Vc2    = 0;// Capacitor de Armadura
// Váriaveis em PU:
    volatile float Velocidade_PU = 0;//Velocidade do motor em PU
    volatile float Ia_PU[2]         = {0,0}; //Corrente de armadura em PU
    volatile float Ia_PUNF[2]       = {0,0}; // Corrente de armadura não filtrada
    volatile float Ic_mm[2]         = {0,0}; // Corrente de excitação de campo do motor de corrente continua
// Variaveis usadas para o dutty
static float tensaoBase = 206.62; // Tensão de base do motor
static float tensaoDC   = 320;//Tensão do link DC
volatile float dut = 0;// Dutty do conversor DC-DC
volatile float dut2 = 0; // Dutty do con
volatile float periodo = 50000;//Periodo do PWM
volatile float T_altoi = 0;//Tempo em alto inicial
volatile float T_alto  = 0;//Tempo em alto do buck
int i = 1;//indice
static float dutmax = 0.6; // Valor máximo do dut.
static float dutmax2 = 0.05; // Valor máximo do dut.

// Contatoras       
int contatora[15] = {29,25,27,24,22,20,28,26,31,21,33,23,30,32,34};//Pinos utilizados para acionanmento dos relés que acionarão as contatoras
// Velocidade
volatile float Ve[3] = {0,0,0};//Variável que armazena a velocidade em RPM
String Estado_v = "W";//Estring que armazena o estados da máquina de estado utilizada para medir a velocidade
volatile float contando;//Variável que conta o numeros de interrupções durante um giro completo da máquina
volatile float velanVe;//Váriável que armazena o valor da porta analógica
volatile float Vef[3] = {0,0,0};//Váriavel que armazena a velocidade filtrada

// Variáveis da comunicação Serial
char   Entrada_serial;//Char que armazena um char da entrada serial
String STRING_SERIAL = "";//Amrazena os dados da porta serial
String STRING="OI";//Armazena os dados da porta serial para serem usados ao longo do algorítimo
String STRING_ENVIAR="";//È a string enviada para a porta serial
String velocidade;// String Utilizada para enviar a velocidade para a IHM
String Vcampo;//String que armazena a tensão de campo a ser enviada para porta serial
String Varmadura;//String que armazena a tensão de Armadura a ser enviada para porta serial
String Icampo;//String que armazena a Corrente de campo a ser enviada para porta serial
String Iarmadura;//String que armazena a corrente de Armadura a ser enviada para porta serial
String dut_serial;//String que armazena o dutty a ser enviada para porta serial
String dut2_serial;//String que armazena o dutty a ser enviada para porta serial
String STRING_CVe; //String que armazena a saida do controlador de velocidade;
String STRING_Pm; // String que armazena a potênica da turbina hidráulica;
String STRING_C;//Saida do controlador de potência;
String STRING_xg;//Abetura do distribuidor;
String STRING_R_Velocidade;//Referênica da velocidade
String Referencia = "";
int flag_comunicacao = 1;//Variável utilizada para alternar a informação a ser enviada para porta serial
int flag_comunicacao2 = 1;//Variável utilizada para alternar a informação a ser enviada para porta serial

String ESTADO = "D1";//Armazena o estado sistema de emulação da micromáquina

//Emulação da turbina:
volatile float deltaH[2] = {0,0};  // Variação da altura da queda da água com relação a queda inicial;
volatile float Pm        =     0;  // Potência mecânica da Turbina;
volatile float H         =     1;  // Altura da queda água;
volatile float Unl       = 0.068;  // Velocidade da água ao passar pela turbina sem carga;
volatile float H0        =     1;  // Altura inicial da queda da água;
volatile float gmin      =  0.06;  // Abertura mínima do distribuidor;
volatile float gmax      =  0.96;  // Abertura máxima do distribuidor;
volatile float At        = 1/(gmax-gmin);  // Ganho da turbina;
volatile float g         =   0.0;  // Posição do servo posicionado;
volatile float Tw        =  1.41;  // Constante de tempo da turbina;
volatile float G         =  0;//Abertura do distribuidor
volatile float U[2]      =  {0,0}; //Altura da queda dágua

//Servoposicionador
static float                  Tp = 0.016;//Constante de tempo entre a válvula ditribuida e servo posicionado
static float                  Tg = 2.8;
volatile float                xg[2] = {0,0};
volatile float                xp[2] = {0,0};
volatile float                un[2] = {0,0};//sinal de referência
volatile float                u     = 0;// Sinal do controlador
static float                  up1 = 0.25;
static float                  up2 = 0.20;
//Controlador de potência

volatile float C[2] = {0,0};
volatile float E[2] = {0,0};
static float      a = 12.783896103896108;
static float      b = 3699.695767856018e+03;
volatile float Pm_pu = 0;

//Regulador de velocidade:
volatile float R_Velocidade = 1;
volatile float R4[2]     = {0,0};
volatile float CF_P[2]   = {0,0};
volatile float CVe[4]    = {0,0,0,0};
volatile float EVe[4]    = {0,0,0,0};
static float Bp  = 0.05;
static float Tiv = 1/0.095;
static float Kp  = 1/(1/0.095-0.05);
static float Tsv = 0.001;
static float a3 = -0.346530399015076;
static float a2 = 1.032534251890936;
static float a1 = -1.025203641664915;
static float a0 = 0.339203454239001;
static float b2 = -2.939309206313322;
static float b1 = 2.882459023169933;
static float b0 = -0.943149816856611;
volatile float C_S_S = 0;
int R = 0;
int Z = 0;
// Regulador de tensão

volatile float V_ter    =  220;// tensão terminal
volatile float u_rat    =  50; // sinal de controle do rat
volatile float V_sg     =  220;// tensão do capacitor de campo do sg
volatile float V_sgmin  =  160;// tensão minima do capacitor de campo para finalizar o carregamento
volatile float Vt_ref   =  200;// tensão de referência
volatile float P_e      =  0;// potência elétrica
volatile float I_arsg   =  10;// corrente de armadura
volatile float I_casg   =  10;// corrente de campo
volatile float Vb_fase  = 10; // fase da tensão do barramento
volatile float V_bus    = 220; // tensão do barramento
volatile float F_bus    = 60; // frequência do barramento
volatile float dut3     = 0;
static float dutmax3  = 0.1;
// Sistema interligado;

volatile float dP_e = 0; // Variação da potência elétrica
volatile float PSS_c = 0; // sinal de controle do PSS

// Valores RMS------------------------------------------------
volatile int   count_rms_flag = 1;
volatile float Vab_ins = 0; // Valor instantâneo da tensão de linha Vab
volatile float Vcb_ins = 0; // Valor instantâneo da tensão de linha Vbc.
volatile float Vab_rms = 0; // Valor eficaz da tensão vab 
volatile float Vcb_rms = 0; // Valor eficaz da tensão vcb
volatile float Ia_rms  = 0; // Valor eficaz da corrente Ia
volatile float Ic_rms  = 0; // Valor eficaz da corrente Ic
volatile float Ia_ins  = 0; // Valor instantâneo da tensão de linha Ia
volatile float Ic_ins  = 0; // Valor instantâneo da tensão de linha Ib
volatile float I_rms   = 0; // Corrente de armadura RMS
volatile float V_rms   = 0; // Tensão do terminal de armadura RMS
// volatile float V_rms_flag      = 0;
// volatile float I_rms_flag      = 0;
volatile float P_ativa           = 0;
volatile float P1_flag           = 0;
volatile float P2_flag           = 0;
volatile int per_rms             = 167;
volatile float V1_flag[2]        = {0,0};
volatile float V2_flag[2]        = {0,0};
volatile float sin_fase          = 0; // zero atrasado e 1 adiantado
volatile float V_rms_flagab[167]  = {0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0};
volatile float V_rms_flagcb[167]  = {0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0};
volatile float I_rms_flaga[167]   =  {0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0};
volatile float I_rms_flagc[167]   =  {0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0};
volatile float P1[167]            =  {0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0};
volatile float P2[167]            =  {0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0};
//---Divisor de frequência digital
volatile int count = 1;
volatile int divfreq = 10;
volatile int sin_est = 2;
int pin_len       = 4;
int pin_rap       = 5;
int pin_sin       = 6;
// Acionar Cargas

volatile int n_cargas = 0;
  void setup() 
{
   SerialUSB.begin(2000000);
   analogReadResolution(12);
   pinMode(contatora[0 ],OUTPUT);
   pinMode(contatora[1 ],OUTPUT);
   pinMode(contatora[2 ],OUTPUT);
   pinMode(contatora[3 ],OUTPUT);
   pinMode(contatora[4 ],OUTPUT);
   pinMode(contatora[5 ],OUTPUT);
   pinMode(contatora[6 ],OUTPUT);
   pinMode(contatora[7 ],OUTPUT);
   pinMode(contatora[8 ],OUTPUT);
   pinMode(contatora[9 ],OUTPUT);
   pinMode(contatora[10],OUTPUT);
   pinMode(contatora[11],OUTPUT);
   pinMode(contatora[12],OUTPUT);
   pinMode(contatora[13],OUTPUT);
   pinMode(contatora[14],OUTPUT);
   pinMode(19,INPUT);
   pinMode(pin_len,INPUT);
   pinMode(pin_rap,INPUT);
   pinMode(pin_sin,INPUT);
   pwm_pin35.start(periodo,T_altoi);
   pwm_pin42.start(periodo/2,T_altoi);
   pwm_pin53.start(periodo/2,T_altoi);
   Timer1.attachInterrupt(micromaquina).start(Ts*1000000);//microsegundos
   while(1)
   {}

}
void loop() {}

//------------------------------------------------Leitura Serial-----------------------------------------------------
  void Leitura_Serial()
{
  
  while(SerialUSB.available()>0)
  {

    Entrada_serial=(byte)SerialUSB.read();
    
    if(Entrada_serial!='\n')
    {
    STRING_SERIAL = STRING_SERIAL + Entrada_serial;
    }
    R = STRING_SERIAL.indexOf('R');
    Z = STRING_SERIAL.indexOf('Z');
    if(STRING_SERIAL.charAt(R)=='R' && STRING_SERIAL.charAt(Z) == 'Z')
    {
      STRING = "";
      for (i = R+1; i < Z;++i)
      {
    STRING = STRING + STRING_SERIAL[i];
      }
    }
  }
  STRING_SERIAL = "";
  }


//  switch (flag_comunicacao)
// {
//  
//  case 0:
//      STRING_R_Velocidade = String(R_Velocidade*1000,0);
//      STRING_ENVIAR = 'Z' + STRING;
//      flag_comunicacao = 0;
//      break;
//  case 1:
//      Varmadura = String (Vc2);
//      STRING_ENVIAR = 'B' + Varmadura+ "}";
//      flag_comunicacao = 2;
//      break;
//  case 2:
//      Iarmadura = String(Ia);
//      STRING_ENVIAR = 'C' + Iarmadura+ "}";
//      flag_comunicacao = 4;
//      break;
//  case 4:
//      velocidade = String(Ve[0],0);
//      STRING_ENVIAR = 'E' + velocidade+ "}";
//      flag_comunicacao = 5;
//      break;
//  case 5:
//      dut_serial = String(dut*1000,0);
//      STRING_ENVIAR = 'F' + dut_serial+ "}";
//      flag_comunicacao = 6;
//      break;
// case 6:
//      STRING_ENVIAR = 'G' + ESTADO+ "}";
//      flag_comunicacao = 7;
//      break;
//case 7:
//      STRING_CVe = String(u*1000,0); //String que armazena a saida do controlador de velocidade;
//      STRING_ENVIAR = 'H' + STRING_CVe+ "}";
//      flag_comunicacao = 8;
//      break;
//case 8:
//      STRING_Pm  = String(Pm*1000,0); // String que armazena a potênica da turbina hidráulica;
//      STRING_ENVIAR = 'I' + STRING_Pm+ "}";
//      flag_comunicacao = 9;
//      break;
//case 9:
//      STRING_C   = String(xp[0]*1000,0);//posição da válvula piloto;
//      STRING_ENVIAR = 'J' + STRING_C+ "}";
//      flag_comunicacao = 10;
//      break;
//case 10:
//      STRING_xg  = String(xg[0]*1000,0);//Abetura do distribuidor;
//      STRING_ENVIAR = 'K' + STRING_xg + "}";
//      flag_comunicacao = 0;
//      break;      
//      default:
//      flag_comunicacao = 0;
// }
//  SerialUSB.println(STRING_ENVIAR);
//
// }
//----------------------------------------------------controlador de potencia do emulador-----------------------------------------------
 void controle_pmcc() // Freqência de chaveamento 0.001
{
    if (0.333<Velocidade_PU)
    {
      Pm_pu = Velocidade_PU*Ia_PU[0];
    }
    else
    {
      Pm_pu = 0.333*Ia_PU[0];
    }
    E[0] = Pm-Pm_pu;
    
    
    C[0] = C[1]+1.200014598519459*E[0] - 1.140764622259762*E[1];
    if (C[0]<0)
    {
      C[0] = 0;
    }
    else if(C[0]>1.5326560)
    {
      C[0] = 1.5326560;
    }
    else if (C[0] > C[1]+0.1*tensaoBase/tensaoDC)
    {
      C[0] = C[1];
    }
    E[1] = E[0];
    C[1] = C[0];
}
//----------------------------------------Controledor PWM turbina -------------------------------------------------------------------
void  Controle_PWM()
  {
   // dut = C[0]*tensaoBase/tensaoDC;
   dut = 0.6;
if (dut < (dutmax))
{
    T_alto = (dut)*(periodo);
    
}
else
{
  T_alto = (periodo*dutmax);
  dut = dutmax;
}
    pwm_pin35.set_duty(periodo*0.05);//Armadura
    pwm_pin42.set_duty((periodo*0.69)/2);//Campo do motor.
    pwm_pin53.set_duty(0);//campo do gerador
}
//----------------------------------------Funcao para salva as medidas realizadas pelos sensosres-------------------------------------
void sensores()
{ 
  Ic_mm[0] = 1; 
  Vc2         = analogRead(sensorPinVc2);
  Vcam = Vc2/11.442+32.045/11.442;
  Vc1         = analogRead(sensorPinVc1);
  Vcarm = Vc1/11.494+28.836/11.494;
  Ia= analogRead(sensorPina12);
  Ia_PUNF[0]  = Ia/(11*381.54);
  Ia_PU[0]    = Ia/(11*381.54);
  Ia_PU[1]    = Ia_PU[0];
  Ie          = analogRead(sensorpinA5);
  Valor_RMS();
  velocidade2();   
//    Vcam = analogRead(sensorPinVc1)/10.762;
//    Vcarm =  analogRead(sensorPinVc2)/10.762; 
//    Ia     = analogRead(sensorPina12);//Dividir por 381.414 para obter valor real
//    Ia_PUNF[0]  = Ia/(11*381.54);
//    Ia_PU[0] = Ia/(11*381.54);
//    Ia_PU[1] = Ia_PU[0];
//    Ie     = analogRead(sensorpinA5);// Dividir por 6601.923 para obter valor real
//    Vc1    = analogRead(sensorPinVc1);// Dividir por 10.762 para obter valor real
//    Vc2    = analogRead(sensorPinVc2);// Dividir por 10.762 para obter valor real  

}
//--------------------------------------Medidor de velocidade----------------------------------------------------------------------
void velocidade2()
  {
    if (Estado_v == "W")
    {
      if (digitalRead(19) == HIGH)
      {
        Estado_v = "H";
        
      }
      }
    if (Estado_v == "H" )
    {
      if (digitalRead(19) == LOW)
      {
        Estado_v = "L";
        contando = contando -1;
      }
      contando = contando + 1;
    }
    if (Estado_v == "L")
    {
      if (digitalRead(19) == HIGH)
      {
        Estado_v = "H";
        
    Ve[0] = (1/(Ts*contando))*60.0;
    
    contando = 0;
      }
      contando = contando +1;
    }
    //if (Ve[0] > Ve[1]*1.5 && Ve[0]>200)
    //{
    //  Ve[0] = Ve[1];
    //}else if (Ve[0] > Ve[1]*1.2 && Ve[0]>800)
    //{
    //  Ve[0] = Ve[1];
    //}
    //else if (Ve[0] > Ve[1]*1.1 && Ve[0]>1000)
    //{
    //  Ve[0] = Ve[1];
    //}
    //Vef[0] = 0.045004500450045*Ve[0]+0.045004500450045*Ve[1]+0.909990999099910*Vef[1];
    Ve[1] = Ve[0];
    Velocidade_PU = Ve[0]/1800;// cálculo da velocidade em P.U. atualizar para determinar a velocidade correta
    //Velocidade_PU = 1.1; //utilizado para teste
  }
 //Sistema emulado
void micromaquina()
{
  // Leitura dos sensores
  //velocidade2();
  sensores();
  // Divisor de frequência do emulador
 if (count < divfreq)
    {count = count+1;}
 else
 {
  count = 1;
  Leitura_Serial();
  maq_estados();
  if (ESTADO=="D1") // Ações do sistema desligado
  {
    funcao_D1();
  }
  else if (ESTADO == "E1")
  {
    funcao_E1();
  }
  else if (ESTADO == "E2")
  {
   funcao_E2(); 
  }
  else if (ESTADO == "E3")
  {
   funcao_E3(); 
  }
  else if (ESTADO == "E4")
  {
   funcao_E4(); 
  }
  else if (ESTADO == "V1")
  {
   funcao_V1(); 
  }
  else if (ESTADO == "V2")
  {
   funcao_V2(); 
  }
  else if (ESTADO == "V3")
  {
   funcao_V3(); 
  }
  else if (ESTADO == "T1")
  {
   funcao_T1();
  }
  else if (ESTADO == "T2")
  {
   funcao_T2();
  }
  else if (ESTADO == "T3")
  {
   funcao_T3();
  }
  else if (ESTADO == "T4")
  {
   funcao_T4();
  }
  else if (ESTADO == "S1")
  {
   funcao_S1();
  }
  else if (ESTADO == "S2")
  {
   funcao_S2(); 
  }
  else if (ESTADO == "S3")
  {
   funcao_S3();
  }

  escrita_serial();
 }
     
}
//------------------------------------------------Maquina de estado do sistema de emulação-------------------------------------------------
void maq_estados()
{
  // Divisor de frequência
  if (ESTADO=="D1") // Desligado
  {
    if (STRING == "LM")
    {
      ESTADO = "E1";
    }
  }
  else if (ESTADO=="E1")//Carregar capacitores
  {
    if (Vcam > Vmincam && Vcarm > Vminarm)
    {
      ESTADO = "E1A";
    }
  }
  else if (ESTADO == "E1A" && STRING == "FM")
  {
      ESTADO = "E2";
  }
  else if (ESTADO=="E2")//Capcitores carregados
  {
   if (STRING == "EB")
   {
    ESTADO = "E3";
   }
  }
  else if (ESTADO=="E3")//Energizar buck
  {
    if (STRING == "EM")
    {
      ESTADO = "E4";
    }
  }
  else if (ESTADO=="E4")//Energizar motor
  {
    if (Ic_mm[0] > 0.1)
    {
      ESTADO = "E4A";
    }
  }
  else if (ESTADO == "E4A" && STRING == "IH")
    {
      ESTADO = "V1";
    }
  else if (ESTADO=="V1")//Iniciar partida
  {
    if (Velocidade_PU > 0.8)
    {
      ESTADO = "V2";
    }
  }
  else if (ESTADO=="V2")//V > 0.8
  {
    if (Velocidade_PU >= 1)
    {
      ESTADO = "V3";
    }
    
  }
  else if (ESTADO=="V3")//Iniciar RV V >= 1
  {
    if (STRING == "LE")
    ESTADO = "T1";
  }
  else if (ESTADO=="T1")//Iniciar SRT
  {
    if (V_sg > V_sgmin)
    {
      ESTADO="T1A";
    }
  }
  else if (ESTADO=="T1A")//Iniciar SRT
  {
    if (STRING == "FE")
    {
      ESTADO="T2";
    }
  }
  else if (ESTADO=="T2")//Carregar capacitores
  {
    if (STRING == "EE")
    {
      ESTADO="T3";
    }
  }
  else if (ESTADO=="T3")//Energizar buck SG
  {
     if (STRING == "IG")
    {
      ESTADO="T4";
    }
  }
  else if (ESTADO=="T4")//Iniciar RAT
  {
    if (STRING == "SM")
    {
      ESTADO = "S1";
    }
  }
  else if (ESTADO=="S1")//Iniciar sicronismo
  {
    //Realiza a rotina para sincronizar a tensão do barramento com a tensão da máquina
    ESTADO = "S1A";
  }
  else if (ESTADO == "S1A")
  {
    sincron_rede();
    acionar_cargas();
    if (STRING == "CB" && sin_est == 1)
    {
      // Conectar ao barramento
      ESTADO = "S2";
    }
  }
  else if (ESTADO=="S2")
  {
     if (STRING == "IP")
     {
         ESTADO="S3";
     }
  }
  else if (ESTADO=="S3")
  {
    // Rotina do PSS
  }
if (STRING=="DM")
{
  ESTADO ="D1";
}
}

//-------------------------------------------Dinâmicas da turbinha hidráulica emulada-----------------------------------


//--------------------------------------------Dinâmica do servoposicionador---------------------------------------------

void servo_posicionador()
{
  //Dinâmica do Servoposicionador
  un[0] = u;
  xg[0] = xg[1]+(Ts/(2*Tg))*(xp[0]+xp[1]);
  xp[0] = ((2*Tp-Ts)/(2*Tp+Ts))*xp[1]+(Ts/(2*Tp+Ts))*((un[0]-xg[0])+(un[1]-xg[1]));
  //Implementando abertura máxima e mínima do distribuidor
  if(xg[0]>gmax)
  {
    xg[0] = gmax;
  }
  if(xg[0] < gmin)
  {
    xg[0] = gmin;
  }
  //Atualizando valores
  xp[1] = xp[0];
  xg[1] = xg[0];
  un[1] = un[0];

}

//-------------------------------------------Dinâmica da turbina hidraulica-----------------------------------------------------

void Turbina()

{
  At = 1/(gmax-gmin);

  g = xg[0];

  if (g > gmax)
  {
    g = gmax;
  }
  
  if (g < gmin)
  {
    g = gmin;
  }
  G         =  At*g;
  H         = (U[0]/G)*(U[0]/G);
  deltaH[0] =  H0 - H; 
  U[0]      =  U[1] + (Ts/(2*Tw))*(deltaH[0]+deltaH[1]);
  Pm        =  H*(U[0]-Unl);
  if (Pm > 1)
  {
    Pm = 1;
  }
  if (Pm<0)
  {
    Pm = 0;
  }
  U[1]      =  U[0];
  deltaH[1] =  deltaH[0];
}
//---------------------------------Pss function---------------------------------------------------

void PSS_controller()
{
  // Adicionar pss desejado
}

//---------------------------------Regulador de velocdidade---------------------------------------
void RV()
{ if (STRING[0] == 'V')
      {
    for(i = 1; i<=STRING.length();++i)
    {
      STRING_SERIAL = STRING_SERIAL + STRING[i];
    }
    R_Velocidade = STRING_SERIAL.toFloat()/1000;
    //STRING_SERIAL.toFloat();
    STRING_SERIAL = "";
      }
  EVe[0] = R_Velocidade-Velocidade_PU;
  CVe[0] = -b2*CVe[1]-b1*CVe[2]-b0*CVe[3]+a3*EVe[0]+a2*EVe[1]+a1*EVe[2]+a0*EVe[3];
  u = CVe[0];
  CVe[3] = CVe[2];
  CVe[2] = CVe[1];
  CVe[1] = CVe[0];

  EVe[3] = EVe[2];
  EVe[2] = EVe[1];
  EVe[1] = EVe[0];
}

/*void RV()
{
  if (STRING[0] == 'C' && STRING[1] == 'F')
  {
    STRING_SERIAL = "";
    for(i = 2; i<=STRING.length();++i)
    {
      STRING_SERIAL = STRING_SERIAL+ STRING;
    }
    CF_P[0] = STRING_SERIAL.toInt()/1000;
    STRING_SERIAL = "";
  }
  EVe[0] = R_Velocidade-Velocidade_PU;
  R4[0]  = CF_P[0]*Bp+EVe[0];
  CVe[0] = -(2*Tiv/Tsv)*CVe[1]-(CVe[1]*Bp-R4[1])*(Tsv-2*Tiv)/Tsv+R4[0]*(Tsv+2*Tiv)/(Tsv);
  u = CVe[0];
  CVe[1]  = CVe[0];
  EVe[1]  = EVe[0];
  R4[1]   = R4[0];

}*/
//--------------------------------Regulador de tensão------------------------------------------------------------------
void RAT() // Freqência de chaveamento 0.001
{  
    // Colocar RAT

    
dut3 = u_rat/tensaoDC;
if (dut3 < (dutmax3))
{
    T_alto = (dut3)*(periodo);
}
else
{
  T_alto = (periodo*dutmax3);
}
    pwm_pin42.set_duty(T_alto);
}
//-------------------------------Excitação da máquina---------------------------------------

void excitacao_mcc() // Controle de corrente de excitação 
{
    dut2 = 220/tensaoDC;
if (dut2 < (dutmax2))
{
    T_alto = (dut2)*(periodo);
}
else
{
    T_alto = (periodo*dutmax2);
}
    pwm_pin53.set_duty(T_alto);
}
//-------------------------------Emulação da turbina hidraulica sem regulador de velocidade----------------------------
void emulador_tub()
{
  servo_posicionador();
  Turbina();
  controle_pmcc();
  Controle_PWM();  
}


//------------------------------------------Escrita da porta serial----------------------------------------------------
void escrita_serial()
 {
  if (ESTADO[0] == 'E'||ESTADO[0] == 'V')
  {
 switch (flag_comunicacao)
  { 
  case 1:
      STRING_ENVIAR = 'X' + String(Velocidade_PU*1000,0);// Velocidade do motor de corrente contínua
      flag_comunicacao = 2;
      break;
  case 2:
      STRING_ENVIAR = 'H' + String(xg[0]*1000,0);// Abertura do didtribuidor
      flag_comunicacao = 3;
      break;
  case 3:
      STRING_ENVIAR = 'I' + String(u*1000,0);// Tensão do capacitor de armadura
      flag_comunicacao = 4;
      break;
  case 4:
      STRING_ENVIAR = 'S' + String(Pm*1000,0);// Potência mecânica do motor de corrente contínua
      flag_comunicacao = 5;
      break;
  case 5: 
      flag_comunicacao = 1;
      switch (flag_comunicacao2)
      { 
        case 1:
        STRING_ENVIAR = 'A'+String(Ve[0]);// Corrente de armadura
        flag_comunicacao2 = 2;
        break;
        case 3:
        STRING_ENVIAR = 'Z'+ESTADO;// Estado da máquina de estado do sistema
        flag_comunicacao2 = 4;
        break;
        case 4:
        STRING_ENVIAR = 'C'+String(Vc1);// Tensão do capacitor de campo
        flag_comunicacao2 = 5;
        break;
        case 5:
        STRING_ENVIAR = 'D'+String(Vc2);// Tensão do capacitor de armadura
        flag_comunicacao2 = 6;
        break;
        case 6:
        STRING_ENVIAR = 'E'+String(dut*1000,0);// Dut da armadura do motor cc
        flag_comunicacao2 = 7;
        break;
        case 7:
        STRING_ENVIAR = 'F'+String(dut2*1000,0);// Dut do campo do motor cc
        flag_comunicacao2 = 8;
        break;
        case 8:
        STRING_ENVIAR = 'J'+String(V_ter,0);// Tensão do capacitor de campo
        flag_comunicacao2 = 9;
        break;
        case 9:
        STRING_ENVIAR = 'K'+String(u_rat*1000,0);// Sinal de controle do rat
        flag_comunicacao2 = 10;
        break;
        case 10:
        STRING_ENVIAR = 'L'+String (V_sg,0);// Tensão da armadura do gerador
        flag_comunicacao2 = 11;
        break;
        case 11:
        STRING_ENVIAR = 'M'+String(Vt_ref,0);// Tensão de referência do rat
        flag_comunicacao2 = 12;
        break;
        case 12:
        STRING_ENVIAR = 'N'+String(P_e*1000,0);// Potência elétrica do gerador
        flag_comunicacao2 = 13;
        break;
        case 13:
        STRING_ENVIAR = 'O'+String(V_bus,0);// Tensão do barramento infinito
        flag_comunicacao2 = 14;
        break;
        case 14:
        STRING_ENVIAR = 'P'+String(dP_e*1000,0);// Variação da potência do barramento infinito
        flag_comunicacao2 = 15;
        break;
        case 15:
        STRING_ENVIAR = 'R'+String(PSS_c*1000,0);// Sinal de controle do PSS_c
        flag_comunicacao2 = 16;
        break;
        case 16:
        STRING_ENVIAR = 'T'+String(I_arsg,0);// Corrente de armadura do gerador
        flag_comunicacao2 = 17;
        break;
        case 17:
        STRING_ENVIAR = 'U'+String(I_casg,0);// Corrente de campo do gerador
        flag_comunicacao2 = 18;
        break;
        case 18:
        STRING_ENVIAR = 'V'+String(Vb_fase,0);// Fase do da tensão do barramento infinito
        flag_comunicacao2 = 19;
        break;
        case 19:
        STRING_ENVIAR = 'W'+String(F_bus,0);// Frequência do barramento infinito
        flag_comunicacao2 = 20;
        break;
        case 20:
        STRING_ENVIAR = 'G'+String(R_Velocidade*1000,0);// Referência de velocidade
        flag_comunicacao2 = 1;
        break;
        case 2:
        STRING_ENVIAR = 'B'+String(Ie);// Corrente de campo do motor de corrente contínua
        flag_comunicacao2 = 3;
        break;
      }
      break;
  default:
  flag_comunicacao = 1;
  }
  }

  
// Regulador de tensão--------------------
   if (ESTADO[0] == 'T')
  {
 switch (flag_comunicacao)
  { 
  case 1:
      STRING_ENVIAR = 'J' + String(V_ter,0);
      flag_comunicacao = 2;
      break;
  case 2:
      STRING_ENVIAR = 'K' + String(u_rat*1000,0);
      flag_comunicacao = 3;
      break;
  case 3:
      STRING_ENVIAR = 'N' + String(P_e*1000,0);
      flag_comunicacao = 4;
      break;
  case 4:
      STRING_ENVIAR = 'T' + String(I_arsg,0);
      flag_comunicacao = 5;
      break;
  case 5: 
      flag_comunicacao = 1;
      switch (flag_comunicacao2)
      { 
        case 1:
        STRING_ENVIAR = 'A'+String(Ia);
        flag_comunicacao2 = 2;
        break;
        case 3:
        STRING_ENVIAR = 'Z'+ESTADO;
        flag_comunicacao2 = 4;
        break;
        case 4:
        STRING_ENVIAR = 'C'+String(Vc1);
        flag_comunicacao2 = 5;
        break;
        case 5:
        STRING_ENVIAR = 'D'+String(Vc2);
        flag_comunicacao2 = 6;
        break;
        case 6:
        STRING_ENVIAR = 'E'+String(dut*1000,0);
        flag_comunicacao2 = 7;
        break;
        case 7:
        STRING_ENVIAR = 'F'+String(dut2*1000,0);
        flag_comunicacao2 = 8;
        break;
        case 8:
        STRING_ENVIAR = 'G'+String(R_Velocidade*1000,0);
        flag_comunicacao2 = 9;
        break;
        case 9:
        STRING_ENVIAR = 'H'+String(xg[0]*1000,0);
        flag_comunicacao2 = 10;
        break;
        case 10:
        STRING_ENVIAR = 'L'+String (V_sg,0);
        flag_comunicacao2 = 11;
        break;
        case 11:
        STRING_ENVIAR = 'M'+String(Vt_ref,0);
        flag_comunicacao2 = 12;
        break;
        case 12:
        STRING_ENVIAR = 'I'+String(u*1000,0);
        flag_comunicacao2 = 13;
        break;
        case 13:
        STRING_ENVIAR = 'O'+String(V_bus);
        flag_comunicacao2 = 14;
        break;
        case 14:
        STRING_ENVIAR = 'P'+String(dP_e*1000,0);
        flag_comunicacao2 = 15;
        break;
        case 15:
        STRING_ENVIAR = 'R'+String(PSS_c*1000,0);
        flag_comunicacao2 = 16;
        break;
        case 16:
        STRING_ENVIAR = 'S'+String(Pm,0);
        flag_comunicacao2 = 17;
        break;
        case 17:
        STRING_ENVIAR = 'U'+String (I_casg);
        flag_comunicacao2 = 18;
        break;
        case 18:
        STRING_ENVIAR = 'V'+String(Vb_fase,0);
        flag_comunicacao2 = 20;
        break;
        case 20:
        STRING_ENVIAR = 'X'+String(Velocidade_PU*1000,0);
        flag_comunicacao2 = 21;
        break;
        case 21:
        STRING_ENVIAR = 'a'+String(n_cargas);
        flag_comunicacao2 = 1;
        break;
        case 2:
        STRING_ENVIAR = 'B'+String(Ie);
        flag_comunicacao2 = 3;
        break;
      }
      break;
  default:
  flag_comunicacao = 1;
  }
  }

 // Estabilzador de ssitema elétrico de potencia--------------------
 if (ESTADO == "D1")
 {
   STRING_ENVIAR = 'Z'+ESTADO;
 }
 
  if (ESTADO[0] == 'S')
  {
 switch (flag_comunicacao)
  { 
  case 1:
      STRING_ENVIAR = 'J' + String(V_ter,0);
      flag_comunicacao = 2;
      break;
  case 2:
      STRING_ENVIAR = 'P' + String(dP_e*1000,0);
      flag_comunicacao = 3;
      break;
  case 3:
      STRING_ENVIAR = 'R' + String(PSS_c*1000,0);
      flag_comunicacao = 4;
      break;
  case 4:
      STRING_ENVIAR = 'X' + String (Velocidade_PU*1000,0);
      flag_comunicacao = 5;
      break;
  case 5: 
      flag_comunicacao = 1;
      switch (flag_comunicacao2)
      { 
        case 1:
        STRING_ENVIAR = 'A'+String(Ia);
        flag_comunicacao2 = 2;
        break;
        case 3:
        STRING_ENVIAR = 'Z'+ESTADO;
        flag_comunicacao2 = 4;
        break;
        case 4:
        STRING_ENVIAR = 'C'+String(Vc1);
        flag_comunicacao2 = 5;
        break;
        case 5:
        STRING_ENVIAR = 'D'+String(Vc2);
        flag_comunicacao2 = 6;
        break;
        case 6:
        STRING_ENVIAR = 'E'+String(dut*1000,0);
        flag_comunicacao2 = 7;
        break;
        case 7:
        STRING_ENVIAR = 'F'+String(dut2*1000,0);
        flag_comunicacao2 = 8;
        break;
        case 8:
        STRING_ENVIAR = 'G'+String(R_Velocidade*1000,0);
        flag_comunicacao2 = 9;
        break;
        case 9:
        STRING_ENVIAR = 'H'+String(xg[0]*1000,0);
        flag_comunicacao2 = 10;
        break;
        case 10:
        STRING_ENVIAR = 'L'+String(V_sg);
        flag_comunicacao2 = 11;
        break;
        case 11:
        STRING_ENVIAR = 'M'+String(Vt_ref,0);
        flag_comunicacao2 = 12;
        break;
        case 12:
        STRING_ENVIAR = 'I'+String(u*1000,0);
        flag_comunicacao2 = 13;
        break;
        case 13:
        STRING_ENVIAR = 'O'+String(V_bus,0);
        flag_comunicacao2 = 14;
        break;
        case 14:
        STRING_ENVIAR = 'N'+String(P_e*1000,0);
        flag_comunicacao2 = 15;
        break;
        case 15:
        STRING_ENVIAR = 'K'+String(u_rat*1000,0);
        flag_comunicacao2 = 16;
        break;
        case 16:
        STRING_ENVIAR = 'S'+String(Pm,0);
        flag_comunicacao2 = 17;
        break;
        case 17:
        STRING_ENVIAR = 'U'+String(I_casg);
        flag_comunicacao2 = 18;
        break;
        case 18:
        STRING_ENVIAR = 'V'+String(Vb_fase,0);
        flag_comunicacao2 = 20;
        break;
        case 20:
        STRING_ENVIAR = 'T'+String(I_arsg);
        flag_comunicacao2 = 21;
        break;
        case 21:
        STRING_ENVIAR = 'a'+String(n_cargas);
        flag_comunicacao2 = 22;
        break;
        case 22:
        STRING_ENVIAR = 'b'+String(sin_est);
        flag_comunicacao2 = 1;
        break;
        case 2:
        STRING_ENVIAR = 'B'+String(Ie);
        flag_comunicacao2 = 3;
        break;
      }
      break;
  default:
  flag_comunicacao = 1;
  }
  }
 
  SerialUSB.println(STRING_ENVIAR);
 }


 // cálculo do valor RMS das ondas

 void Valor_RMS()
 {
    /*
    for(count_rms_flag = 0; count_rms_flag <= per_rms - 2;++count_rms_flag)
    {
      V_rms_flagab[per_rms - 1- count_rms_flag]    = V_rms_flagab[per_rms - 2- count_rms_flag];
      V_rms_flagcb[per_rms - 1- count_rms_flag]    = V_rms_flagcb[per_rms - 2- count_rms_flag];
      I_rms_flaga [per_rms - 1- count_rms_flag]    = I_rms_flaga [per_rms - 2- count_rms_flag];
      I_rms_flagc [per_rms - 1- count_rms_flag]    = I_rms_flagc [per_rms - 2- count_rms_flag];
      P1          [per_rms - 1- count_rms_flag]    = P1          [per_rms - 2- count_rms_flag];
      P2          [per_rms - 1- count_rms_flag]    = P2          [per_rms - 2- count_rms_flag];
    }
    V_rms_flagab[0]    = Vab_ins;
    V_rms_flagcb[0]    = Vcb_ins;
    I_rms_flaga [0]    = Ia_ins;
    I_rms_flagc [0]    = Ic_ins;
    P1[0]              = Vab_ins*Ia_ins;
    P2[0]              = Vcb_ins*Ic_ins;
    Vab_ins = 0;
    Vcb_ins = 0;
    Ia_ins  = 0;
    Ic_ins  = 0;
    P1_flag = 0;
    P2_flag = 0;
    */
    /*
    for(count_rms_flag = 0; count_rms_flag <= per_rms - 1; ++count_rms_flag)
    {
      Vab_ins =Vab_ins + V_rms_flagab[count_rms_flag]*V_rms_flagab[count_rms_flag]/per_rms;
      Vcb_ins =Vcb_ins + V_rms_flagcb[count_rms_flag]*V_rms_flagcb[count_rms_flag]/per_rms;
      Ia_ins  =Ia_ins  + I_rms_flaga [count_rms_flag]*I_rms_flaga [count_rms_flag]/per_rms;
      Ic_ins  =Ic_ins  + I_rms_flagc [count_rms_flag]*I_rms_flagc [count_rms_flag]/per_rms;
      P1_flag =P1_flag + P1[count_rms_flag]*P1[count_rms_flag]/per_rms;
      P2_flag =P2_flag + P2[count_rms_flag]*P2[count_rms_flag]/per_rms;
    }*/
    /*Vab_rms = sqrt(Vab_ins);
    Vcb_rms = sqrt(Vcb_ins);
    Ia_rms  = sqrt(Ia_ins);
    Ic_rms  = sqrt(Ic_ins);
    P_ativa = sqrt(P1_flag) + sqrt(P1_flag);
  */
 }
void fase_inf_bus()
{
  V1_flag[0] = 0;// colocar valor instatâneo
  V2_flag[0] = 0;// colocar valor instatâneo
  if (V1_flag[0] < 0 && V1_flag[1] > 0)
  {
    if (V2_flag[0] > 0)
    {
      sin_fase = 1;
    }
    else
    {
      sin_fase = 0;
    }
  }
  if (V2_flag[0] > V2_flag[1])
  {
  }
}
 // Acionamento de cargas

 void acionar_cargas()
 {
  if (ESTADO == "T4"|| ESTADO[0] == 'S')
  {
   if (STRING == "CG1")
   {
    n_cargas = n_cargas +1;
    STRING = "";
   }
   else if (STRING == "CG0")
   {
    n_cargas = n_cargas - 1;
    STRING = "";
   }
   if (n_cargas > 10)
   {
    n_cargas = 10;
   }
   if (n_cargas < 0)
   {
    n_cargas = 0;
   } 
   switch (n_cargas)
   {
    case 0:
    digitalWrite(contatora[7],  LOW);
    digitalWrite(contatora[8],  LOW);
    digitalWrite(contatora[9],  LOW);
    digitalWrite(contatora[10], LOW);
    digitalWrite(contatora[11], LOW);
    digitalWrite(contatora[12], LOW);
    digitalWrite(contatora[13], LOW);
    digitalWrite(contatora[14], LOW);
    break;
    case 1:
    digitalWrite(contatora[7], HIGH);
    digitalWrite(contatora[8],  LOW);
    digitalWrite(contatora[9],  LOW);
    digitalWrite(contatora[10], LOW);
    digitalWrite(contatora[11], LOW);
    digitalWrite(contatora[12], LOW);
    digitalWrite(contatora[13], LOW);
    digitalWrite(contatora[14], LOW);
    break;
    case 2:
    digitalWrite(contatora[7], HIGH);
    digitalWrite(contatora[8], HIGH);
    digitalWrite(contatora[9],  LOW);
    digitalWrite(contatora[10], LOW);
    digitalWrite(contatora[11], LOW);
    digitalWrite(contatora[12], LOW);
    digitalWrite(contatora[13], LOW);
    digitalWrite(contatora[14], LOW);
    break;
    case 3:
    digitalWrite(contatora[7], HIGH);
    digitalWrite(contatora[8], HIGH);
    digitalWrite(contatora[9], HIGH);
    digitalWrite(contatora[10], LOW);
    digitalWrite(contatora[11], LOW);
    digitalWrite(contatora[12], LOW);
    digitalWrite(contatora[13], LOW);
    digitalWrite(contatora[14], LOW);
    break;
    case 4:
    digitalWrite(contatora[7], HIGH);
    digitalWrite(contatora[8], HIGH);
    digitalWrite(contatora[9], HIGH);
    digitalWrite(contatora[10],HIGH);
    digitalWrite(contatora[11], LOW);
    digitalWrite(contatora[12], LOW);
    digitalWrite(contatora[13], LOW);
    digitalWrite(contatora[14], LOW);
    break;
    case 5:
    digitalWrite(contatora[7], HIGH);
    digitalWrite(contatora[8], HIGH);
    digitalWrite(contatora[9], HIGH);
    digitalWrite(contatora[10],HIGH);
    digitalWrite(contatora[11],HIGH);
    digitalWrite(contatora[12], LOW);
    digitalWrite(contatora[13], LOW);
    digitalWrite(contatora[14], LOW);
    break;
    case 6:
    digitalWrite(contatora[7], HIGH);
    digitalWrite(contatora[8], HIGH);
    digitalWrite(contatora[9], HIGH);
    digitalWrite(contatora[10],HIGH);
    digitalWrite(contatora[11],HIGH);
    digitalWrite(contatora[12],HIGH);
    digitalWrite(contatora[13], LOW);
    digitalWrite(contatora[14], LOW);
    break;
    case 7:
    digitalWrite(contatora[7], HIGH);
    digitalWrite(contatora[8], HIGH);
    digitalWrite(contatora[9], HIGH);
    digitalWrite(contatora[10],HIGH);
    digitalWrite(contatora[11],HIGH);
    digitalWrite(contatora[12],HIGH);
    digitalWrite(contatora[13],HIGH);
    digitalWrite(contatora[14], LOW);
    break;
    case 8:
    digitalWrite(contatora[7], HIGH);
    digitalWrite(contatora[8], HIGH);
    digitalWrite(contatora[9], HIGH);
    digitalWrite(contatora[10],HIGH);
    digitalWrite(contatora[11],HIGH);
    digitalWrite(contatora[12],HIGH);
    digitalWrite(contatora[13],HIGH);
    digitalWrite(contatora[14],HIGH);
    break;
    default:
    break;
   }
   }
   else
   {
    digitalWrite(contatora[7],  LOW);
    digitalWrite(contatora[8],  LOW);
    digitalWrite(contatora[9],  LOW);
    digitalWrite(contatora[10], LOW);
    digitalWrite(contatora[11], LOW);
    digitalWrite(contatora[12], LOW);
    digitalWrite(contatora[13], LOW);
    digitalWrite(contatora[14], LOW);
   }
 }

 //----------------------------Funções do estado--------------------------------

 void funcao_D1()
 {/*
   digitalWrite(contatora[0 ], LOW);
   digitalWrite(contatora[1 ], LOW);
   digitalWrite(contatora[2 ], LOW);
   digitalWrite(contatora[3 ], LOW);
   digitalWrite(contatora[4 ], LOW);
   digitalWrite(contatora[5 ], LOW);
   digitalWrite(contatora[6 ], LOW);
   digitalWrite(contatora[7 ], LOW);
   digitalWrite(contatora[8 ], LOW);
   digitalWrite(contatora[9 ], LOW);
   digitalWrite(contatora[10], LOW);
   digitalWrite(contatora[11], LOW);
   digitalWrite(contatora[12], LOW);
   digitalWrite(contatora[13], LOW);
   digitalWrite(contatora[14], LOW);
*/
// Reset das variáveis do sistema de emulação
/*

  sensorPinVc1   = A3;//Entrada do banco de capacitores do campo
  sensorPinVc2   = A4;//Entrada do banco de capaciotres da Armadura
  sensorPina12   = A0;//Entrdada da corrente de armadura
  sensorpinA5    = A2;//Entrada da corrente de campo
  sensorpinVab   = A5;//Entrada da tensão Vab
  sensorpinVbc   = A6;//Entrada da tensão Vbc
  sensorpinIa    = A7;//Entrada da corrente Ia
  sensorpinIb    = A8;//Entrada da corrente Ib
  velan          = A9;//Entrada da velocidade
  exgs           = A10;//Entrada da velocidade

//Valores das tensões nos capacitores

Vcam  = 0;//Variável que armazena a tensão de campo
Vcarm = 0;//Variável que armazena a tensão de armadura
Vminarm = 300;//Tensão mínima do banco de capacitores da armadura, para finalizar o carregamento dos capacitores
Vmincam = 160;//Tensão mínima do banco de capacitores do campo, para finalizar o carregamento dos capacitores
  
// Definindo as variaveis que refebecerão os dados da porta analógica

Ia     = 0;// Corrente de armadura
Ie     = 0;// Corrente de campo
Vc1    = 0;// Capacitor de campo
Vc2    = 0;// Capacitor de Armadura
// Váriaveis em PU:
Velocidade_PU = 0;//Velocidade do motor em PU
Ia_PU[0]         = 0; //Corrente de armadura em PU
Ia_PU[1]         = 0; //Corrente de armadura em PU
Ia_PUNF[0]       = 0; // Corrente de armadura não filtrada
Ia_PUNF[1]       = 0; // Corrente de armadura não filtrada
Ic_mm[0]         = 0; // Corrente de excitação de campo do motor de corrente continua
Ic_mm[1]         = 0; // Corrente de excitação de campo do motor de corrente continua
// Variaveis usadas para o dutty
tensaoBase = 206.62; // Tensão de base do motor
tensaoDC   = 320;//Tensão do link DC
dut = 0;// Dutty do conversor DC-DC
dut2 = 0; // Dutty do con
periodo = 50000;//Periodo do PWM
T_altoi = 0;//Tempo em alto inicial
T_alto  = 0;//Tempo em alto do buck
i = 1;//indice
dutmax = 0.6; // Valor máximo do dut.
dutmax2 = 0.05; // Valor máximo do dut.

Ve[0] = 0;//Variável que armazena a velocidade em RPM
Ve[1] = 0;//Variável que armazena a velocidade em RPM
Ve[2] = 0;//Variável que armazena a velocidade em RPM
Estado_v = "W";//Estring que armazena o estados da máquina de estado utilizada para medir a velocidade
contando;//Variável que conta o numeros de interrupções durante um giro completo da máquina
velanVe;//Váriável que armazena o valor da porta analógica
Vef[0] = 0;//Váriavel que armazena a velocidade filtrada
Vef[1] = 0;//Váriavel que armazena a velocidade filtrada
Vef[2] = 0;//Váriavel que armazena a velocidade filtrada

STRING_SERIAL = "";//Amrazena os dados da porta serial
STRING="OI";//Armazena os dados da porta serial para serem usados ao longo do algorítimo
STRING_ENVIAR="";//È a string enviada para a porta serial
velocidade;// String Utilizada para enviar a velocidade para a IHM
Vcampo;//String que armazena a tensão de campo a ser enviada para porta serial
Varmadura;//String que armazena a tensão de Armadura a ser enviada para porta serial
Icampo;//String que armazena a Corrente de campo a ser enviada para porta serial
Iarmadura;//String que armazena a corrente de Armadura a ser enviada para porta serial
dut_serial;//String que armazena o dutty a ser enviada para porta serial
dut2_serial;//String que armazena o dutty a ser enviada para porta serial
STRING_CVe; //String que armazena a saida do controlador de velocidade;
STRING_Pm; // String que armazena a potênica da turbina hidráulica;
STRING_C;//Saida do controlador de potência;
STRING_xg;//Abetura do distribuidor;
STRING_R_Velocidade;//Referênica da velocidade
Referencia = "";
flag_comunicacao = 1;//Variável utilizada para alternar a informação a ser enviada para porta serial
flag_comunicacao2 = 1;//Variável utilizada para alternar a informação a ser enviada para porta serial

//Emulação da turbina:
deltaH[0] = 0;  // Variação da altura da queda da água com relação a queda inicial;
deltaH[1] = 0;  // Variação da altura da queda da água com relação a queda inicial;
Pm        =     0;  // Potência mecânica da Turbina;
H         =     1;  // Altura da queda água;
Unl       = 0.068;  // Velocidade da água ao passar pela turbina sem carga;
H0        =     1;  // Altura inicial da queda da água;
gmin      =  0.06;  // Abertura mínima do distribuidor;
gmax      =  0.96;  // Abertura máxima do distribuidor;
At        = 1/(gmax-gmin);  // Ganho da turbina;
g         =   0.0;  // Posição do servo posicionado;
Tw        =  1.41;  // Constante de tempo da turbina;
G         =  0;//Abertura do distribuidor
U[0]         =  0;
U[1]         =  0;

//Servoposicionador
xg[0] = 0;
xg[1] = 0;
xp[0] = 0;
xp[1] = 0;
un[0] = 0;//sinal de referência
un[1] = 0;//sinal de referência
u     = 0;// Sinal do controlador
//Controlador de potência

C[0] = 0;
C[1] = 0;
E[0] = 0;
E[1] = 0;
Pm_pu = 0;

//Regulador de velocidade:
R_Velocidade = 1;
R4[0]     = 0;
R4[1]     = 0;
CF_P[0]   = 0;
CF_P[1]   = 0;
CVe[0]    = 0;
CVe[1]    = 0;
CVe[2]    = 0;
CVe[3]    = 0;
EVe[0]    = 0;
EVe[1]    = 0;
EVe[2]    = 0;
EVe[3]    = 0;
C_S_S = 0;
R = 0;
Z = 0;
// Regulador de tensão

V_ter    =  220;// tensão terminal
u_rat    =  50; // sinal de controle do rat
V_sg     =  220;// tensão do capacitor de campo do sg
V_sgmin  =  160;// tensão minima do capacitor de campo para finalizar o carregamento
Vt_ref   =  200;// tensão de referência
P_e      =  0;// potência elétrica
I_arsg   =  10;// corrente de armadura
I_casg   =  10;// corrente de campo
Vb_fase  = 10; // fase da tensão do barramento
V_bus    = 220; // tensão do barramento
F_bus    = 60; // frequência do barramento
dut3     = 0;
// Sistema interligado;

dP_e = 0; // Variação da potência elétrica
PSS_c = 0; // sinal de controle do PSS

// Valores RMS------------------------------------------------
count_rms_flag = 1;
Vab_ins = 0; // Valor instantâneo da tensão de linha Vab
Vcb_ins = 0; // Valor instantâneo da tensão de linha Vbc.
Vab_rms = 0; // Valor eficaz da tensão vab 
Vcb_rms = 0; // Valor eficaz da tensão vcb
Ia_rms  = 0; // Valor eficaz da corrente Ia
Ic_rms  = 0; // Valor eficaz da corrente Ic
Ia_ins  = 0; // Valor instantâneo da tensão de linha Ia
Ic_ins  = 0; // Valor instantâneo da tensão de linha Ib
I_rms   = 0; // Corrente de armadura RMS
V_rms   = 0; // Tensão do terminal de armadura RMS
// volatile float V_rms_flag      = 0;
// volatile float I_rms_flag      = 0;
P_ativa      = 0;
P1_flag      = 0;
P2_flag      = 0;
per_rms      = 167;
sin_fase        = 0; // zero atrasado e 1 adiantado
//---Divisor de frequência digital
count = 1;
divfreq = 1;
sin_est = 2;
pin_len       = 4;
pin_rap       = 5;
pin_sin       = 6;
// Acionar Cargas

n_cargas = 0;*/
 }

 void funcao_E1()
 {
  digitalWrite(contatora[0], HIGH);
 }

 void funcao_E2()
 {
      digitalWrite(contatora[0], LOW);
      digitalWrite(contatora[1], HIGH);
 }

 void funcao_E3()
 {
  
      digitalWrite(contatora[2], HIGH);
 }

 void funcao_E4()
 {
  Controle_PWM();
 }

 void funcao_V1()
 {
    u = up1;
    emulador_tub();
 }

 void funcao_V2()
 {
    u = up2;
    emulador_tub();
 }

 void funcao_V3()
 {
    RV();
    emulador_tub();
 }

 void funcao_T1()//Energizar buck
 {
  RV();
  emulador_tub();
  acionar_cargas();
 }

  void funcao_T2()//ENEGIZAR O TRANSFORMADOR
 {
  RV();
  emulador_tub();
  acionar_cargas();
  digitalWrite(contatora[4], HIGH);
 }

 void funcao_T3()// Energizar linha
 {
  RV();
  emulador_tub();
  acionar_cargas();
  digitalWrite(contatora[3], HIGH);
 }

 void funcao_T4()// Iniciar RAT
 {
  RV();
  emulador_tub();
  sincron_rede();
  acionar_cargas();
  RAT();
  }

 void funcao_S1()
 {
  RV();
  emulador_tub();
  sincron_rede();
  acionar_cargas();
 }
 void funcao_S2()// Conectar a rede
 {
  RV();
  emulador_tub();
  sincron_rede();
  acionar_cargas();
  digitalWrite(contatora[5], HIGH); 
  if (STRING == "DS")
   {
    ESTADO = "S1";
     digitalWrite(contatora[5], LOW); 
   }
 }
 void funcao_S3()
 {
  RV();
  emulador_tub();
  acionar_cargas();
  sincron_rede();
  RAT();
  PSS_controller();
  if (STRING == "DS")
  {
    ESTADO = "S1";
     digitalWrite(contatora[5], LOW); 
  }
 }

 void sincron_rede()
 { 
  int val0 = digitalRead(pin_len);
  int val1 = digitalRead(pin_sin);
  int val2 = digitalRead(pin_rap);
  if (val0 == 0)
  {
    sin_est = 0;
  }
  if (val2== 0)
  {
    sin_est = 2;
  }
  if (val1 == 0)
  {
    sin_est = 1;
  }
 }
