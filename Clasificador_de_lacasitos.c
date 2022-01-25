#include <msp430.h> 

/*-----Órdenes y direcciones para la comunicación con el TCS34725 (Sensor de color)----*/
#define dirSensor 0x29 //Dirección del sensor como Slave

#define regCom 0b10000000 //Accede al registro de comandos del TCS34725

#define borrarInt 0b01100110 //Escrito en el registro de comandos borra todas las interrupciones pendientes

#define dirAct 0x00 //Dirección del registro de activación. En él escribiremos los siguientes valores:
#define actOsc 0b00000001 //(Activamos el oscilador)
#define actInt 0b00010000 //(Activamos las interrupciones)
#define actADC 0b00000010 //(Activamos el convertidor analógico digital)

#define dirPers 0x0C //Dirección del "Persistence Register" (Controla las interrupciones)
#define confInter 0b00000000 //Se generarán interrupciones cada vez que finalice la integración del convertidor

#define dirTiming 0x01 //Registro que controla el tiempo de integración del convertidor
#define timing 0xC0 //154 ms (Tiempo de integración escogido)

#define dirCtrl 0x0F //Dirección del registro de control (Me permite modificar la ganancia del convertidor)
#define g1 0b00000000 //Ganancia 1 (Mismo valor)
#define g4 0b00000001 //Ganancia 4
#define g16 0b00000010 ////Ganancia 16
#define g60 0b00000011 // Ganancia 60

#define dirColor 0x14 //Dirección del registro del primer bit (Luminosidad)
//Nota: El resto de registros que almacenan el resultado de la conversión RGBC están en orden ascendente
//El orden de los registros es: C,R,G,B (Luminosidad, Rojo, Verde, Azul)
//Para cada color hay dos registros, aportándonos una resolución de 2 bytes.

/*----Ángulos Servo 180----*/
#define deg_0 999 //0.5 ms -->      Rosa
#define deg_45 1999 //1 ms -->      Naranja
#define deg_70 2499 //1.25 ms -->   Defectos
#define deg_90 2999 //1.5 ms -->    Celeste
#define deg_135 3999 //2 ms -->     Rojo
#define deg_180 4999 //2.5 ms -->   Verde

/*----Movimiento servo 360----*/
#define giro1 3299 //1.75 ms -->    Horario
#define giro2 2499 //1.24 ms -->    Antihorario
#define parar 2999 //1.5 ms -->     Parado


/*----Función que escribe en la flash un vector----*/
void escribeFlash(char dato[], unsigned int direc){
    char *  Puntero = (char *) direc; //Dirección inicial
    char i=0; //Índice

    /*Comprobamos que la dirección se encuentre en el segmento D*/
    if(direc>=0x1000 && direc<0x10C0){
        /*Borramos todo el segmento*/
        FCTL1 = FWKEY + ERASE;  //Activa Erase
        FCTL3 = FWKEY;          //Borra Lock
        *Puntero = 0;           //Borramos el segmento

        /*Escribimos el vector en memoria*/
        FCTL1 = FWKEY + WRT;    //Activamos WRT (y desactivamos ERASE)
        for (i=0;i<6;i++)
            *(Puntero+i)= dato[i]; // Escribe el dato en 0x1000

        /*Restauramos valores*/
        FCTL1 = FWKEY;          //Borramos el bit WRT
        FCTL3 = FWKEY + LOCK;   //Resturamos el bit LOCK
    }
}

/*----Función que configura el reloj----*/
void conf_reloj(char VEL){
    BCSCTL2 = SELM_0 | DIVM_0 | DIVS_0;
    switch(VEL){
    case 1:
        if (CALBC1_1MHZ != 0xFF) {
            DCOCTL = 0x00;
            BCSCTL1 = CALBC1_1MHZ;      /* Set DCO to 1MHz */
            DCOCTL = CALDCO_1MHZ;
        }
        break;
    case 8:

        if (CALBC1_8MHZ != 0xFF) {
            __delay_cycles(100000);
            DCOCTL = 0x00;
            BCSCTL1 = CALBC1_8MHZ;      /* Set DCO to 8MHz */
            DCOCTL = CALDCO_8MHZ;
        }
        break;
    case 12:
        if (CALBC1_12MHZ != 0xFF) {
            __delay_cycles(100000);
            DCOCTL = 0x00;
            BCSCTL1 = CALBC1_12MHZ;     /* Set DCO to 12MHz */
            DCOCTL = CALDCO_12MHZ;
        }
        break;
    case 16:
        if (CALBC1_16MHZ != 0xFF) {
            __delay_cycles(100000);
            DCOCTL = 0x00;
            BCSCTL1 = CALBC1_16MHZ;     /* Set DCO to 16MHz */
            DCOCTL = CALDCO_16MHZ;
        }
        break;
    default:
        if (CALBC1_1MHZ != 0xFF) {
            DCOCTL = 0x00;
            BCSCTL1 = CALBC1_1MHZ;      /* Set DCO to 1MHz */
            DCOCTL = CALDCO_1MHZ;
        }
        break;

    }
    BCSCTL1 |= XT2OFF | DIVA_0;
    BCSCTL3 = XT2S_0 | LFXT1S_2 | XCAP_1;

}
/*----Variables Globales----*/
/*Control del bus I2C*/
char leer = 0; //Almacena el valor leido por el bus I2C
char enviar[2] = {0,0}; //Almacena el valor que enviaremos al sensor (Enviaremos uno o dos bytes)
char tam,ind; // tam -> Número de bytes a transmitir/recibir -- ind -> índice de los vectores para leer/escribir datos
char I2C_RW=0; //Configura ciclo de lectura o escritura

/*Lecturas del sensor de color*/
char bytesCRGB[8]={0,0,0,0,0,0,0,0}; //La lectura se realiza en bytes, cada color ocupa dos bytes
unsigned int CRGB[4] = {0,0,0,0}; //Valores de claridad, rojo, verde y azul respectivamente
unsigned int ratioRGB[3]={0,0,0}; //Porcentaje de cada color sobre la claridad
char contPOBRGE[6] = {0,0,0,0,0,0}; //Contador de lacasitos clasificados (Pink - Yellow - Orange - Red - Green - Errors)

/*Mecánica del claisficador*/
char rebotes = 0; //Indica a la rutina de interrupción del sensor IR si ha entrado como causa de un rebote
char alineado=0; //Se pone a 1 cuando la rueda alinea el lacasito y el sensor
char start = 0; //Control del proceso (Se activa o desactiva por BT)
char atasco = 0; //Se activa cuando se produce un atasco (3 Seg el programa parado)
char errorLectura=0; //Color que no coincide con ninguno de los 5
char errores =0; //Veces que ocurre un defecto de lectura (Antes de activar errorLectura, leemos 5 veces)
char invLectura = 0; //Invalidamos la lectura (Durante un atasco evitamos que se lea el mismo lacasito)

/*Temporizadores*/
/*TA1:*/
char tempOn = 0; //Inicia el temporizador de módulo variable
char mod=0; //Modulo del temporizador (x20ms)
char t=0; //Contador del temporizador

char tempAtasco = 0; //Inicia el temporizador comprobador de atascos
char tAtasco = 0; //Contador del temporizador

/*TA0:*/
char tleds=0; //PWM Software del led RGB
char dutyR=0,dutyG=0,dutyB=0; //Duty Cycle de los leds (0-19)

/*Bluetooth*/
char leeBT = 0; //Valor leído por BT

/*Flash*/
char * leeFlash= (char *) 0x1000; //Dirección en la que almacenamos el vector que cuenta los lacasitos

/*----Programa principal----*/
int main(void)
{
    char i=0; //Índice

    /*------------CONFIGURACIÓN------------*/
    WDTCTL = WDTPW | WDTHOLD;   //Paramos el watchDog
    conf_reloj(8); //SMCLK a 8MHz

    /*---------Pin de interrucpciones del sensor RGB-------*/
    P1DIR &= ~BIT3;//Entrada
    P1REN|=BIT3; //Pull-up
    P1OUT|=BIT3;
    P1IE|=BIT3; //Habilito interrupción
    P1IES|=BIT3; //Flanco de bajada
    P1IFG=0; //Borramos todos los flags

    /*---------Pin de interrupciones del sensor IR--------*/
    P1DIR &= ~BIT4;//Entrada
    P1IE|=BIT4; //Habilito interrupción
    P1IES&=~BIT4; //Flanco de subida
    P1IFG=0; //Borramos todos los flags
    //NOTA: 0 = Pared <---> 1 == Hueco

    /*------Configuración del Timer y de los servos------*/
    /*Configuración del timer TA1*/
    TA1CTL=TASSEL_2|ID_2| MC_1; //SMCLK, DIV=4 (2MHz), UP
    TA1CCR0=39999; //Periodo 20 ms
    TA1CCTL0=CCIE; //Habilitamos interrupciones

    /*PWM -- Servo 180º (Dirección del tubo)*/
    P2DIR |= BIT2; //Salida
    P2OUT &=~  BIT2; //Apagados (Si activo el modo I/O estará apagados)

    TA1CCR1=deg_70; //Comenzamos apuntando a la zona de defectos
    TA1CCTL1=OUTMOD_7; //Activo a nivel alto

    P2SEL |= BIT2; //PWM
    P2SEL2 &=~ BIT2; //PWM

    /*Configuración del pwm (servo 360º)*/
    P2DIR |= BIT5; //Salida
    P2OUT &=~  BIT5; //Apagados (Si activo el modo I/O estará apagados)

    TA1CCR2=parar;
    TA1CCTL2=OUTMOD_7; //Activo a nivel alto

    P2SEL |= BIT5; //PWM
    P2SEL2 &=~ BIT5; //PWM

    /*------Configuración de los leds RGB------*/
    /*Configuramos el Timer 0 para generar interrupciones cada 0.01ms*/
    TA0CTL=TASSEL_1|ID_0| MC_1; //ACLK, DIV=1 , UP
    TA0CCR0=19;       //Periodo 1.6 ms
    TA0CCTL0=CCIE; //Habilitamos interrpciones

    /*Configuramos los pines de los leds como salidas (inicialmente apagadas)*/
    P2DIR |= BIT3+BIT4+BIT0; //R G B
    P2OUT &=~ BIT3+BIT4+BIT0;
    //NOTA:El PWM lo generaremos por software en la rutina de interrupción del timer 0

    /*------Configuración del Módulo Bluetooth------*/
    /*Configuración de la USCI_A*/
    UCA0CTL1 |= UCSWRST; //Mantenemos el reset
    UCA0CTL1 = UCSSEL_2 | UCSWRST; //SMCLK
    UCA0MCTL = UCBRF_0 | UCBRS_6;
    UCA0BR0 = 65; //9600 bps
    UCA0BR1 = 3; //9600 bps
    UCA0CTL1 &= ~UCSWRST; //Quitamos el reset

    IFG2 &= ~(UCA0RXIFG); //Borramos el flag de RX
    IE2 |= UCA0RXIE; //Activamos interrupciones de lectura
    //NOTA: La escritura la realizamos por pulling

    P1SEL |= BIT1 | BIT2; //P1.1 RX -- P1.2 TX
    P1SEL2 |= BIT1 | BIT2;

    /*------Configuración del I2C------*/
    /*Pines de SCL y SDA  P1.6(SCL) P1.7(SDA)*/
    P1SEL |= BIT6|BIT7;
    P1SEL2 |= BIT6|BIT7;

    /*Configuración de la USCI_B*/
    UCB0CTL1 |= UCSWRST; //Mantenemos un reset en la USCI (Vamos a configurarlo)
    UCB0CTL0 = UCMST | UCMODE_3 | UCSYNC; //Configuración: Master | I2C | Modo Síncrono (I2C)
    UCB0CTL1 = UCSSEL_2 | UCSWRST; //Reloj : SMCLK | Mantenemos el reset

    UCB0I2CSA = dirSensor; //Dirección del Slave
    UCB0BR0 = 10; //Divisor de frecuencia (8Mhz/10 = 8kHz)
    UCB0CTL1 &= ~UCSWRST; // Hemos finalizado la configuración de la USCI, quitamos el reset

    IFG2 &= ~(UCB0TXIFG); //Borramos el flag de lectura
    IFG2 &= ~(UCB0RXIFG); //Borramos el flag de escritura
    IE2 |= UCB0TXIE+UCB0RXIE; //Habilitamos interrupciones

    /*-----Habilitamos interrupciones----*/
    __bis_SR_register(GIE);

    /*------------CONFIGURACIÓN DEL SENSOR A TRAVÉS DEL BUS I2C------------*/
    /*Activación*/
    I2C_RW = 0; //Escritura
    enviar[1]=regCom|dirAct; //Accedemos al registro de activación
    enviar[0]=actOsc|actADC; //Activamos el oscilador y el convertidor
    tam=2;
    ind=tam;
    UCB0CTL1 |= UCTR + UCTXSTT; // Modo escritura y start
    LPM0; //Duermo al micro
    while(I2C_RW!=3); //Me aseguro de que ha terminado

    /*Configuración de las interrupciones*/
    I2C_RW = 0; //Escritura
    enviar[1]=regCom|dirPers; //Accedemos al Persistence Register
    enviar[0]= confInter;//Se generarán interrupciones cada vez que finalice la integración del convertidor
    tam=2;
    ind=tam;
    UCB0CTL1 |= UCTR + UCTXSTT; // Modo escritura y start
    LPM0; //Duermo al micro
    while(I2C_RW!=3); //Me aseguro de que ha terminado

    /*Configuración del tiempo de integración*/
    I2C_RW = 0; //Escritura
    enviar[1]=regCom|dirTiming; //Accedemos al registro
    enviar[0]=timing; //Escogemos el tiempo de integración máximo 700ms (Probar varios)
    tam=2;
    ind=tam;
    UCB0CTL1 |= UCTR + UCTXSTT; // Modo escritura y start
    LPM0; //Duermo al micro
    while(I2C_RW!=3); //Me aseguro de que ha terminado

    /*Configuración de la ganancia del convertidor*/
    I2C_RW = 0; //Escritura
    enviar[1]=regCom|dirCtrl; //Accedemos al registro
    enviar[0]=g4; //Escogemos el tiempo de integración máximo 700ms (Probar varios)
    tam=2;
    ind=tam;
    UCB0CTL1 |= UCTR + UCTXSTT; // Modo escritura y start
    LPM0; //Duermo al micro
    while(I2C_RW!=3); //Me aseguro de que ha terminado

    /*Activamos las interrupciones*/
    I2C_RW = 0; //Escritura
    enviar[1]=regCom|dirAct; //Accedemos al registro de activación
    enviar[0]=actOsc|actADC|actInt; //Activamos las interrupciones
    tam=2;
    ind=tam;
    UCB0CTL1 |= UCTR + UCTXSTT; // Modo escritura y start
    LPM0; //Duermo al micro
    while(I2C_RW!=3); //Me aseguro de que ha terminado

    //NOTA: Hemos configurado el sensor para que genere interrupciones cuando termine la conversión de CAD



    /*------------COMIENZO DEL PROGRAMA------------*/
    /*Recuperamos la cuenta de lacasitos de la memoria FLASH*/
    for (i=0;i<6;i++)
        contPOBRGE[i]=*(leeFlash+i); //Leemos los valores almacenados

    /*Debemos distinguir entre si comienza en hueco o en pared para filtrar los rebotes del flanco de bajada*/
    if(P1IN&BIT4) //Hueco
        rebotes=1; //Necesario para filtrar rebotes en el flanco de subida
    else //Pared
        rebotes=0;

    TA1CCR2=giro1; //Ponemos en marcha el motor

    /*------Bucle infinito------*/
    while(1)
    {
        LPM0; //Dormimos al micro hasta que el ADC esté listo para darnos un valor

        /*Lectura del color*/
        if(alineado&&(!tempOn)&&start) //Solo leemos si está alineado y se ha dado la orden de start
        {
            /*Guardamos en memoria todos los bytes de la lectura*/
            for(i=0;i<8;i++)
            {
                I2C_RW = 1; //Lectura
                enviar[0]=regCom|(dirColor+i); //Accedemos al registro
                tam=1;
                ind=tam;
                UCB0CTL1 |= UCTR + UCTXSTT; // Modo escritura y start
                LPM0;
                while(I2C_RW!=3); //Me aseguro de que ha terminado
                bytesCRGB[i]=leer;
            }

            /*Generamos los valores de 2 bytes*/
            for(i=0;i<4;i++)
                CRGB[i] = bytesCRGB[2*i]+(bytesCRGB[2*i+1]<<8);

            /*Creamos el ratio de color sobre claridad (%)*/
            for(i=0;i<3;i++)
                ratioRGB[i] =(CRGB[i+1])/(CRGB[0]/100);

            /*Posicionamiento del servo y actualización de contadores*/
            if(CRGB[0]>900 && (!invLectura)) //Solo lo realizamos si no es un hueco (Luminosidad<900) y si no está resolviendo un atasco
            {
                /*ROSA*/
                if(ratioRGB[0]>22 && ratioRGB[0]<38 && ratioRGB[1]>24 && ratioRGB[1]<35 &&ratioRGB[2]>29 &&ratioRGB[2]<39)
                {
                    TA1CCR1=deg_0;
                    contPOBRGE[0]++;
                    dutyR=17;dutyG=0;dutyB=17; //Led Púrpura
                }
                /*NARANJA*/
                else if(ratioRGB[0]>38 && ratioRGB[0]<60 && ratioRGB[1]>24 && ratioRGB[1]<36 &&ratioRGB[2]>9 && ratioRGB[2]<17)
                {
                    TA1CCR1=deg_45;
                    contPOBRGE[1]++;
                    dutyR=17;dutyG=1;dutyB=0; //Led Naranja
                }
                /*CELESTE*/
                else if(ratioRGB[0]>14 && ratioRGB[0]<29 && ratioRGB[1]>34 && ratioRGB[1]<42 &&ratioRGB[2]>34 &&ratioRGB[2]<43)
                {
                    TA1CCR1=deg_90;
                    contPOBRGE[2]++;
                    dutyR=0;dutyG=5;dutyB=17; //Led Celeste
                }
                /*ROJO*/
                else if(ratioRGB[0]>40 && ratioRGB[0]<62 && ratioRGB[1]>19 && ratioRGB[1]<30 &&ratioRGB[2]>18 &&ratioRGB[2]<30)
                {
                    TA1CCR1=deg_135;
                    contPOBRGE[3]++;
                    dutyR=17;dutyG=0;dutyB=0; //Led Rojo
                }
                /*VERDE*/
                else if(ratioRGB[0]>24 && ratioRGB[0]<42 && ratioRGB[1]>38 && ratioRGB[1]<49 &&ratioRGB[2]>14 &&ratioRGB[2]<23)
                {
                    TA1CCR1=deg_180;
                    contPOBRGE[4]++;
                    dutyR=17;dutyG=17;dutyB=0; //Led Verde
                }
                /*DEFECTO*/
                else /*Lectura errónea*/
                {
                    if(errores<5) //Leemos de nuevo el color, dado que puede existir cierto ruido
                        errorLectura=1;
                    else //Si hemos leido 5 veces de forma errónea, concluimos en que ha ocurrido un defecto (lacasito descolorido, de otro color,etc)
                    {
                        TA1CCR1=deg_70; //Zona de defecto
                        contPOBRGE[5]++;
                        dutyR=17;dutyG=17;dutyB=17; //Led Blanco --> Indica defecto
                    }
                    errores++;
                }
                /*Escribimos en flash la cuenta de lacasitos y defectos*/
                escribeFlash(contPOBRGE,0x1000);
            }
            else //Ha leido un hueco
            {
                dutyR=0;dutyG=0;dutyB=0; //Apagamos el Led
            }

            if(!errorLectura) //Si no ha ocurrido un error en la lectura
            {
                /*Espero a que el servo llegue a su posición final*/
                tempOn=1;
                mod=25; //0.5 Seg
                while(tempOn);

                P1IE |=BIT4; //Habilitamos interrupciones del sensor IR
                alineado = 0;
                TA1CCR2=giro1; //Ponemos en marcha el motor
                tempAtasco = 1; //Activamos el temporizador de atasco
                errores=0; //Ponemos a cero el contador de lecturas erróneas
            }
            else
                errorLectura = 0; //Si ha habido un error repito el proceso de letura

            invLectura = 0; //Deshacemos la invalidación de lectura

            /*Reevaluamos la condición de hueco/pared (Seguridad, evita que una causa externa desincronice el proceso)*/
            if(P1IN&BIT4)
                rebotes=1;
            else
                rebotes=0;

            /*Actualizamos el contador por bluetooth*/
            for(i=0;i<6;i++)
            {
                while (!(IFG2&UCA0TXIFG)); // Espera Tx libre
                UCA0TXBUF = contPOBRGE[i];
            }
        }
        /*Borramos el flag de interrupciones del sensor (Hayamos leído o no el color)*/
        I2C_RW = 0; //Escritura
        enviar[0]=regCom|borrarInt;
        tam=1;
        ind=tam;
        UCB0CTL1 |= UCTR + UCTXSTT; // Modo escritura y start
        while(I2C_RW!=3); //Me aseguro de que ha terminado
        //NOTA: De esta forma el valor disponible está siempre actualizado
    }
}
/*------Interrupciones por el bus I2C------*/
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR_HOOK(void)
{
    if(IFG2&UCB0TXIFG)  //Interrupción en la transmisión
    {
        if(I2C_RW==0) //Ciclo de escritura
        {
            if (ind) {  //Primera interrupción (Dirección recibida)
                ind--;
                UCB0TXBUF = enviar[ind]; //Enviamos el dato
            }
            else {  //Segunda interrupción (El slave ha aceptado el dato)
                UCB0CTL1 |= UCTXSTP; //Mandamos un stop
                I2C_RW=3; //Nos indica fin del proceso
                LPM0_EXIT;
            }
        }
        else if(I2C_RW==1) //Ciclo de escritura-lectura
        {
            if(ind) //Primera interrupción (El slave responde a la dirección)
            {
                ind--;
                UCB0TXBUF=enviar[ind]; //Enviamos el dato
            }
            else //Segunda interrupción (El slave ha aceptado el dato)
            {
                ind=tam; //Reseteamosel índice
                UCB0CTL1&=~UCTR; //Modo Rx (Lectura)
                UCB0CTL1 |= UCTXSTT;    //Mandamos un start
                while(UCB0CTL1&UCTXSTT); //Esperamos a que el slave lo acepte
                UCB0CTL1 |= UCTXSTP; //Enviamos un Stop
            }
        }
        IFG2 &= ~UCB0TXIFG; //Borramos el flag de Tx
    }
    else if(IFG2&UCB0RXIFG){ //Interrupción por Rx
        if(ind) //Primera interrupción (Dato en el bus)
        {
            ind--;
            leer=UCB0RXBUF;    //Leemos el bus
            I2C_RW=3;// Señaliza fin del proceso
            LPM0_EXIT;
        }
        IFG2 &= ~UCB0RXIFG; //Borramos el flag de Rx
    }
}

/*Interrupciones por el mod BT (Lectura)*/
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR_HOOK(void)
{
    char i=0;
    if(IFG2&UCA0RXIFG)
    {
        leeBT=UCA0RXBUF;
        if(leeBT == 1) //Comienza/Renauda la producción
            start = 1;
        else if(leeBT == 2) //Pausamos la producción
            start=0;
        else if(leeBT == 3) //Reseteamos el contador de lacasitos clasificados
        {
            for(i=0;i<6;i++)
                contPOBRGE[i]=0;
            escribeFlash(contPOBRGE,0x1000); //Reseteamos el contador en la flash
        }
        /*Enviamos el contador por bluetooth*/
        for(i=0;i<6;i++)
        {
            while (!(IFG2&UCA0TXIFG)); // Espera Tx libre
            UCA0TXBUF = contPOBRGE[i];
        }
        IFG2 &= ~(UCA0RXIFG);
    }
}

/*Interrupciones por el sensor IR o por el sensor de color*/
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR_HOOK(void)
{
    if(P1IFG&BIT3) //Interrupción por el sensor de color
    {
        P1IFG&=~BIT3; //Borramos el flag
        LPM0_EXIT; //Despertamos al micro
    }
    if (P1IFG&BIT4) //Interrupción por el sensor IR
    {
        if (atasco) //Si ha ocurrido un atasco hemos hecho que gire en sentido contrario hasta esta interrupción
        {
            atasco = 0;
            TA1CCR2=giro1; //Vuelve a girar en sentido horario (Atasco subsanado a priori)
            tAtasco=0;
            //NOTA: No se ha desactivado el temporizador de atasco así que si no llega ninguna interrupción
            //comenzará el proceso nuevamente
        }
        else
        {
            if(rebotes) //Se producen rebotes en el flanco de bajada (Falsos flancos de subida)
            {
                //Iniciamos un breve temporizador que filtre esos rebotes
                tempOn=1;
                mod=5;
                rebotes=0;
            }
            else if(!tempOn)
            {
                tempAtasco=0; tAtasco = 0; //Está alineado (Desactivamos y resetamos el temporizador de atasco)
                alineado=1;
                TA1CCR2=parar;
                P1IE &=~BIT4; //Desactivamos las interrupciones (Nos deshacemos de rebotes posteriores)
                tempOn=1; //Inicamos un temporizador que espera a que el sistema se estabilice antes de leer el color
                mod=25; //0.5 Seg
            }
        }
        P1IFG&=~BIT4; //Borramos el flag
    }
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR_HOOK(void)
{
    /*Temporizador de módulo variable*/
    if(tempOn) //Lo pongo a 1 de forma externa. Cuando acaba lo pongo a cero.
    {
        t++; //Cada 20 ms incrementa
        if(t>=mod)
        {
            t=0;
            tempOn=0; //Fin de la temporizador
        }
    }

    /*Temporizador que comprueba si ha ocurrido un atasco*/
    if(tempAtasco) //Se inicia el temporizador de forma externa (Al iniciar el giro del motor)
    {
        tAtasco++;
        if(tAtasco>=150) //Si el temporizador ha estado 3 segundos activado, está atascado
        {
            tAtasco=0; //Reseteamos el tiempo
            /*Invertimos el giro cada 3 seg hasta que se desatasque*/
            if(TA1CCR2 == giro1)
                TA1CCR2=giro2;
            else if (TA1CCR2 == giro2)
                TA1CCR2=giro1;

            atasco=1; //Señalizamos al gestor de las interrupciones de IR que ha ocurrido un atasco
            //No desactivamos el temporizador (Si no es resuelto se cambia el sentido de giro indefinidamente)

            /*Invalida las lecturas que se produzcan durante la resolución del atasco*/
            invLectura = 1;
        }
    }
}

/*PWM Softwate -- Control del led RGB*/
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR_HOOK(void)
{
    tleds++;
    if(tleds>=10)//16ms (Periodo)
    {
        tleds=0; //Comenzamos con los leds encendidos
        if(dutyR)
            P2OUT|=BIT3; //R
        if(dutyG)
            P2OUT|=BIT4; //G
        if(dutyB)
            P2OUT|=BIT0; //B
    }
    if (tleds==dutyR)
    {
        P2OUT&=~BIT3; //Lo apagamos cuando completa el duty-cycle
    }
    if (tleds==dutyG)
    {
        P2OUT&=~BIT4; //Lo apagamos cuando completa el duty-cycle
    }
    if (tleds==dutyB)
    {
        P2OUT&=~BIT0; //Lo apagamos cuando completa el duty-cycle
    }
}














