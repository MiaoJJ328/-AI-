
#include "blood.h"
#include <rtthread.h>
#include "mx30102.h"
int heart;      //????
float SpO2;     //???????
rt_thread_t health_thread=RT_NULL;
void health_thread_start(void)
{
    health_thread=rt_thread_create("health", health_entry, RT_NULL,1024, 20, 20);
       if(health_thread==RT_NULL)
       {
           rt_kprintf("health_thread failed...\r\n");
       }
       else{
           rt_kprintf("health_thread succeed\r\n");
           rt_thread_startup(health_thread);
       }
}
//??????
extern rt_uint16_t fifo_red;       //??FIFO??????
extern rt_uint16_t fifo_ir;        //??FIFO???????

rt_uint16_t g_fft_index = 0;               //fft??????
struct compx s1[FFT_N+16];              //FFT?????:?S[1]????,????????
struct compx s2[FFT_N+16];              //FFT?????:?S[1]????,????????



#define CORRECTED_VALUE         47              //????????


void blood_data_update(void)
{
    //??????? ??FIFO
    g_fft_index=0;
    while(g_fft_index < FFT_N)
    {
        while(max30102_read_reg(REG_INTR_STATUS_1)&0x40 )
        {
            //??FIFO
            max30102_read_fifo();  //read from MAX30102 FIFO2
            //?????fft???????
            if(g_fft_index < FFT_N)
            {
                //?????fft???????
                s1[g_fft_index].real = fifo_red;
                s1[g_fft_index].imag= 0;
                s2[g_fft_index].real = fifo_ir;
                s2[g_fft_index].imag= 0;
                g_fft_index++;
            }
        }
    }
}



void blood_data_translate(void)
{
    float n_denom;
    uint16_t i;

    //????
    float dc_red =0;
    float dc_ir =0;
    float ac_red =0;
    float ac_ir =0;

    for (i=0 ; i<FFT_N ; i++ )
    {
        dc_red += s1[i].real ;
        dc_ir +=  s2[i].real ;
    }
        dc_red =dc_red/FFT_N ;
        dc_ir =dc_ir/FFT_N ;
    for (i=0 ; i<FFT_N ; i++ )
    {
        s1[i].real =  s1[i].real - dc_red ;
        s2[i].real =  s2[i].real - dc_ir ;
    }

    //??????
    for(i = 1;i < FFT_N-1;i++)
    {
        n_denom= ( s1[i-1].real + 2*s1[i].real + s1[i+1].real);
        s1[i].real=  n_denom/4.00;

        n_denom= ( s2[i-1].real + 2*s2[i].real + s2[i+1].real);
        s2[i].real=  n_denom/4.00;
    }

    //??????
    for(i = 0;i < FFT_N-8;i++)
    {
        n_denom= ( s1[i].real+s1[i+1].real+ s1[i+2].real+ s1[i+3].real+ s1[i+4].real+ s1[i+5].real+ s1[i+6].real+ s1[i+7].real);
        s1[i].real=  n_denom/8.00;

        n_denom= ( s2[i].real+s2[i+1].real+ s2[i+2].real+ s2[i+3].real+ s2[i+4].real+ s2[i+5].real+ s2[i+6].real+ s2[i+7].real);
        s2[i].real=  n_denom/8.00;

    }

    //??????
    g_fft_index = 0;
    //???????
    FFT(s1);
    FFT(s2);

    for(i = 0;i < FFT_N;i++)
    {
        s1[i].real=sqrtf(s1[i].real*s1[i].real+s1[i].imag*s1[i].imag);
        s1[i].real=sqrtf(s2[i].real*s2[i].real+s2[i].imag*s2[i].imag);
    }
    //??????
    for (i=1 ; i<FFT_N ; i++ )
    {
        ac_red += s1[i].real ;
        ac_ir +=  s2[i].real ;
    }

    for(i = 0;i < 50;i++)
    {
        if(s1[i].real<=10)
            break;
    }

    //?????????  ????????
    int s1_max_index = find_max_num_index(s1, 60);
    int s2_max_index = find_max_num_index(s2, 60);

    //??HbO2?Hb?????????
    if(i>=45)
    {
        //????
        uint16_t Heart_Rate = 60.00 * SAMPLES_PER_SECOND * s1_max_index / FFT_N;
        heart = Heart_Rate;

        //??????
        float R = (ac_ir*dc_red)/(ac_red*dc_ir);
        float sp02_num = -45.060*R*R+ 30.354 *R + 94.845;
        SpO2 = sp02_num;

        //????
    }
    else //??????
    {
        heart = 0;
        SpO2 = 0;
    }
    //??????
}


void blood_Loop(void)
{

    blood_data_update();

    blood_data_translate();

    SpO2 = (SpO2 > 99.99) ? 99.99:SpO2;

    rt_kprintf("heartrate%3d/min;blood_oxygen%2d%%\n", heart, (int)SpO2);

}

void health_entry(void)
{
        max30102_init("i2c2");
        Max30102_reset();
        rt_thread_delay(300);
        MAX30102_Config();
        rt_kprintf("please wait for 10-20 seconds\r\n");
        rt_thread_mdelay(300);
        while(1)
        {
            blood_Loop();
            rt_thread_mdelay(300);
        }

}
