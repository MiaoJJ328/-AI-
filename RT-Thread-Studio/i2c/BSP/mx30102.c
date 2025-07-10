#include <rtthread.h>
#include "mx30102.h"
#include <rtdevice.h>
#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
/* ȫ�ֱ������� */
uint16_t fifo_red ;
uint16_t fifo_ir ;

/* ˽�б��� */
 struct rt_i2c_bus_device *i2c_bus = RT_NULL;

/***********************************************************
 * ��ʼ��������������
 ***********************************************************/
void max30102_init(const char *i2c_bus_name)
{
    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(i2c_bus_name);
    if(i2c_bus != RT_NULL)
    {
        rt_kprintf("i2c init succeed\r\n");

    }
    else
    {
        rt_kprintf("can not find\r\n");
    }

}

/***********************************************************
 * ����ԭ�к������ƺͲ���
 ***********************************************************/
uint8_t max30102_write_reg(uint8_t addr, uint8_t data)
{
    struct rt_i2c_msg msg;
    uint8_t buf[2] = {addr, data};

    if (!i2c_bus)
    {
        rt_kprintf("i2c_bus can not find\r\n");
    }

    msg.addr  = MAX30102_Device_address;
    msg.flags = RT_I2C_WR;
    msg.buf   = buf;
    msg.len   = 2;
    rt_size_t ret = rt_i2c_transfer(i2c_bus, &msg, 1);
    if (ret != 1)
    {
        rt_kprintf("Write failed! ret=%d\n", ret);
        return 0;
    }
    return 1;
}

uint8_t max30102_read_reg(uint8_t addr)
{
    struct rt_i2c_msg msgs[2];
    uint8_t reg = addr;
    uint8_t data = 0;

    if (!i2c_bus) return 0xFF;

    /* д�Ĵ�����ַ */
    msgs[0].addr  = MAX30102_Device_address;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg;
    msgs[0].len   = 1;

    /* ������ */
    msgs[1].addr  = MAX30102_Device_address;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = &data;
    msgs[1].len   = 1;

    /* 执行传输 */
    rt_size_t ret = rt_i2c_transfer(i2c_bus, msgs, 2);
    if (ret != 2)
    {
        rt_kprintf("Read failed! ret=%d\n", ret);
        return 0xFF;
    }
    return data;
}

uint8_t Max30102_reset(void)
{
    return max30102_write_reg(REG_MODE_CONFIG, 0x40);
}

void MAX30102_Config(void)
{
    /* ����ԭ���������� */
    max30102_write_reg(REG_INTR_ENABLE_1, 0xc0);
    max30102_write_reg(REG_INTR_ENABLE_2, 0x00);
    max30102_write_reg(REG_FIFO_WR_PTR, 0x00);
    max30102_write_reg(REG_OVF_COUNTER, 0x00);
    max30102_write_reg(REG_FIFO_RD_PTR, 0x00);

    max30102_write_reg(REG_FIFO_CONFIG, 0x0f);
    max30102_write_reg(REG_MODE_CONFIG, 0x03);
    max30102_write_reg(REG_SPO2_CONFIG, 0x27);
    max30102_write_reg(REG_LED1_PA, 0x32);
    max30102_write_reg(REG_LED2_PA, 0x32);
    max30102_write_reg(REG_PILOT_PA, 0x7f);

}

//void max30102_read_fifo(void)
//{
//    struct rt_i2c_msg msgs[2];
//    uint8_t reg = REG_FIFO_DATA;
//    uint8_t ach_i2c_data[6];
//
//    if (!i2c_bus)
//    {
//        rt_kprintf("i2c设备错误！\r\n");
//        return;
//    }
//
//    /* ����ж�״̬ */
//    max30102_read_reg(REG_INTR_STATUS_1);
//    max30102_read_reg(REG_INTR_STATUS_2);
//
//    /* �����Ϣ��д�Ĵ�����ַ + ������ */
//    msgs[0].addr  = MAX30102_Device_address;
//    msgs[0].flags = RT_I2C_WR;
//    msgs[0].buf   = &reg;
//    msgs[0].len   = 1;
//
//    msgs[1].addr  = MAX30102_Device_address;
//    msgs[1].flags = RT_I2C_RD;
//    msgs[1].buf   = ach_i2c_data;
//    msgs[1].len   = 6;
//
//    if (rt_i2c_transfer(i2c_bus, msgs, 2) == 2) {
//        /* ����ԭ�����ݴ����߼� */
//        fifo_red = (ach_i2c_data[0] << 14) | (ach_i2c_data[1] << 6) | (ach_i2c_data[2] >> 2);
//        fifo_ir = (ach_i2c_data[3] << 14) | (ach_i2c_data[4] << 6) | (ach_i2c_data[5] >> 2);
//
//        if (fifo_ir <= 10000) fifo_ir = 0;
//        if (fifo_red <= 10000) fifo_red = 0;
//    }
//}

void max30102_read_fifo(void)
{
    uint16_t un_temp;
    fifo_red = 0;
    fifo_ir = 0;
    uint8_t ach_i2c_data[6];
    uint8_t reg = REG_FIFO_DATA;
    struct rt_i2c_msg msgs[2];
    /* 读取并清除中断状态寄存器 */
    max30102_read_reg(REG_INTR_STATUS_1);
    max30102_read_reg(REG_INTR_STATUS_2);

    /* д�Ĵ�����ַ */
    msgs[0].addr  = MAX30102_Device_address;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &reg;
    msgs[0].len   = 1;

    /* ������ */
    msgs[1].addr  = MAX30102_Device_address;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf   = ach_i2c_data;
    msgs[1].len   = 6;

    /* 执行传输 */
    rt_size_t ret = rt_i2c_transfer(i2c_bus, msgs, 2);
    if (ret != 2)
    {
        rt_kprintf("Read failed! ret=%d\n", ret);
        return;
    }

    /*****数据解析*******/
    un_temp=ach_i2c_data[0];
    un_temp<<=14;
    fifo_red+=un_temp;
    un_temp=ach_i2c_data[1];
    un_temp<<=6;
    fifo_red+=un_temp;
    un_temp=ach_i2c_data[2];
    un_temp>>=2;
    fifo_red+=un_temp;

    un_temp=ach_i2c_data[3];
    un_temp<<=14;
    fifo_ir+=un_temp;
    un_temp=ach_i2c_data[4];
    un_temp<<=6;
    fifo_ir+=un_temp;
    un_temp=ach_i2c_data[5];
    un_temp>>=2;
    fifo_ir+=un_temp;

    if(fifo_ir<=10000)
    {
        fifo_ir=0;
    }
    if(fifo_red<=10000)
    {
        fifo_red=0;
    }

}


