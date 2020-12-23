/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */

/* For usleep() */
/* For usleep() */
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>



/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>


/* Driver configuration */
#include "ti_drivers_config.h"

#define TASKSTACKSIZE       640



uint8_t txBuffer[1];
uint8_t rxBuffer[2];



int humd,temp;

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    bool            retVal = false;
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;




    /* Call driver init functions */
    GPIO_init();

    I2C_init();



    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
            printf("Error Initializing I2");
    }
    else {
        printf("Initializing I2\n");

    }


             //txBuffer[1] = 16;
             i2cTransaction.slaveAddress = 0x40;
             i2cTransaction.writeBuf = txBuffer;
             i2cTransaction.writeCount = 1;
             txBuffer[0] = 0xE3;
             i2cTransaction.readBuf = rxBuffer;
             i2cTransaction.readCount = 1;

             rxBuffer[1] &= ~(1<<0);
             rxBuffer[1] &= ~(1<<1);

             /* Re-try writing to slave till I2C_transfer returns true */
             do
             {
                 retVal = I2C_transfer(i2c, &i2cTransaction);
             } while(!retVal);


            i2cTransaction.slaveAddress = 0x40;
            i2cTransaction.writeCount = 1;
            i2cTransaction.writeBuf = txBuffer;
            txBuffer[0] = 0xE5;
            i2cTransaction.readCount = 1;
            i2cTransaction.readBuf = rxBuffer;

            rxBuffer[1] &= ~(1<<0);
            rxBuffer[1] &= ~(1<<1);
            /* Re-try writing to slave till I2C_transfer returns true */
            do {
                retVal = I2C_transfer(i2c, &i2cTransaction);
            } while(!retVal);



            while(1)
            {

            //txBuffer[1] = 8;
            i2cTransaction.slaveAddress = 0x40;
            i2cTransaction.writeCount = 1;
            i2cTransaction.writeBuf = txBuffer;
            txBuffer[0] = 0xE3;
            sleep(1);
            i2cTransaction.readCount = 1;
            i2cTransaction.readBuf = rxBuffer;


            do {
                retVal = I2C_transfer(i2c, &i2cTransaction);
            } while(!retVal);


            rxBuffer[1] &= ~(1<<0);
            rxBuffer[1] &= ~(1<<1);

            temp= rxBuffer[0]<<8|rxBuffer[1];
            //printf("temp->%d\n",temp);

           // temperature=-46.85+175.72*(temp/65536);
            temp *= 175.72;
            temp /= 65536;
            temp -= 46.85;
            printf("\n");
            printf("temperature->%d°C\n",temp);
            /* Re-try reading from slave till I2C_transfer returns true */

            //txBuffer[1] = 8;
            i2cTransaction.slaveAddress = 0x40;
            i2cTransaction.writeCount = 1;
            i2cTransaction.writeBuf = txBuffer;
            txBuffer[0] = 0xE5;
            sleep(1);
            i2cTransaction.readCount = 1;
            i2cTransaction.readBuf = rxBuffer;

            do {
                   retVal = I2C_transfer(i2c, &i2cTransaction);
               } while(!retVal);


            rxBuffer[1] &= ~(1<<0);
            rxBuffer[1] &= ~(1<<1);

            humd = rxBuffer[0]<<8|rxBuffer[1];
           //printf("humd->%d\n",humd);
            // Humidity=-6+125*(temp/65536);

            humd *= 125;
            humd /= 65536;
            humd -= 6;
            printf("\n");

            printf("Humidity->%d%%RH\n",humd);




   }

            I2C_close(i2c);

            return (0);
}
