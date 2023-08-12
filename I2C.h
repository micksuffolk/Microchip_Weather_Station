/*
 * File: Header file to use I2C with PIC16F887 or similar
 * Author: Mike Suffolk
 */



void I2C_Hold()
{
    while (   (SSPCON2 & 0b00011111)    ||    (SSPSTAT & 0b00000100)   ) ; //check the bits on registers to make sure the IIC is not in progress
}



void I2C_Begin()
{
  I2C_Hold();  //Hold the program is I2C is busy  
  SEN = 1;     //Begin IIC pg85/234
  I2C_Status_LED = 1;
}



void I2C_End()
{
  I2C_Hold(); //Hold the program is I2C is busy  
  PEN = 1;    //End IIC pg85/234
  I2C_Status_LED = 0;
}



void I2C_Write(unsigned data)
{
  I2C_Hold(); //Hold the program is I2C is busy 
  SSPBUF = data;         //pg82/234
}



unsigned short I2C_Read(unsigned short ack)
{
  unsigned short incoming;
  I2C_Hold();
  RCEN = 1;
  
  I2C_Hold();
  incoming = SSPBUF;      //get the data saved in SSPBUF
  
  I2C_Hold();
  ACKDT = (ack)?0:1;    //check if ack bit received  
  ACKEN = 1;          //pg 85/234
  
  return incoming;
}


