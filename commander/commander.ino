/******************
 * 用于远程控制端
 * Author : Er1c
 * ****************
 * 无线芯片接串口3
 ******************/

#define WIRELESS_SERIAL Serial3
#define WIRELESS_BAUD_RATE 9600
#define PC_SERIAL Serial
#define PC_BAUD_RATE 9600

void setup() {
        // 初始化串行接口
        PC_SERIAL.begin(PC_BAUD_RATE);
        WIRELESS_SERIAL.begin(WIRELESS_BAUD_RATE);
}

void loop() {
        // 转发数据
        if(WIRELESS_SERIAL.available()) {
                PC_SERIAL.write(WIRELESS_SERIAL.read());
        }
        if(PC_SERIAL.available()) {
                WIRELESS_SERIAL.write(PC_SERIAL.read());
        }
}
