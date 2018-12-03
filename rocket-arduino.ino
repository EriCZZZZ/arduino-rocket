/**
 * ********************
 * 接线
 * 
 * 无线接Serial1
 * gps接Serial2
 * 加速度x-y-z接analog8-9-10
 * 舵机 红-5v 棕-GND 黄-Digital52
 * **重要 注意Digital两排接口两头分别为GND和5V的接口
 * ********************
 * 无线芯片配置
 * http://pan.baidu.com/s/1sj9lCzN  DL-20
 * 
 * 已经配成9600-点对点开机-同一频道
 * 
 * 配置手册pdf中灯光示意
 * 红    红
 * 绿    绿
 *   排针
 * 
 * 无线通信大概有1秒的延迟
 * ********************
 */
#include "pt.h"
#include "pt-timer-arduino.h"
#include <Servo.h>

#define DEBUG

#ifdef DEBUG
#define SERIAL_COMMANDER Serial
#else
#define SERIAL_COMMANDER Serial1
#endif
#define BAUD_RATE_COMMANDER 9600
#define SERIAL_GPS Serial2
#define BAUD_RATE_GPS 9600

#define STATUS_WAIT_LAUNCH 1
#define STATUS_LAUNCHING 2
#define STATUS_FLYING 3
#define STATUS_OPEN_PARACHUTE 4

#define SECONDS_BEFORE_LAUNCH 3

#define PIN_MOTO 52
#define PIN_ACC_X 8
#define PIN_ACC_Y 9
#define PIN_ACC_Z 10

static int status_flags = 0;

static struct pt pt_taskWirelessSend;
static struct pt pt_taskWirelessRecv;
static struct pt pt_taskCommander;
static struct pt pt_taskLaunch;
static struct pt pt_taskAcc;
static struct pt pt_taskGPS;
static struct pt pt_taskOpenParachute;

static unsigned long ts_system_start;

/**
 * bootloader
 */
void setup() {
        // 记录开机时间
        ts_system_start = millis();

        // 初始化任务
        PT_INIT(&pt_taskCommander);
        PT_INIT(&pt_taskLaunch);
        PT_INIT(&pt_taskAcc);
        PT_INIT(&pt_taskGPS);
        PT_INIT(&pt_taskOpenParachute);

        // 初始化串行IO
        SERIAL_COMMANDER.begin(BAUD_RATE_COMMANDER);
        SERIAL_GPS.begin(BAUD_RATE_GPS);

        // init motor
        initMotor();

        // init acc
        initAcc();

        send2commander("[SETUP] 初始化完成");
        send2commander("[SYSTEM] 开机时间 " + String(ts_system_start));
}

void loop() {
        taskCommander(&pt_taskCommander);
        taskLaunch(&pt_taskLaunch);
        taskAcc(&pt_taskAcc);
        taskGPS(&pt_taskGPS);
        taskOpenParachute(&pt_taskOpenParachute);
}

/**
 * tasks
 */
// 处理接受命令
static pta_timer timer_commander;
static String command;
static int taskCommander(struct pt *pt) {
        PT_BEGIN(pt);
        send2commander("[COMMANDER] 等待接受命令 LAUNCH - 发射 PARACHUTE - 强制开伞 MONITOR - 状态");
        while(true) {
                PT_WAIT_UNTIL_OR_TIMEOUT(pt, SERIAL_COMMANDER.available(), &timer_commander, 3000);
                if(SERIAL_COMMANDER.available()) {
                        // 处理收到的命令
                        command = SERIAL_COMMANDER.readString();
                        command = command.substring(0, command.length() - 1);

                        send2commander("[COMMANDER] 收到命令 " + command);

                        if(command.equalsIgnoreCase("launch")) {
                                status_flags = STATUS_WAIT_LAUNCH;
                        } else if(command.equalsIgnoreCase("parachute")) {
                                status_flags = STATUS_OPEN_PARACHUTE;
                        } else if(command.equalsIgnoreCase("monitor")) {
                                send2commander("[MONITOR] status_flags " + String(status_flags));
                                send2commander("[MONITOR] 开机毫秒数 " + String(millis() - ts_system_start));
                        } else {
                                send2commander("[COMMANDER] 未知命令");
                        }
                } else {
                        send2commander("[COMMANDER] 等待接受命令 LAUNCH - 发射 PARACHUTE - 强制开伞 MONITOR - 状态");
                }
                
                PT_YIELD(pt); // ***important***
        }
        PT_END(pt);
}

// 点火
static pta_timer timer_launch;
static int counter_launch = SECONDS_BEFORE_LAUNCH;
static int taskLaunch(struct pt *pt) {
        PT_BEGIN(pt);
        // 等待发射命令
        PT_WAIT_UNTIL(pt, status_flags == STATUS_WAIT_LAUNCH);
        status_flags = STATUS_LAUNCHING;
        while(counter_launch > 0) {
                send2commander("[LAUNCH] 倒计时 " + String(counter_launch) + "s");
                counter_launch--;
                PT_DELAY_MILLIS(pt, &timer_launch, 1000);
        }
        // todo 点火
        send2commander("[LAUNCH] 触发点火 todo");
        status_flags = STATUS_FLYING;
        PT_END(pt);
}

// 加速度
static pta_timer timer_acc;
// base val of xyz
static int bx;
static int by;
static int bz;
static int bg; // val / g
static char accMsg[64];
static char accXStr[8];
static char accYStr[8];
static char accZStr[8];
static void initAcc() {
        send2commander("[ACC] 初始化数据");
        bx = calXYZBase(PIN_ACC_X);
        by = calXYZBase(PIN_ACC_Y);
        bz = calXYZBase(PIN_ACC_Z);

        send2commander("[ACC] 初始化x=" + String(bx) + " y=" + String(by) + " z=" + String(bz));
        bx = (bx + by) / 2;
        by = bx;
        bg = bz - bx;
        bz = bx;
        send2commander("[ACC] 初始化完成");
}
static int taskAcc(struct pt *pt) {
        PT_BEGIN(pt);
        // 等待发射
        PT_WAIT_UNTIL(pt, status_flags == STATUS_FLYING);
        while(true) {
                dtostrf(calAcc(PIN_ACC_X, bx), 1, 2, accXStr);
                dtostrf(calAcc(PIN_ACC_Y, by), 1, 2, accYStr);
                dtostrf(calAcc(PIN_ACC_Z, bz), 1, 2, accZStr);
                sprintf(accMsg, "[ACC] x=%sg y=%sg z=%sg", accXStr, accYStr, accZStr);
                send2commander(accMsg);
                PT_DELAY_MILLIS(pt, &timer_acc, 1000);
        }
        PT_END(pt);
}
int calXYZBase(int pin) {
        int result = analogRead(pin);
        int i = 0;
        while(i++ < 10) {
                result += analogRead(pin);
                result /= 2;
                delay(30);
        }
        return result;
}
float calAcc(int pin, int base) {
        return ((float) analogRead(pin) - base) / bg;
}

// gps
static pta_timer timer_gps;
static int taskGPS(struct pt *pt) {
        PT_BEGIN(pt);
        // 等待发射
        PT_WAIT_UNTIL(pt, status_flags == STATUS_FLYING);
        while(true) {
                send2commander("[gps]");
                PT_DELAY_MILLIS(pt, &timer_gps, 1000);
        }
        PT_END(pt);
}

// 开伞
static pta_timer timer_parachute;
static Servo parachute;
static int currentMotorDegree;

static void initMotor() {
        parachute.attach(PIN_MOTO);
        currentMotorDegree = parachute.read();
        send2commander("[PARACHUTE MOTOR] Current degree " + String(currentMotorDegree) + "°");
} 

static int taskOpenParachute(struct pt * pt) {
        PT_BEGIN(pt);
        PT_WAIT_UNTIL(pt, status_flags == STATUS_OPEN_PARACHUTE);
        parachute.write((currentMotorDegree + 90) % 180);
        PT_END(pt);
        PT_EXIT(pt);
}

static void send2commander(String s) {
        SERIAL_COMMANDER.println(s);
}
