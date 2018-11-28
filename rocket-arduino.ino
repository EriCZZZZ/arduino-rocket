/**
 * 命令说明
 * l(aunch) - 进入发射流程
 * o(pen)   - 强制立即开伞
 * s(top)   - 进入回收流程, 清理数据
 */
#include <SCoop.h>

//#define DEBUG 

#define STATUS_UN_INIT 0
#define STATUS_INIT 1
#define STATUS_WAIT_LAUNCH 2
#define STATUS_LAUNCHING 4
#define STATUS_FLYING 8
#define STATUS_OPEN_PARACHUTE 16

#define COMMANDER_PORT_RATE 9600

/***************
 * global vars *
 ***************/
// rocket flags
volatile int status_flags = 0;
// flags mutex
boolean mutex_status_flags = false;

/****************
 * global funcs *
 ****************/
void send2commander(char msg[]);

/***************
 * bootloader *
 ***************/
void setup() {
        // init communication with commander
        Serial.begin(COMMANDER_PORT_RATE);
        send2commander("[INIT] 初始化系统");
        sendStatusFlags2Commander();

        // change flag
        status_flags = status_flags | STATUS_INIT;
        send2commander("[INIT] 初始化完毕,启动Scoop主循环");
        sendStatusFlags2Commander();

        // start SCoop
        mySCoop.start();
}

void loop() {
        yield();
}

// task to recv command from serial
defineTask(RecvCommand);
void RecvCommand::setup() {
}

void RecvCommand::loop() {
        if((status_flags & STATUS_INIT) > 0 && Serial.available()) {
                char cmd = Serial.read();
                switch(cmd) {
                        case 'l':
                            send2commander("[RECV COMMAND] 发射");
                            status_flags |= STATUS_WAIT_LAUNCH;
                            break;
                        case 'o':
                            send2commander("[RECV COMMAND] 开降落伞");
                            break;
                        case 's':
                            send2commander("[RECV COMMAND] 开始回收流程,清理数据");
                            break;
                        case '\n':
                            break;
                        default:
                            send2commander("[RECV COMMAND] UNKNOWN COMMAND");
                            break;
                }
        }
}

// task to report xyz
defineTask(WorkReportXYZ);
void WorkReportXYZ::setup() {

}

void WorkReportXYZ::loop() {
        if ((status_flags & STATUS_FLYING) > 0) {
#ifdef DEBUG
                sendStatusFlags2Commander();
                char cfMsg[16];
                sprintf(cfMsg, "[xyz debug] %d", status_flags & STATUS_WAIT_LAUNCH);
                Serial.println(cfMsg);
#endif
                send2commander("[XYZ] x - 1 y - 2 z - 3");
                yield();
                sleep(1000);
        }
}

// task to report gps
defineTask(WorkReportGPS);
void WorkReportGPS::setup() {

}

void WorkReportGPS::loop() {
        if ((status_flags & STATUS_FLYING) > 0) {
                send2commander("[GPS] lat lng");
                sleep(1000);
        }
}

// task to launch
defineTask(LaunchRocket);
void LaunchRocket::setup() {

}
void LaunchRocket::loop() {
        if(((status_flags & STATUS_WAIT_LAUNCH) > 0) && ((status_flags & STATUS_LAUNCHING) == 0)) {
                status_flags |= STATUS_LAUNCHING;
                int seconds2wait = 5;
                char msg[16];
                while(seconds2wait > 0) {
                        sprintf(msg, "[LAUNCH] %d s", seconds2wait--);
                        send2commander(msg);
                        yield();
                        sleep(1000);
                }
                send2commander("[LAUNCH] 点火");
                status_flags |= STATUS_FLYING;
        }
}

/**
 * utils
 */
void send2commander(char msg[]) {
        Serial.println(msg);
}
void sendStatusFlags2Commander() {
        char msg[16];
        sprintf(msg, "[FLAGS] %d", status_flags);
        Serial.println(msg);
}
