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

#define XYZ_X_PIN 8
#define XYZ_Y_PIN 9
#define XYZ_Z_PIN 10

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

        // change flag
        status_flags = status_flags | STATUS_INIT;
        send2commander("[INIT] 初始化完毕,启动Scoop主循环");

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
int bx;
int by;
int bz;
int bg;
defineTask(WorkReportXYZ);
void WorkReportXYZ::setup() {
        send2commander("[XYZ] 初始化加速度芯片");
        sleep(1000);
        bx = calXYZBase(XYZ_X_PIN);
        by = calXYZBase(XYZ_Y_PIN);
        bz = calXYZBase(XYZ_Z_PIN);
        int tmp = (bx + by) / 2 ;
        bg = bz - tmp;
        bx = tmp;
        by = tmp;
        bz = tmp;
        char msg[48];
        sprintf(msg, "[XYZ] 初始值 x=%d y=%d z=%d 1g=%d", bx, by, bz, bg);
        send2commander(msg);
}

void WorkReportXYZ::loop() {
        if ((status_flags & STATUS_FLYING) > 0) {
                char xyz[32];
                char xstr[6];
                char ystr[6];
                char zstr[6];
                dtostrf(calAcc(XYZ_X_PIN, bx), 1, 2, xstr);
                dtostrf(calAcc(XYZ_Y_PIN, by), 1, 2, ystr);
                dtostrf(calAcc(XYZ_Z_PIN, bz), 1, 2, zstr);

                sprintf(xyz, "[XYZ] x=%sg y=%sg z=%sg", xstr, ystr, zstr);
                send2commander(xyz);
#ifdef DEBUG
                char cfMsg[16];
                sprintf(cfMsg, "[xyz debug] %d", stackLeft());
                Serial.println(cfMsg);
#endif
                sleep(300);
        }
}

// task to report gps
defineTask(WorkReportGPS);
void WorkReportGPS::setup() {
}

void WorkReportGPS::loop() {
        if ((status_flags & STATUS_FLYING) > 0) {
                send2commander("[GPS] lat lng");
                yield();
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

int calXYZBase(int pin) {
        int result = analogRead(pin);
        int i = 0;
        while(i++ < 10) {
                result += analogRead(pin);
                result /= 2;
        }
        return result;
}

float calAcc(int pin, int base) {
        return ((float) (analogRead(pin) - base) / bg);
}
