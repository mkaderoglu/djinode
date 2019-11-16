//#include <dji_telemetry.hpp>
#include "../lib/io/src/sio_client.h"
#include <iostream>
#include <dji_vehicle.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <ctime>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using namespace sio;
using namespace std;

char func[50];
std::mutex _lock;
std::condition_variable_any _cond;
socket::ptr current_socket;
sio::client h;
sio::client w;
char* vers;

std::vector <DJI::OSDK::Vehicle> araclar_drone;
std::vector <int> araclar_id;
std::vector <string> araclar_serial;


int freq                    = 50;
int pkgIndex                = 0;
int functionTimeout         = 1;
int participants            = -1;
bool connect_finish         = false;
int responseTimeout         = 1;

TopicName topicListAll[]    = {TOPIC_QUATERNION, TOPIC_GPS_FUSED, TOPIC_VELOCITY, TOPIC_RC_WITH_FLAG_DATA, TOPIC_RTK_CONNECT_STATUS, TOPIC_POSITION_VO,TOPIC_ALTITUDE_FUSIONED, TOPIC_ALTITUDE_BAROMETER, TOPIC_HEIGHT_FUSION, TOPIC_BATTERY_INFO,TOPIC_FLIGHT_ANOMALY,TOPIC_HEIGHT_HOMEPOINT, TOPIC_GPS_POSITION, TOPIC_GPS_VELOCITY, TOPIC_GPS_SIGNAL_LEVEL,TOPIC_GPS_DATE, TOPIC_GPS_TIME, TOPIC_GPS_DETAILS, TOPIC_STATUS_FLIGHT, TOPIC_STATUS_DISPLAYMODE};

TopicName topicListGo[]     = {TOPIC_QUATERNION, TOPIC_GPS_FUSED};
TopicName topicList50Hz[]   = {TOPIC_VELOCITY, TOPIC_RC_WITH_FLAG_DATA, TOPIC_RTK_CONNECT_STATUS, TOPIC_POSITION_VO,TOPIC_ALTITUDE_FUSIONED, TOPIC_ALTITUDE_BAROMETER, TOPIC_HEIGHT_FUSION, TOPIC_BATTERY_INFO,TOPIC_FLIGHT_ANOMALY};
TopicName topicList1Hz[]    = {TOPIC_HEIGHT_HOMEPOINT, TOPIC_GPS_POSITION, TOPIC_GPS_VELOCITY, TOPIC_GPS_SIGNAL_LEVEL,TOPIC_GPS_DATE, TOPIC_GPS_TIME, TOPIC_GPS_DETAILS};
TopicName topicList10Hz[]   = {TOPIC_STATUS_FLIGHT, TOPIC_STATUS_DISPLAYMODE};
string Start                = "start";
string End                  = "end";


#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"


#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

string
getTime(){
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%H:%M:%S",timeinfo);
    std::string str(buffer);
    return "[CMC Savunma "+str+"]";
};

void LOG_RED(std::string s) {
    char X[s.size() + 1];
    strcpy(X, s.c_str());
    printf("%s %s %s %s %s",getTime().c_str(), ANSI_COLOR_RED, X, ANSI_COLOR_RESET, "\n");
};
void LOG_GREEN(std::string s) {
    char X[s.size() + 1];
    strcpy(X, s.c_str());
    printf("%s %s %s %s %s",getTime().c_str(), ANSI_COLOR_GREEN, X, ANSI_COLOR_RESET, "\n");
};
void LOG_YELLOW(std::string s) {
    char X[s.size() + 1];
    strcpy(X, s.c_str());
    printf("%s %s %s %s %s",getTime().c_str(), ANSI_COLOR_YELLOW, X, ANSI_COLOR_RESET, "\n");
};


void LOG_CYAN(std::string s) {
    char X[s.size() + 1];
    strcpy(X, s.c_str());
    printf("%s %s %s %s", ANSI_COLOR_CYAN, X, ANSI_COLOR_RESET, "\n");
};
void LOG_MAGENTA(std::string s) {
    time_t now = chrono::system_clock::to_time_t(chrono::system_clock::now());
    char X[s.size() + 1];
    strcpy(X, s.c_str());
    printf("%s %s %s %s", ANSI_COLOR_MAGENTA, X, ANSI_COLOR_RESET, "\n");
};

int findInVector(std::vector<int> dizi,int deger)
{
    int a=-1;
    for (int i = 0; i < dizi.size(); i++) {
        if (deger==dizi[i]){
            return i;
        }
    };
    return a;
}
int findInVectorString(std::vector<string> dizi,string deger)
{
    int a=-1;
    for (int i = 0; i < dizi.size(); i++) {
        if (deger==dizi[i]){
            return i;
        }
    };
    return a;
}
int findInVectorStringFromDroneId(std::vector<string> dizi,string deger)
{
    int a=-1;
    for (int i = 0; i < dizi.size(); i++) {
        if (deger==dizi[i]){
            return araclar_id[i];
        }
    };
    return a;
}


class LinuxSetup{
    public:
        LinuxSetup(int app_id,string app_key,string device,int baudrate, bool enableAdvancedSensing = false);
        ~LinuxSetup();

    public:
        void setupEnvironment(int app_id,string app_key,string device,int baudrate);
        void initVehicle(int app_id,string app_key,string device,int baudrate);
        bool validateSerialPort();

    public:
        void setTestSerialDevice(DJI::OSDK::LinuxSerialDevice* serialDevice);
        DJI::OSDK::Vehicle* getVehicle(){return this->vehicle;}
        DJI::OSDK::Vehicle::ActivateData* getActivateData(){return &activateData;}

    private:
        DJI::OSDK::Vehicle*              vehicle;
        DJI::OSDK::LinuxSerialDevice*    testSerialDevice;
        DJI::OSDK::Vehicle::ActivateData activateData;
        int                              functionTimeout;
        bool                             useAdvancedSensing;
};
LinuxSetup::LinuxSetup(int app_id,string app_key,string device,int baudrate, bool enableAdvancedSensing){
    this->functionTimeout     = 1;
    this->vehicle             = nullptr;
    this->testSerialDevice    = nullptr;
    this->useAdvancedSensing  = enableAdvancedSensing;
    setupEnvironment(app_id,app_key,device,baudrate);
    initVehicle(app_id,app_key,device,baudrate);
}
LinuxSetup::~LinuxSetup(){
    if (vehicle){
        delete (vehicle);
        vehicle = nullptr;
    }
    if (testSerialDevice){
        delete (testSerialDevice);
        testSerialDevice = nullptr;
    }
}
void LinuxSetup::setupEnvironment(int app_id,string app_key,string device,int baudrate){
    const char* acm_dev_prefix = "/dev/ttyACM";
    std::string config_file_path;
    std::string acm_device_path = "";
    std::string sample_case_name = "";
}
void LinuxSetup::initVehicle(int app_id, string app_key, string device, int baudrate){
    bool threadSupport = true;

    char chDevice[device.size() + 1];
    strcpy(chDevice, device.c_str());


    this->vehicle = new Vehicle(chDevice,baudrate,true,false);
    //LOG_MAGENTA("Ara Nokta 0001 ["+to_string(vehicle->getFwVersion())+"]");

    if (!vehicle->protocolLayer->getDriver()->getDeviceStatus())
    {
        LOG_RED("Ayarlarınızı kontrol ediniz.");
        delete (vehicle);
        this->vehicle     = nullptr;
        return;
    }
    activateData.ID = app_id;
    char app_keys[65];
    activateData.encKey = app_keys;

    char chApp_key[app_key.size() + 1];
    strcpy(chApp_key, app_key.c_str());

    strcpy(activateData.encKey, chApp_key);
    activateData.version = vehicle->getFwVersion();
    ACK::ErrorCode ack   = vehicle->activate(&activateData, functionTimeout);
    if (ACK::getError(ack)){
        LOG_RED("Cihazınızın açık olduğundan emin olunuz.");
        //ACK::getErrorCodeMessage(ack, __func__);
        //delete (vehicle);
        //delete (environment);
        this->vehicle     = nullptr;
        return;
    }

    //LOG_CYAN("Kontrol Noktası 05");
}
bool LinuxSetup::validateSerialPort(){
    static const int BUFFER_SIZE = 2048;
    uint8_t buf[BUFFER_SIZE];
    if (!testSerialDevice->setSerialPureTimedRead())
    {
        LOG_RED("Bağlantı noktası ayarlanamadı.\n");
        return (false);
    };
    usleep(100000);
    if (testSerialDevice->serialRead(buf, BUFFER_SIZE))
    {
        LOG_GREEN("Seri bağlantı başarı ile oluşturuldu.\n");
    }
    else
    {
        LOG_RED("Seri cihazdan okunamadı. Onboard SDK, dronunuzla iletişim kurmuyor.");
        return (false);
    }
    int baudCheckStatus = testSerialDevice->checkBaudRate(buf);
    if (baudCheckStatus == -1)
    {
        LOG_RED("Veri okunamadı. Drone açık olmayabilir.");

        return false;
    }
    if (baudCheckStatus == -2)
    {
        LOG_RED("Baud hızı uyumsuzluğu bulundu. DJI Assistant 2'nin, CMC Programında ki ile aynı baud ayarına sahip olduğundan emin olun.");
        return (false);
    }
    testSerialDevice->unsetSerialPureTimedRead();
    return (true);
}
Telemetry::Vector3f toEulerAngle(void *quaternionData) {
    Telemetry::Vector3f ans;
    Telemetry::Quaternion *quaternion = (Telemetry::Quaternion *) quaternionData;

    double q2sqr = quaternion->q2 * quaternion->q2;
    double t0 = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
    double t1 = +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
    double t2 = -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
    double t3 = +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
    double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;

    ans.x = asin(t2);
    ans.y = atan2(t3, t4);
    ans.z = atan2(t1, t0);

    return ans;
}
sio::message::ptr getData(Vehicle *vehicle,int drone_id) {
    TypeMap<TOPIC_VELOCITY>::type velocity;
    TypeMap<TOPIC_RC_WITH_FLAG_DATA>::type rc_with_flag_data;
    TypeMap<TOPIC_RTK_CONNECT_STATUS>::type rtk_connect_status;
    TypeMap<TOPIC_POSITION_VO>::type position_vo;
    TypeMap<TOPIC_ALTITUDE_FUSIONED>::type altitude_fusioned;
    TypeMap<TOPIC_ALTITUDE_BAROMETER>::type altitude_barometer;
    TypeMap<TOPIC_HEIGHT_HOMEPOINT>::type height_homepoint;
    TypeMap<TOPIC_HEIGHT_FUSION>::type height_fusion;
    TypeMap<TOPIC_GPS_POSITION>::type gpsPostion;
    TypeMap<TOPIC_GPS_VELOCITY>::type gpsVelocity;
    TypeMap<TOPIC_BATTERY_INFO>::type batteryInfo;
    TypeMap<TOPIC_FLIGHT_ANOMALY>::type FlightAnomaly;
    TypeMap<TOPIC_GPS_SIGNAL_LEVEL>::type gpsSignalLevel;
    TypeMap<TOPIC_POSITION_VO>::type gpsPositionVo;
    TypeMap<TOPIC_GPS_DATE>::type gpsDate;
    TypeMap<TOPIC_GPS_TIME>::type gpsTime;
    TypeMap<TOPIC_GPS_DETAILS>::type gpsDetail;
    Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();

    velocity                    = vehicle->subscribe->getValue<TOPIC_VELOCITY>();
    rc_with_flag_data           = vehicle->subscribe->getValue<TOPIC_RC_WITH_FLAG_DATA>();
    rtk_connect_status          = vehicle->subscribe->getValue<TOPIC_RTK_CONNECT_STATUS>();
    position_vo                 = vehicle->subscribe->getValue<TOPIC_POSITION_VO>();
    altitude_fusioned           = vehicle->subscribe->getValue<TOPIC_ALTITUDE_FUSIONED>();
    altitude_barometer          = vehicle->subscribe->getValue<TOPIC_ALTITUDE_BAROMETER>();
    height_homepoint            = vehicle->subscribe->getValue<TOPIC_HEIGHT_HOMEPOINT>();
    height_fusion               = vehicle->subscribe->getValue<TOPIC_HEIGHT_FUSION>();
    gpsPostion                  = vehicle->subscribe->getValue<TOPIC_GPS_POSITION>();
    gpsVelocity                 = vehicle->subscribe->getValue<TOPIC_GPS_VELOCITY>();
    batteryInfo                 = vehicle->subscribe->getValue<TOPIC_BATTERY_INFO>();
    FlightAnomaly               = vehicle->subscribe->getValue<TOPIC_FLIGHT_ANOMALY>();
    gpsSignalLevel              = vehicle->subscribe->getValue<TOPIC_GPS_SIGNAL_LEVEL>();
    gpsPositionVo               = vehicle->subscribe->getValue<TOPIC_POSITION_VO>();
    gpsDate                     = vehicle->subscribe->getValue<TOPIC_GPS_DATE>();
    gpsTime                     = vehicle->subscribe->getValue<TOPIC_GPS_TIME>();
    gpsDetail                   = vehicle->subscribe->getValue<TOPIC_GPS_DETAILS>();
    Telemetry::GlobalPosition currentBroadcastGP;
    currentBroadcastGP          = vehicle->broadcast->getGlobalPosition();
    subscriptionQ               = vehicle->subscribe->getValue<TOPIC_QUATERNION>();


    sio::message::ptr gonderilecek_paket(sio::object_message::create());
    std::map<std::string, sio::message::ptr>& gonderilecek_data = gonderilecek_paket->get_map();

    sio::message::ptr object_velocity(sio::object_message::create());
    sio::message::ptr object_rc_with_flag_data(sio::object_message::create());
    sio::message::ptr object_rtk_connect_status(sio::object_message::create());
    sio::message::ptr object_position_vo(sio::object_message::create());
    sio::message::ptr object_altitude_fusioned(sio::object_message::create());
    sio::message::ptr object_altitude_barometer(sio::object_message::create());
    sio::message::ptr object_height_homepoint(sio::object_message::create());
    sio::message::ptr object_height_fusion(sio::object_message::create());
    sio::message::ptr object_gpsPostion(sio::object_message::create());
    sio::message::ptr object_gpsVelocity(sio::object_message::create());
    sio::message::ptr object_batteryInfo(sio::object_message::create());
    sio::message::ptr object_FlightAnomaly(sio::object_message::create());
    sio::message::ptr object_gpsSignalLevel(sio::object_message::create());
    sio::message::ptr object_gpsPositionVo(sio::object_message::create());
    sio::message::ptr object_gpsDate(sio::object_message::create());
    sio::message::ptr object_gpsTime(sio::object_message::create());
    sio::message::ptr object_gpsDetail(sio::object_message::create());
    sio::message::ptr object_currentBroadcastGP(sio::object_message::create());
    sio::message::ptr object_currentSubscriptionGPS(sio::object_message::create());


    std::map<std::string, sio::message::ptr>& velocity_data = object_velocity->get_map();
    std::map<std::string, sio::message::ptr>& rc_with_flag_data_data = object_rc_with_flag_data->get_map();
    std::map<std::string, sio::message::ptr>& rtk_connect_status_data = object_rtk_connect_status->get_map();
    std::map<std::string, sio::message::ptr>& position_vo_data = object_position_vo->get_map();
    std::map<std::string, sio::message::ptr>& altitude_fusioned_data = object_altitude_fusioned->get_map();
    std::map<std::string, sio::message::ptr>& altitude_barometer_data = object_altitude_barometer->get_map();
    std::map<std::string, sio::message::ptr>& height_homepoint_data = object_height_homepoint->get_map();
    std::map<std::string, sio::message::ptr>& height_fusion_data = object_height_fusion->get_map();
    std::map<std::string, sio::message::ptr>& gpsPostion_data = object_gpsPostion->get_map();
    std::map<std::string, sio::message::ptr>& gpsVelocity_data = object_gpsVelocity->get_map();
    std::map<std::string, sio::message::ptr>& batteryInfo_data = object_batteryInfo->get_map();
    std::map<std::string, sio::message::ptr>& FlightAnomaly_data = object_FlightAnomaly->get_map();
    std::map<std::string, sio::message::ptr>& gpsSignalLevel_data = object_gpsSignalLevel->get_map();
    std::map<std::string, sio::message::ptr>& gpsPositionVo_data = object_gpsPositionVo->get_map();
    std::map<std::string, sio::message::ptr>& gpsDate_data = object_gpsDate->get_map();
    std::map<std::string, sio::message::ptr>& gpsTime_data = object_gpsTime->get_map();
    std::map<std::string, sio::message::ptr>& gpsDetail_data = object_gpsDetail->get_map();
    std::map<std::string, sio::message::ptr>& currentBroadcastGP_data = object_currentBroadcastGP->get_map();
    std::map<std::string, sio::message::ptr>& currentSubscriptionGPS_data = object_currentSubscriptionGPS->get_map();

    double yawInRad = toEulerAngle((static_cast<void *>(&subscriptionQ))).z / DEG2RAD;

    velocity_data.insert(std::make_pair("x", sio::double_message::create(velocity.data.x)));
    velocity_data.insert(std::make_pair("y", sio::double_message::create(velocity.data.y)));
    velocity_data.insert(std::make_pair("z", sio::double_message::create(velocity.data.z)));

    rc_with_flag_data_data.insert(std::make_pair("roll", sio::double_message::create(rc_with_flag_data.roll)));
    rc_with_flag_data_data.insert(std::make_pair("pitch", sio::double_message::create(rc_with_flag_data.pitch)));
    rc_with_flag_data_data.insert(std::make_pair("yaw", sio::double_message::create(yawInRad)));
    rc_with_flag_data_data.insert(std::make_pair("throttle", sio::double_message::create(rc_with_flag_data.throttle)));

    position_vo_data.insert(std::make_pair("x", sio::double_message::create(position_vo.x)));
    position_vo_data.insert(std::make_pair("y", sio::double_message::create(position_vo.y)));
    position_vo_data.insert(std::make_pair("z", sio::double_message::create(position_vo.z)));

    altitude_fusioned_data.insert(std::make_pair("altitude_fusioned", sio::double_message::create(altitude_fusioned)));
    altitude_barometer_data.insert(std::make_pair("altitude_barometer", sio::double_message::create(altitude_barometer)));
    height_homepoint_data.insert(std::make_pair("height_homepoint", sio::double_message::create(height_homepoint)));
    height_fusion_data.insert(std::make_pair("height_fusion", sio::double_message::create(height_fusion)));

    gpsPostion_data.insert(std::make_pair("x", sio::double_message::create(gpsPostion.x)));
    gpsPostion_data.insert(std::make_pair("y", sio::double_message::create(gpsPostion.y)));
    gpsPostion_data.insert(std::make_pair("z", sio::double_message::create(gpsPostion.z)));

    gpsVelocity_data.insert(std::make_pair("x", sio::double_message::create(gpsVelocity.x)));
    gpsVelocity_data.insert(std::make_pair("y", sio::double_message::create(gpsVelocity.y)));
    gpsVelocity_data.insert(std::make_pair("z", sio::double_message::create(gpsVelocity.z)));

    rtk_connect_status_data.insert(std::make_pair("rtkConnected", sio::double_message::create(rtk_connect_status.rtkConnected)));

    batteryInfo_data.insert(std::make_pair("capacity", sio::double_message::create(batteryInfo.capacity)));
    batteryInfo_data.insert(std::make_pair("voltage", sio::double_message::create(batteryInfo.voltage)));
    batteryInfo_data.insert(std::make_pair("current", sio::double_message::create(batteryInfo.current)));
    batteryInfo_data.insert(std::make_pair("percentage", sio::double_message::create(batteryInfo.percentage)));

    FlightAnomaly_data.insert(std::make_pair("aircraftIsFalling", sio::double_message::create(FlightAnomaly.aircraftIsFalling)));
    FlightAnomaly_data.insert(std::make_pair("atLeastOneEscDisconnected", sio::double_message::create(FlightAnomaly.atLeastOneEscDisconnected)));
    FlightAnomaly_data.insert(std::make_pair("compassInstallationError", sio::double_message::create(FlightAnomaly.compassInstallationError)));
    FlightAnomaly_data.insert(std::make_pair("escTemperatureHigh", sio::double_message::create(FlightAnomaly.escTemperatureHigh)));
    FlightAnomaly_data.insert(std::make_pair("gpsYawError", sio::double_message::create(FlightAnomaly.gpsYawError)));
    FlightAnomaly_data.insert(std::make_pair("heightCtrlFail", sio::double_message::create(FlightAnomaly.heightCtrlFail)));
    FlightAnomaly_data.insert(std::make_pair("impactInAir", sio::double_message::create(FlightAnomaly.impactInAir)));
    FlightAnomaly_data.insert(std::make_pair("imuInstallationError", sio::double_message::create(FlightAnomaly.imuInstallationError)));
    FlightAnomaly_data.insert(std::make_pair("randomFly", sio::double_message::create(FlightAnomaly.randomFly)));
    FlightAnomaly_data.insert(std::make_pair("reserved", sio::double_message::create(FlightAnomaly.reserved)));
    FlightAnomaly_data.insert(std::make_pair("rollPitchCtrlFail", sio::double_message::create(FlightAnomaly.rollPitchCtrlFail)));
    FlightAnomaly_data.insert(std::make_pair("strongWindLevel1", sio::double_message::create(FlightAnomaly.strongWindLevel1)));
    FlightAnomaly_data.insert(std::make_pair("strongWindLevel2", sio::double_message::create(FlightAnomaly.strongWindLevel2)));
    FlightAnomaly_data.insert(std::make_pair("yawCtrlFail", sio::double_message::create(FlightAnomaly.yawCtrlFail)));

    gpsSignalLevel_data.insert(std::make_pair("gpsSignalLevel", sio::double_message::create(gpsSignalLevel)));

    gpsPositionVo_data.insert(std::make_pair("x", sio::double_message::create(gpsPositionVo.x)));
    gpsPositionVo_data.insert(std::make_pair("y", sio::double_message::create(gpsPositionVo.y)));
    gpsPositionVo_data.insert(std::make_pair("z", sio::double_message::create(gpsPositionVo.z)));
    gpsPositionVo_data.insert(std::make_pair("xHealth", sio::double_message::create(gpsPositionVo.xHealth)));
    gpsPositionVo_data.insert(std::make_pair("yHealth", sio::double_message::create(gpsPositionVo.yHealth)));
    gpsPositionVo_data.insert(std::make_pair("zHealth", sio::double_message::create(gpsPositionVo.zHealth)));


    gpsDate_data.insert(std::make_pair("gpsDate", sio::double_message::create(gpsDate)));
    gpsTime_data.insert(std::make_pair("gpsTime", sio::double_message::create(gpsTime)));

    gpsDetail_data.insert(std::make_pair("fix", sio::double_message::create(gpsDetail.fix)));
    gpsDetail_data.insert(std::make_pair("gnssStatus", sio::double_message::create(gpsDetail.gnssStatus)));
    gpsDetail_data.insert(std::make_pair("GPScounter", sio::double_message::create(gpsDetail.GPScounter)));
    gpsDetail_data.insert(std::make_pair("hacc", sio::double_message::create(gpsDetail.hacc)));
    gpsDetail_data.insert(std::make_pair("hdop", sio::double_message::create(gpsDetail.hdop)));
    gpsDetail_data.insert(std::make_pair("NSV", sio::double_message::create(gpsDetail.NSV)));
    gpsDetail_data.insert(std::make_pair("pdop", sio::double_message::create(gpsDetail.pdop)));
    gpsDetail_data.insert(std::make_pair("sacc", sio::double_message::create(gpsDetail.sacc)));
    gpsDetail_data.insert(std::make_pair("usedGLN", sio::double_message::create(gpsDetail.usedGLN)));
    gpsDetail_data.insert(std::make_pair("usedGPS", sio::double_message::create(gpsDetail.usedGPS)));

    currentBroadcastGP_data.insert(std::make_pair("latitude", sio::double_message::create(currentBroadcastGP.latitude)));
    currentBroadcastGP_data.insert(std::make_pair("longitude", sio::double_message::create(currentBroadcastGP.longitude)));
    currentBroadcastGP_data.insert(std::make_pair("altitude", sio::double_message::create(currentBroadcastGP.altitude)));
    currentBroadcastGP_data.insert(std::make_pair("height", sio::double_message::create(currentBroadcastGP.height)));
    currentBroadcastGP_data.insert(std::make_pair("health", sio::double_message::create(currentBroadcastGP.health)));

    currentSubscriptionGPS_data.insert(std::make_pair("altitude", sio::double_message::create(currentSubscriptionGPS.altitude)));
    currentSubscriptionGPS_data.insert(std::make_pair("latitude", sio::double_message::create(currentSubscriptionGPS.latitude)));
    currentSubscriptionGPS_data.insert(std::make_pair("longitude", sio::double_message::create(currentSubscriptionGPS.longitude)));

    gonderilecek_data.insert(std::make_pair("drone_id", sio::int_message::create(drone_id)));
    gonderilecek_data.insert(std::make_pair("velocity", object_velocity));
    gonderilecek_data.insert(std::make_pair("rc_with_flag_data", object_rc_with_flag_data));
    gonderilecek_data.insert(std::make_pair("rtk_connect_status", object_rtk_connect_status));
    gonderilecek_data.insert(std::make_pair("position_vo", object_position_vo));
    gonderilecek_data.insert(std::make_pair("altitude_fusioned", object_altitude_fusioned));
    gonderilecek_data.insert(std::make_pair("altitude_barometer", object_altitude_barometer));
    gonderilecek_data.insert(std::make_pair("height_homepoint", object_height_homepoint));
    gonderilecek_data.insert(std::make_pair("height_fusion", object_height_fusion));
    gonderilecek_data.insert(std::make_pair("gpsPostion", object_gpsPostion));
    gonderilecek_data.insert(std::make_pair("gpsVelocity", object_gpsVelocity));
    gonderilecek_data.insert(std::make_pair("batteryInfo", object_batteryInfo));
    gonderilecek_data.insert(std::make_pair("FlightAnomaly", object_FlightAnomaly));
    gonderilecek_data.insert(std::make_pair("gpsSignalLevel", object_gpsSignalLevel));
    gonderilecek_data.insert(std::make_pair("gpsPositionVo", object_gpsPositionVo));
    gonderilecek_data.insert(std::make_pair("gpsDate", object_gpsDate));
    gonderilecek_data.insert(std::make_pair("gpsTime", object_gpsTime));
    gonderilecek_data.insert(std::make_pair("gpsDetail", object_gpsDetail));
    gonderilecek_data.insert(std::make_pair("currentBroadcastGP", object_currentBroadcastGP));
    gonderilecek_data.insert(std::make_pair("currentSubscriptionGPS", object_currentSubscriptionGPS));





    return gonderilecek_paket;
}
sio::message::ptr getDataSample() {
    sio::message::ptr gonderilecek_paket(sio::object_message::create());
    std::map<std::string, sio::message::ptr>& gonderilecek_data = gonderilecek_paket->get_map();

    sio::message::ptr object_velocity(sio::object_message::create());
    sio::message::ptr object_rc_with_flag_data(sio::object_message::create());
    sio::message::ptr object_rtk_connect_status(sio::object_message::create());
    sio::message::ptr object_position_vo(sio::object_message::create());
    sio::message::ptr object_altitude_fusioned(sio::object_message::create());
    sio::message::ptr object_altitude_barometer(sio::object_message::create());
    sio::message::ptr object_height_homepoint(sio::object_message::create());
    sio::message::ptr object_height_fusion(sio::object_message::create());
    sio::message::ptr object_gpsPostion(sio::object_message::create());
    sio::message::ptr object_gpsVelocity(sio::object_message::create());
    sio::message::ptr object_batteryInfo(sio::object_message::create());
    sio::message::ptr object_FlightAnomaly(sio::object_message::create());
    sio::message::ptr object_gpsSignalLevel(sio::object_message::create());
    sio::message::ptr object_gpsPositionVo(sio::object_message::create());
    sio::message::ptr object_gpsDate(sio::object_message::create());
    sio::message::ptr object_gpsTime(sio::object_message::create());
    sio::message::ptr object_gpsDetail(sio::object_message::create());
    sio::message::ptr object_currentBroadcastGP(sio::object_message::create());
    sio::message::ptr object_currentSubscriptionGPS(sio::object_message::create());


    std::map<std::string, sio::message::ptr>& velocity_data = object_velocity->get_map();
    std::map<std::string, sio::message::ptr>& rc_with_flag_data_data = object_rc_with_flag_data->get_map();
    std::map<std::string, sio::message::ptr>& rtk_connect_status_data = object_rtk_connect_status->get_map();
    std::map<std::string, sio::message::ptr>& position_vo_data = object_position_vo->get_map();
    std::map<std::string, sio::message::ptr>& altitude_fusioned_data = object_altitude_fusioned->get_map();
    std::map<std::string, sio::message::ptr>& altitude_barometer_data = object_altitude_barometer->get_map();
    std::map<std::string, sio::message::ptr>& height_homepoint_data = object_height_homepoint->get_map();
    std::map<std::string, sio::message::ptr>& height_fusion_data = object_height_fusion->get_map();
    std::map<std::string, sio::message::ptr>& gpsPostion_data = object_gpsPostion->get_map();
    std::map<std::string, sio::message::ptr>& gpsVelocity_data = object_gpsVelocity->get_map();
    std::map<std::string, sio::message::ptr>& batteryInfo_data = object_batteryInfo->get_map();
    std::map<std::string, sio::message::ptr>& FlightAnomaly_data = object_FlightAnomaly->get_map();
    std::map<std::string, sio::message::ptr>& gpsSignalLevel_data = object_gpsSignalLevel->get_map();
    std::map<std::string, sio::message::ptr>& gpsPositionVo_data = object_gpsPositionVo->get_map();
    std::map<std::string, sio::message::ptr>& gpsDate_data = object_gpsDate->get_map();
    std::map<std::string, sio::message::ptr>& gpsTime_data = object_gpsTime->get_map();
    std::map<std::string, sio::message::ptr>& gpsDetail_data = object_gpsDetail->get_map();
    std::map<std::string, sio::message::ptr>& currentBroadcastGP_data = object_currentBroadcastGP->get_map();
    std::map<std::string, sio::message::ptr>& currentSubscriptionGPS_data = object_currentSubscriptionGPS->get_map();



    velocity_data.insert(std::make_pair("x", sio::double_message::create(0)));
    velocity_data.insert(std::make_pair("y", sio::double_message::create(0)));
    velocity_data.insert(std::make_pair("z", sio::double_message::create(0)));

    rc_with_flag_data_data.insert(std::make_pair("roll", sio::double_message::create(0)));
    rc_with_flag_data_data.insert(std::make_pair("pitch", sio::double_message::create(0)));
    rc_with_flag_data_data.insert(std::make_pair("yaw", sio::double_message::create(0)));
    rc_with_flag_data_data.insert(std::make_pair("throttle", sio::double_message::create(0)));

    position_vo_data.insert(std::make_pair("x", sio::double_message::create(0)));
    position_vo_data.insert(std::make_pair("y", sio::double_message::create(0)));
    position_vo_data.insert(std::make_pair("z", sio::double_message::create(0)));

    altitude_fusioned_data.insert(std::make_pair("altitude_fusioned", sio::double_message::create(0)));
    altitude_barometer_data.insert(std::make_pair("altitude_barometer", sio::double_message::create(0)));
    height_homepoint_data.insert(std::make_pair("height_homepoint", sio::double_message::create(0)));
    height_fusion_data.insert(std::make_pair("height_fusion", sio::double_message::create(0)));

    gpsPostion_data.insert(std::make_pair("x", sio::double_message::create(0)));
    gpsPostion_data.insert(std::make_pair("y", sio::double_message::create(0)));
    gpsPostion_data.insert(std::make_pair("z", sio::double_message::create(0)));

    gpsVelocity_data.insert(std::make_pair("x", sio::double_message::create(0)));
    gpsVelocity_data.insert(std::make_pair("y", sio::double_message::create(0)));
    gpsVelocity_data.insert(std::make_pair("z", sio::double_message::create(0)));

    rtk_connect_status_data.insert(std::make_pair("rtkConnected", sio::double_message::create(0)));

    batteryInfo_data.insert(std::make_pair("capacity", sio::double_message::create(0)));
    batteryInfo_data.insert(std::make_pair("voltage", sio::double_message::create(0)));
    batteryInfo_data.insert(std::make_pair("current", sio::double_message::create(0)));
    batteryInfo_data.insert(std::make_pair("percentage", sio::double_message::create(0)));

    FlightAnomaly_data.insert(std::make_pair("aircraftIsFalling", sio::double_message::create(0)));
    FlightAnomaly_data.insert(std::make_pair("atLeastOneEscDisconnected", sio::double_message::create(0)));
    FlightAnomaly_data.insert(std::make_pair("compassInstallationError", sio::double_message::create(0)));
    FlightAnomaly_data.insert(std::make_pair("escTemperatureHigh", sio::double_message::create(0)));
    FlightAnomaly_data.insert(std::make_pair("gpsYawError", sio::double_message::create(0)));
    FlightAnomaly_data.insert(std::make_pair("heightCtrlFail", sio::double_message::create(0)));
    FlightAnomaly_data.insert(std::make_pair("impactInAir", sio::double_message::create(0)));
    FlightAnomaly_data.insert(std::make_pair("imuInstallationError", sio::double_message::create(0)));
    FlightAnomaly_data.insert(std::make_pair("randomFly", sio::double_message::create(0)));
    FlightAnomaly_data.insert(std::make_pair("reserved", sio::double_message::create(0)));
    FlightAnomaly_data.insert(std::make_pair("rollPitchCtrlFail", sio::double_message::create(0)));
    FlightAnomaly_data.insert(std::make_pair("strongWindLevel1", sio::double_message::create(0)));
    FlightAnomaly_data.insert(std::make_pair("strongWindLevel2", sio::double_message::create(0)));
    FlightAnomaly_data.insert(std::make_pair("yawCtrlFail", sio::double_message::create(0)));

    gpsSignalLevel_data.insert(std::make_pair("gpsSignalLevel", sio::double_message::create(0)));

    gpsPositionVo_data.insert(std::make_pair("x", sio::double_message::create(0)));
    gpsPositionVo_data.insert(std::make_pair("y", sio::double_message::create(0)));
    gpsPositionVo_data.insert(std::make_pair("z", sio::double_message::create(0)));
    gpsPositionVo_data.insert(std::make_pair("xHealth", sio::double_message::create(0)));
    gpsPositionVo_data.insert(std::make_pair("yHealth", sio::double_message::create(0)));
    gpsPositionVo_data.insert(std::make_pair("zHealth", sio::double_message::create(0)));


    gpsDate_data.insert(std::make_pair("gpsDate", sio::double_message::create(0)));
    gpsTime_data.insert(std::make_pair("gpsTime", sio::double_message::create(0)));

    gpsDetail_data.insert(std::make_pair("fix", sio::double_message::create(0)));
    gpsDetail_data.insert(std::make_pair("gnssStatus", sio::double_message::create(0)));
    gpsDetail_data.insert(std::make_pair("GPScounter", sio::double_message::create(0)));
    gpsDetail_data.insert(std::make_pair("hacc", sio::double_message::create(0)));
    gpsDetail_data.insert(std::make_pair("hdop", sio::double_message::create(0)));
    gpsDetail_data.insert(std::make_pair("NSV", sio::double_message::create(0)));
    gpsDetail_data.insert(std::make_pair("pdop", sio::double_message::create(0)));
    gpsDetail_data.insert(std::make_pair("sacc", sio::double_message::create(0)));
    gpsDetail_data.insert(std::make_pair("usedGLN", sio::double_message::create(0)));
    gpsDetail_data.insert(std::make_pair("usedGPS", sio::double_message::create(0)));

    currentBroadcastGP_data.insert(std::make_pair("latitude", sio::double_message::create(0)));
    currentBroadcastGP_data.insert(std::make_pair("longitude", sio::double_message::create(0)));
    currentBroadcastGP_data.insert(std::make_pair("altitude", sio::double_message::create(0)));
    currentBroadcastGP_data.insert(std::make_pair("height", sio::double_message::create(0)));
    currentBroadcastGP_data.insert(std::make_pair("health", sio::double_message::create(0)));

    currentSubscriptionGPS_data.insert(std::make_pair("altitude", sio::double_message::create(0)));
    currentSubscriptionGPS_data.insert(std::make_pair("latitude", sio::double_message::create(0)));
    currentSubscriptionGPS_data.insert(std::make_pair("longitude", sio::double_message::create(0)));


    gonderilecek_data.insert(std::make_pair("velocity", object_velocity));
    gonderilecek_data.insert(std::make_pair("rc_with_flag_data", object_rc_with_flag_data));
    gonderilecek_data.insert(std::make_pair("rtk_connect_status", object_rtk_connect_status));
    gonderilecek_data.insert(std::make_pair("position_vo", object_position_vo));
    gonderilecek_data.insert(std::make_pair("altitude_fusioned", object_altitude_fusioned));
    gonderilecek_data.insert(std::make_pair("altitude_barometer", object_altitude_barometer));
    gonderilecek_data.insert(std::make_pair("height_homepoint", object_height_homepoint));
    gonderilecek_data.insert(std::make_pair("height_fusion", object_height_fusion));
    gonderilecek_data.insert(std::make_pair("gpsPostion", object_gpsPostion));
    gonderilecek_data.insert(std::make_pair("gpsVelocity", object_gpsVelocity));
    gonderilecek_data.insert(std::make_pair("batteryInfo", object_batteryInfo));
    gonderilecek_data.insert(std::make_pair("FlightAnomaly", object_FlightAnomaly));
    gonderilecek_data.insert(std::make_pair("gpsSignalLevel", object_gpsSignalLevel));
    gonderilecek_data.insert(std::make_pair("gpsPositionVo", object_gpsPositionVo));
    gonderilecek_data.insert(std::make_pair("gpsDate", object_gpsDate));
    gonderilecek_data.insert(std::make_pair("gpsTime", object_gpsTime));
    gonderilecek_data.insert(std::make_pair("gpsDetail", object_gpsDetail));
    gonderilecek_data.insert(std::make_pair("currentBroadcastGP", object_currentBroadcastGP));
    gonderilecek_data.insert(std::make_pair("currentSubscriptionGPS", object_currentSubscriptionGPS));



    return gonderilecek_paket;
}
void GpsSifirla(Vehicle *vehicle, Telemetry::Vector3f &deltaNed, void *target, void *origin) {
    Telemetry::GPSFused *subscriptionTarget;
    Telemetry::GPSFused *subscriptionOrigin;
    Telemetry::GlobalPosition *broadcastTarget;
    Telemetry::GlobalPosition *broadcastOrigin;
    double deltaLon;
    double deltaLat;

    if (!vehicle->isM100() && !vehicle->isLegacyM600()) {
        subscriptionTarget = (Telemetry::GPSFused *) target;
        subscriptionOrigin = (Telemetry::GPSFused *) origin;
        deltaLon = subscriptionTarget->longitude - subscriptionOrigin->longitude;
        deltaLat = subscriptionTarget->latitude - subscriptionOrigin->latitude;
        deltaNed.x = deltaLat * C_EARTH;
        deltaNed.y = deltaLon * C_EARTH * cos(subscriptionTarget->latitude);
        deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;
    } else {
        broadcastTarget = (Telemetry::GlobalPosition *) target;
        broadcastOrigin = (Telemetry::GlobalPosition *) origin;
        deltaLon = broadcastTarget->longitude - broadcastOrigin->longitude;
        deltaLat = broadcastTarget->latitude - broadcastOrigin->latitude;
        deltaNed.x = deltaLat * C_EARTH;
        deltaNed.y = deltaLon * C_EARTH * cos(broadcastTarget->latitude);
        deltaNed.z = broadcastTarget->altitude - broadcastOrigin->altitude;
    }
};
void mesajgonder(string baslik,int drone_id,std::vector<string> alanlar,std::vector<string> mesajlar){
    sio::message::ptr gonderilecek_paket(sio::object_message::create());
    std::map<std::string, sio::message::ptr>& gonderilecek_data = gonderilecek_paket->get_map();
    gonderilecek_data.insert(std::make_pair("drone_id", sio::int_message::create(drone_id)));
    for (int i = 0; i < alanlar.size(); i++) {
        string alan=alanlar[i];
        string mesaj=mesajlar[i];
        gonderilecek_data.insert(std::make_pair(alan, sio::string_message::create(mesaj)));
    }
    current_socket->emit(baslik, gonderilecek_paket);
}
void hataver(std::string datas,int drone_id) {
    mesajgonder("error",drone_id,{std::string("mesaj")},{std::string(datas)});
};
void islemver(std::string islem,int drone_id) {
    mesajgonder("islem",drone_id,{std::string("mesaj")},{std::string(islem)});
};
void setWaypointDefaults(WayPointSettings *wp) {
    wp->damping = 0;
    wp->yaw = 0;
    wp->gimbalPitch = 0;
    wp->turnMode = 0;
    wp->hasAction = 0;
    wp->actionTimeLimit = 100;
    wp->actionNumber = 0;
    wp->actionRepeat = 0;
    for (int i = 0; i < 16; ++i) {
        wp->commandList[i] = 0;
        wp->commandParameter[i] = 0;
    }
}
void setWaypointInitDefaults(WayPointInitSettings *fdata) {
    fdata->maxVelocity = 10;
    fdata->idleVelocity = 5;
    fdata->finishAction = 0;
    fdata->executiveTimes = 1;
    fdata->yawMode = 0;
    fdata->traceMode = 0;
    fdata->RCLostAction = 1;
    fdata->gimbalPitch = 0;
    fdata->latitude = 0;
    fdata->longitude = 0;
    fdata->altitude = 0;
}
void uploadWaypoints(Vehicle *vehicle, std::vector <DJI::OSDK::WayPointSettings> &wp_list, int responseTimeout) {
    for (std::vector<WayPointSettings>::iterator wp = wp_list.begin(); wp != wp_list.end(); ++wp) {
        //printf("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude, wp->longitude, wp->altitude);
        ACK::WayPointIndex wpDataACK = vehicle->missionManager->wpMission->uploadIndexData(&(*wp), responseTimeout);
        //ACK::getErrorCodeMessage(wpDataACK.ack, __func__);
    }
}
void goreviizle(Vehicle *vehicle, RecvContainer recvFrame, UserData userData) {
    vector<int> sub_id = *(vector<int>*)userData;
    int wayindex = recvFrame.recvData.wayPointReachedData.waypoint_index;
    int waystatus=recvFrame.recvData.wayPointReachedData.current_status;
    mesajgonder("gorev_ucus_check",sub_id[0],{std::string("wayindex"),std::string("waystatus"),std::string("sub_id")},{to_string(wayindex),to_string(waystatus),to_string(sub_id[1])});
}
bool GlobalLokasyonBaslat(Vehicle *vehicle) {
    uint8_t freq[16];

    /* Channels definition for A3/N3/M600
     * 0 - Timestamp
     * 1 - Attitude Quaternions
     * 2 - Acceleration
     * 3 - Velocity (Ground Frame)
     * 4 - Angular Velocity (Body Frame)
     * 5 - Position
     * 6 - GPS Detailed Information
     * 7 - RTK Detailed Information
     * 8 - Magnetometer
     * 9 - RC Channels Data
     * 10 - Gimbal Data
     * 11 - Flight Status
     * 12 - Battery Level
     * 13 - Control Information
     */
    freq[0] = DataBroadcast::FREQ_HOLD;
    freq[1] = DataBroadcast::FREQ_HOLD;
    freq[2] = DataBroadcast::FREQ_HOLD;
    freq[3] = DataBroadcast::FREQ_HOLD;
    freq[4] = DataBroadcast::FREQ_HOLD;
    freq[5] = DataBroadcast::FREQ_50HZ;
    freq[6] = DataBroadcast::FREQ_HOLD;
    freq[7] = DataBroadcast::FREQ_HOLD;
    freq[8] = DataBroadcast::FREQ_HOLD;
    freq[9] = DataBroadcast::FREQ_HOLD;
    freq[10] = DataBroadcast::FREQ_HOLD;
    freq[11] = DataBroadcast::FREQ_HOLD;
    freq[12] = DataBroadcast::FREQ_HOLD;
    freq[13] = DataBroadcast::FREQ_HOLD;

    ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreq(freq, 1);
    if (ACK::getError(ack)) {
        //ACK::getErrorCodeMessage(ack, __func__);
        return false;
    } else {
        return true;
    }
}
bool motor_control(Vehicle *vehicle) {
    string dronversion=vehicle->getHwSerialNum();
    int drone_id=findInVectorStringFromDroneId(araclar_serial,dronversion);
    int motorsNotStarted = 0;
    int timeoutCycles = 20;
    while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() != VehicleStatus::FlightStatus::ON_GROUND &&
           vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_ENGINE_START &&
           motorsNotStarted < timeoutCycles) {
        motorsNotStarted++;
        usleep(100000);
    }
    if (motorsNotStarted == timeoutCycles) {
        LOG_RED("Motor Kontrolü->Başarısız");
        hataver("ER0006",drone_id);
        return false;
    }
    LOG_GREEN("Motor Kontrolü->Başarılı");
    return true;
};
bool ucus_control(Vehicle *vehicle) {
    string dronversion=vehicle->getHwSerialNum();
    int drone_id=findInVectorStringFromDroneId(araclar_serial,dronversion);
    int stillOnGround = 0;
    int timeoutCycles = 110;
    while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() != VehicleStatus::FlightStatus::IN_AIR &&
           (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) && stillOnGround < timeoutCycles) {
        stillOnGround++;
        usleep(100000);
        LOG_MAGENTA("Uçuş Kontrolü Deneme Sayısı->" + to_string(stillOnGround));
    }
    if (stillOnGround == timeoutCycles) {
        LOG_RED("Uçuş Kontrolü->Başarısız");
        hataver("ER0007",drone_id);
        return false;
    }
    LOG_GREEN("Uçuş Kontrolü->Başarılı");
    return true;
}
bool durum_control(Vehicle *vehicle) {
    string dronversion=vehicle->getHwSerialNum();
    int drone_id=findInVectorStringFromDroneId(araclar_serial,dronversion);
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
           VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
           vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() == VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) {
        sleep(1);
    }
    LOG_GREEN("Durum Kontrolü->Başarılı");
    return true;
}
bool takeoff_control(Vehicle *vehicle) {
    string dronversion=vehicle->getHwSerialNum();
    int drone_id=findInVectorStringFromDroneId(araclar_serial,dronversion);
    bool GPS = vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() == VehicleStatus::DisplayMode::MODE_P_GPS;
    bool ATTITUDE =
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() == VehicleStatus::DisplayMode::MODE_ATTITUDE;
    bool RET = GPS || ATTITUDE;
    if (!GPS) {
        LOG_RED("Kalkış Kontrolü [GPS]->Başarısız");
        hataver("ER0009",drone_id);
    };
    if (!ATTITUDE) {
        LOG_RED("Kalkış Kontrolü [YÜKSEKLİK]->Başarısız");
        hataver("ER0010",drone_id);
    };
    if (!RET) {
        hataver("ER0008",drone_id);
        return false;
    };
    return true;
}
bool arm(Vehicle *vehicle, int timeout,string dronversion) {
    int drone_id=findInVectorStringFromDroneId(araclar_serial,dronversion);
    ACK::ErrorCode subscribeStatus = vehicle->control->armMotors(timeout);
    bool durum = ACK::getError(subscribeStatus);
    if (durum != ACK::SUCCESS) {
        LOG_RED("Motor Çalıştırma->Başarısız");
    } else {
        LOG_GREEN("Motor Çalıştırma->Başarılı");
    }
    islemver("IS0003",drone_id);
    LOG_GREEN("Motor Çalıştırma->Başarılı");
    return true;

}
bool disarm(Vehicle *vehicle, int timeout,string droneversion) {
    int drone_id=findInVectorStringFromDroneId(araclar_serial,droneversion);
    ACK::ErrorCode subscribeStatus = vehicle->control->disArmMotors(timeout);
    bool durum = ACK::getError(subscribeStatus);
    if (durum != ACK::SUCCESS) {
        LOG_RED("Motor Durdurma->Başarısız");
    } else {
        LOG_GREEN("Motor Durdurma->Başarılı");
    }
    islemver("IS0004",drone_id);
    LOG_GREEN("Motor Durdurma->Başarılı");
    return true;

}
bool kalkis(Vehicle *vehicle, int timeout,string serial) {
    string dronversion=vehicle->getHwSerialNum();
    int drone_id=findInVectorStringFromDroneId(araclar_serial,serial);
    if (dronversion==serial){
        LOG_YELLOW("Kontrollü Kalkış->Başlatıldı");
        ACK::ErrorCode subscribeStatus = vehicle->control->takeoff(timeout);
        bool durum = ACK::getError(subscribeStatus);
        if (durum != ACK::SUCCESS) {
            LOG_RED("Kontrollü Kalkış->Başarısız");
            mesajgonder("gorev_check",drone_id,{std::string("gorev_tipi"),std::string("durum")},{to_string(0),to_string(false)});
        } else {
            LOG_GREEN("Kontrollü Kalkış->Başarılı");
        }
        if (!(motor_control(vehicle) && ucus_control(vehicle) && durum_control(vehicle) && takeoff_control(vehicle))) {
            mesajgonder("gorev_check",drone_id,{std::string("gorev_tipi"),std::string("durum")},{to_string(0),to_string(false)});
            return false;
        }
        islemver("IS0001",drone_id);
        LOG_GREEN("Kontrollü Kalkış->Başarılı");
        mesajgonder("gorev_check",drone_id,{std::string("gorev_tipi"),std::string("durum")},{to_string(0),to_string(true)});
        return true;
    };
}
bool landed_control(Vehicle *vehicle) {
    string dronversion=vehicle->getHwSerialNum();
    int drone_id=findInVectorStringFromDroneId(araclar_serial,dronversion);
    int stillOnGround=0;
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() == VehicleStatus::DisplayMode::MODE_AUTO_LANDING ||
           vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() == VehicleStatus::FlightStatus::IN_AIR) {
        sleep(1);
        LOG_MAGENTA("İniş Yapıldımı Kontrolü Deneme Sayısı->" + to_string(stillOnGround));
        stillOnGround=stillOnGround+1;
    }
    LOG_GREEN("Kontrollü İniş->Başarılı");
    return true;
}
bool landing_control(Vehicle *vehicle) {
    string dronversion=vehicle->getHwSerialNum();
    int drone_id=findInVectorStringFromDroneId(araclar_serial,dronversion);
    int landingNotStarted = 0;
    int timeoutCycles = 20;
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           landingNotStarted < timeoutCycles) {
        landingNotStarted++;
        usleep(100000);
        LOG_MAGENTA("İniş Kontrolü Deneme Sayısı->" + to_string(landingNotStarted));
    }
    if (landingNotStarted == timeoutCycles) {
        LOG_RED("İniş Kontrolü->Başarısız");
        hataver("ER0011",drone_id);
        return false;
    }
    LOG_GREEN("İniş Kontrolü->Başarılı");
    return true;
};
bool inis(Vehicle *vehicle, int timeout,string serial) {
    string dronversion=vehicle->getHwSerialNum();
    int drone_id=findInVectorStringFromDroneId(araclar_serial,serial);

    if (dronversion==serial){
        LOG_YELLOW("Kontrollü İniş->Başlatıldı");
        ACK::ErrorCode subscribeStatus = vehicle->control->land(timeout);
        bool durum = ACK::getError(subscribeStatus);
        if (durum != ACK::SUCCESS) {
            LOG_RED("Kontrollü İniş->Başarısız");
            mesajgonder("gorev_check",drone_id,{std::string("gorev_tipi"),std::string("durum")},{to_string(4),to_string(false)});
        } else {
            LOG_GREEN("Kontrollü İniş->Başarılı");
        }
        if (!(landing_control(vehicle) && landed_control(vehicle))) {
            mesajgonder("gorev_check",drone_id,{std::string("gorev_tipi"),std::string("durum")},{to_string(4),to_string(false)});
            return false;
        }
        islemver("IS0002",drone_id);
        LOG_GREEN("Kontrollü İniş->Başarılı");
        mesajgonder("gorev_check",drone_id,{std::string("gorev_tipi"),std::string("durum")},{to_string(4),to_string(true)});
        return true;
    };
}
bool golocation(DJI::OSDK::Vehicle *vehicle, int responseTimeout, std::vector <std::vector<float>> koordinatlar,string serial,int sub_id) {
    string dronversion=vehicle->getHwSerialNum();
    int drone_id=findInVectorStringFromDroneId(araclar_serial,serial);
    if (dronversion==serial){
        LOG_YELLOW("Lokasyona Gitme->Başlatıldı");
        WayPointInitSettings fdata;
        setWaypointInitDefaults(&fdata);
        fdata.indexNumber = koordinatlar.size();
        ACK::ErrorCode initAck = vehicle->missionManager->init(DJI_MISSION_TYPE::WAYPOINT, responseTimeout, &fdata);
        if (ACK::getError(initAck)) {
            LOG_RED("Lokasyona Gitme->Başarısız");
            //ACK::getErrorCodeMessage(initAck, __func__);
            return false;
        }
        vehicle->missionManager->printInfo();
        LOG_YELLOW("Görev Hazırlanma->Başlatıldı");
        std::vector <WayPointSettings> generatedWaypts;
        for (int i = 0; i < koordinatlar.size(); i++) {
            WayPointSettings wp;
            setWaypointDefaults(&wp);
            std::vector<float> koordinat = koordinatlar[i];
            wp.index = i;
            wp.latitude = koordinat[0];
            wp.longitude = koordinat[1];
            wp.altitude = koordinat[2];
            generatedWaypts.push_back(wp);
        }
        uploadWaypoints(vehicle, generatedWaypts, responseTimeout);
        LOG_GREEN("Görev Oluşturma->Başarılı");

        vector<int> paket = {drone_id,sub_id};
        vector<int>* ptrsub_id = new vector<int>(paket);


        vehicle->missionManager->wpMission->setWaypointEventCallback(goreviizle, ptrsub_id);
        ACK::ErrorCode startAck = vehicle->missionManager->wpMission->start(responseTimeout);
        if (ACK::getError(startAck)) {
            LOG_RED("Görev Başlatma->Başarısız");
            mesajgonder("gorev_check",drone_id,{std::string("gorev_tipi"),std::string("durum")},{to_string(3),to_string(false)});
            //ACK::getErrorCodeMessage(initAck, __func__);
        } else {
            mesajgonder("gorev_check",drone_id,{std::string("gorev_tipi"),std::string("durum")},{to_string(3),to_string(true)});
            LOG_GREEN("Görev Başlatılma->Başarılı");
        }
    };
}
bool goposition(Vehicle *vehicle, float xOffsetDesired, float yOffsetDesired, float zOffsetDesired, float yawDesired,float posThresholdInM, float yawThresholdInDeg,string serial) {
    string dronversion=vehicle->getHwSerialNum();
    int drone_id=findInVectorStringFromDroneId(araclar_serial,serial);
    if (dronversion==serial){
        LOG_YELLOW("Pozisyona Gitme->Başlatıldı");
        int responseTimeout = 1;
        int timeoutInMilSec = 40000;
        int controlFreqInHz = 50;
        int cycleTimeInMs = 1000;
        int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;
        int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs;

        GlobalLokasyonBaslat(vehicle);
        sleep(1);

        Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
        Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
        Telemetry::GlobalPosition currentBroadcastGP;
        Telemetry::GlobalPosition originBroadcastGP;
        Telemetry::Vector3f localOffset;
        currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
        originSubscriptionGPS = currentSubscriptionGPS;
        GpsSifirla(vehicle, localOffset, static_cast<void *>(&currentSubscriptionGPS),static_cast<void *>(&originSubscriptionGPS));
        currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
        double xOffsetRemaining = xOffsetDesired - localOffset.x;
        double yOffsetRemaining = yOffsetDesired - localOffset.y;
        double zOffsetRemaining = zOffsetDesired - localOffset.z;
        double yawDesiredRad = DEG2RAD * yawDesired;
        double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;
        Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
        Telemetry::Quaternion broadcastQ;
        double yawInRad;
        subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
        yawInRad = toEulerAngle((static_cast<void *>(&subscriptionQ))).z / DEG2RAD;
        int elapsedTimeInMs = 0;
        int withinBoundsCounter = 0;
        int outOfBounds = 0;
        int brakeCounter = 0;
        int speedFactor = 2;
        float xCmd, yCmd, zCmd;
        if (xOffsetDesired > 0)
            xCmd = (xOffsetDesired < speedFactor) ? xOffsetDesired : speedFactor;
        else if (xOffsetDesired < 0)
            xCmd =
                    (xOffsetDesired > -1 * speedFactor) ? xOffsetDesired : -1 * speedFactor;
        else
            xCmd = 0;
        if (yOffsetDesired > 0)
            yCmd = (yOffsetDesired < speedFactor) ? yOffsetDesired : speedFactor;
        else if (yOffsetDesired < 0)
            yCmd =
                    (yOffsetDesired > -1 * speedFactor) ? yOffsetDesired : -1 * speedFactor;
        else
            yCmd = 0;

        zCmd = currentBroadcastGP.height + zOffsetDesired;
        while (elapsedTimeInMs < timeoutInMilSec) {
            vehicle->control->positionAndYawCtrl(xCmd, yCmd, zCmd, yawDesiredRad / DEG2RAD);
            usleep(cycleTimeInMs * 1000);
            elapsedTimeInMs += cycleTimeInMs;
            subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
            yawInRad = toEulerAngle((static_cast<void *>(&subscriptionQ))).z;
            currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
            GpsSifirla(vehicle, localOffset, static_cast<void *>(&currentSubscriptionGPS),static_cast<void *>(&originSubscriptionGPS));
            currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
            xOffsetRemaining = xOffsetDesired - localOffset.x;
            yOffsetRemaining = yOffsetDesired - localOffset.y;
            zOffsetRemaining = zOffsetDesired - localOffset.z;
            if (std::abs(xOffsetRemaining) < speedFactor) { xCmd = xOffsetRemaining; }
            if (std::abs(yOffsetRemaining) < speedFactor) { yCmd = yOffsetRemaining; }
            if (std::abs(xOffsetRemaining) < posThresholdInM && std::abs(yOffsetRemaining) < posThresholdInM &&
                std::abs(zOffsetRemaining) < posThresholdInM &&
                std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad) { withinBoundsCounter += cycleTimeInMs; }
            else { if (withinBoundsCounter != 0) { outOfBounds += cycleTimeInMs; }}
            if (outOfBounds > outOfControlBoundsTimeLimit) {
                withinBoundsCounter = 0;
                outOfBounds = 0;
            }
            if (withinBoundsCounter >= withinControlBoundsTimeReqmt) { break; }
        }
        while (brakeCounter < withinControlBoundsTimeReqmt) {
            vehicle->control->emergencyBrake();
            usleep(cycleTimeInMs * 10);
            brakeCounter += cycleTimeInMs;
        }
        LOG_GREEN("Pozisyona Gitme->Başarılı");
        mesajgonder("gorev_check",drone_id,{std::string("gorev_tipi"),std::string("durum")},{to_string(2),to_string(true)});
        return ACK::SUCCESS;
    };
}
Vehicle * find_vehicle(int drone_id){
    int indx=findInVector(araclar_id,drone_id);
    if (indx==-1){
        return NULL;
    }else{
        return &araclar_drone[indx];
    };
};
Vehicle * find_serial(string serial){
    int indx=findInVectorString(araclar_serial,serial);
    if (indx==-1){
        return NULL;
    }else{
        return &araclar_drone[indx];
    };
};
sio::socket::event_listener_aux ongetTelemetry(sio::socket::event_listener_aux([](string const &name, message::ptr const &data, bool isAck, message::list &ack_resp) {
    int drone_id=data->get_map()["drone_id"]->get_int();
    Vehicle * vehicle = find_vehicle(drone_id);
    if (vehicle!=NULL){current_socket->emit("telemetryData", getData(vehicle,drone_id));}

}));
sio::socket::event_listener_aux onisonline(sio::socket::event_listener_aux([](string const &name, message::ptr const &data, bool isAck, message::list &ack_resp) {
    int drone_id=data->get_map()["drone_id"]->get_int();
    Vehicle * vehicle = find_vehicle(drone_id);
    if (vehicle!=NULL){
        string dronversion=vehicle->getHwSerialNum();
        mesajgonder("online",drone_id,{std::string("serial")},{std::string(dronversion)});
    }
}));
sio::socket::event_listener_aux ontakeoff(sio::socket::event_listener_aux([](string const &name, message::ptr const &data, bool isAck, message::list &ack_resp) {
    Vehicle *vehicle = find_vehicle(data->get_map()["drone_id"]->get_int());
    if (vehicle!=NULL) {
        string dronversion = vehicle->getHwSerialNum();
        kalkis(vehicle, 2, dronversion);
    }
}));
sio::socket::event_listener_aux onland(sio::socket::event_listener_aux([](string const &name, message::ptr const &data, bool isAck, message::list &ack_resp) {
    Vehicle * vehicle = find_vehicle(data->get_map()["drone_id"]->get_int());
    if (vehicle!=NULL) {
        string dronversion=vehicle->getHwSerialNum();
        inis(vehicle, 2,dronversion);
    }
}));
sio::socket::event_listener_aux onarm(sio::socket::event_listener_aux([](string const &name, message::ptr const &data, bool isAck, message::list &ack_resp) {
    Vehicle * vehicle = find_vehicle(data->get_map()["drone_id"]->get_int());
    if (vehicle!=NULL) {
        string dronversion=vehicle->getHwSerialNum();
        arm(vehicle, 2,dronversion);
    }
}));
sio::socket::event_listener_aux ondisarm(sio::socket::event_listener_aux([](string const &name, message::ptr const &data, bool isAck, message::list &ack_resp) {
    Vehicle * vehicle = find_vehicle(data->get_map()["drone_id"]->get_int());
    if (vehicle!=NULL) {
        string dronversion=vehicle->getHwSerialNum();
        disarm(vehicle, 2,dronversion);
    }
}));
sio::socket::event_listener_aux ongolocation(sio::socket::event_listener_aux([](string const &name, message::ptr const &data, bool isAck, message::list &ack_resp) {
    int drone_id=data->get_map()["drone_id"]->get_int();
    Vehicle * vehicle = find_vehicle(drone_id);
    if (vehicle!=NULL) {
        std::vector <message::ptr> tumveri = data->get_map()["gorevler"]->get_vector();
        std::vector <std::vector<float>> koordinatlar;
        for (int i = 0; i < tumveri.size(); i++) {
            std::vector <message::ptr> veri = tumveri[i].get()->get_vector();
            std::vector<float> wp;
            for (int i = 0; i < veri.size(); i++) {
                float deger = veri[i].get()->get_double();
                wp.push_back(deger);
            }
            koordinatlar.push_back(wp);
        }
        string dronversion=vehicle->getHwSerialNum();
        //golocation(vehicle, 10, koordinatlar,dronversion);
    }

}));
sio::socket::event_listener_aux ongo(sio::socket::event_listener_aux([](string const &name, message::ptr const &data, bool isAck, message::list &ack_resp) {
    Vehicle * vehicle = find_vehicle(data->get_map()["drone_id"]->get_int());
    if (vehicle!=NULL) {
        float x = data->get_map()["x"]->get_double();
        float y = data->get_map()["y"]->get_double();
        float z = data->get_map()["z"]->get_double();
        float yaw = data->get_map()["yaw"]->get_double();
        string dronversion=vehicle->getHwSerialNum();
        goposition(vehicle, x, y, z, yaw, 0.5, 1.0,dronversion);
    }

}));
sio::socket::event_listener_aux onDeneme(sio::socket::event_listener_aux([](string const &name, message::ptr const &data, bool isAck, message::list &ack_resp) {
    Vehicle * vehicle = find_vehicle(data->get_map()["drone_id"]->get_int());
    if (vehicle!=NULL) {
        current_socket->emit("deneme", getDataSample());
    }

}));
void gorevioynat(std::vector <message::ptr> tumveri,string serial,Vehicle* vehicle){
    for (int i = 0; i < tumveri.size(); i++) {
        std::map<std::string, sio::message::ptr>&       gorev_data  = tumveri[i]->get_map();
        string                                          komut       = gorev_data["komut"]->get_string();
        int                                             tip         = gorev_data["tip"]->get_int();
        int                                             bekleme     = gorev_data["bekleme"]->get_int();
        int                                             sub_id      = gorev_data["sub_id"]->get_int();
        int                                             drone_id    = findInVectorStringFromDroneId(araclar_serial,serial);
        std::vector <message::ptr>                      pointler    = gorev_data["data"]->get_vector();
        if (tip==0){
            //LOG_GREEN("kalkis("+serial+")");
            kalkis(vehicle, 2, serial);
        }
        if (tip==1){
            //LOG_GREEN("bekle("+serial+","+to_string(bekleme)+")");
            sleep(bekleme);
            mesajgonder("gorev_check",drone_id,{std::string("gorev_tipi"),std::string("durum")},{to_string(1),to_string(true)});
        }
        if (tip==2){
            for (int j = 0; j < pointler.size(); j++) {
                std::vector <message::ptr> altveri = pointler[j].get()->get_vector();
                for (int u = 0; u < altveri.size(); u++) {
                    std::vector<float> wp;
                    std::vector <message::ptr> lokasyonveri = altveri[u].get()->get_vector();
                    for (int n = 0; n < lokasyonveri.size(); n++) {
                        float deger = lokasyonveri[n].get()->get_double();
                        wp.push_back(deger);
                    }
                    //LOG_GREEN("goposition("+to_string(wp[0])+","+to_string(wp[1])+","+to_string(wp[2])+","+to_string(wp[3])+")");
                    goposition(vehicle, wp[0], wp[1], wp[2], wp[3], 0.5, 1.0,serial);
                }
            }
        }
        if (tip==3){
            for (int j = 0; j < pointler.size(); j++) {
                std::vector <std::vector<float>> koordinatlar;
                std::vector <message::ptr> altveri = pointler[j].get()->get_vector();
                for (int u = 0; u < altveri.size(); u++) {
                    std::vector<float> wp;
                    std::vector <message::ptr> lokasyonveri = altveri[u].get()->get_vector();
                    for (int n = 0; n < lokasyonveri.size(); n++) {
                        float deger = lokasyonveri[n].get()->get_double();
                        wp.push_back(deger);
                    }
                    koordinatlar.push_back(wp);
                }
                //LOG_GREEN("golocation("+serial+")");
                golocation(vehicle, 10, koordinatlar, serial,sub_id);
            }
        }
        if (tip==4){
            //LOG_GREEN("inis("+serial+")");
            inis(vehicle, 2, serial);
        }
    };
}
void async_gorevi_oynat(std::vector <message::ptr> tumveri,string serial,Vehicle* vehicle){
    std::thread t(gorevioynat,tumveri,serial,vehicle);
    t.detach();
}
sio::socket::event_listener_aux onStartMission(sio::socket::event_listener_aux([](string const &name, message::ptr const &data, bool isAck, message::list &ack_resp) {
    std::vector <message::ptr> droneveri = data->get_map()["droneveri"]->get_vector();
    for (int k = 0; k < droneveri.size(); k++) {
        std::map<std::string, sio::message::ptr>& dronedata = droneveri[k]->get_map();
        std::vector <message::ptr> tumveri = dronedata["gorevler"]->get_vector();
        string serial = dronedata["serial"]->get_string();
        int drone_id = dronedata["drone_id"]->get_int();
        Vehicle * vehicle = find_vehicle(drone_id);
        if (vehicle!=NULL){
            async_gorevi_oynat(tumveri,serial,vehicle);
        }
    }
}));
Vehicle * init_drone(int app_id,string app_key,string device,int baudrate){
    LinuxSetup linuxEnvironment(app_id,app_key,device,baudrate);
    Vehicle * vehicle = linuxEnvironment.getVehicle();
    if (vehicle == NULL) {
        LOG_RED("Dron Başlatma->Başarısız");
        return NULL;
    }
    vehicle->obtainCtrlAuthority(functionTimeout);
    vers = vehicle->getHwSerialNum();
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(30);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
        LOG_RED("Dron Doğrulama->Başarısız");
        //ACK::getErrorCodeMessage(subscribeStatus, __func__);
        return NULL;
    }

    int numTopic = sizeof(topicListAll) / sizeof(topicListAll[0]);
    bool enableTimestamp = false;
    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopic, topicListAll, enableTimestamp,10);
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, 5);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
        LOG_RED("Tüm Telemetri Oluşturma->Başarısız");
        //ACK::getErrorCodeMessage(subscribeStatus, __func__);
        vehicle->subscribe->removePackage(pkgIndex, 5);
        return NULL;
    }


    /*
    int numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
    bool enableTimestamp = false;
    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopic, topicList50Hz, enableTimestamp,freq);
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, 20);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
        LOG_RED("50 Hz. Telemetri Oluşturma->Başarısız");
        //ACK::getErrorCodeMessage(subscribeStatus, __func__);
        vehicle->subscribe->removePackage(pkgIndex, 20);
        return NULL;
    }
    pkgIndex = 1;
    freq = 1;
    numTopic = sizeof(topicList1Hz) / sizeof(topicList1Hz[0]);
    enableTimestamp = false;
    pkgStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopic, topicList1Hz, enableTimestamp, freq);
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, 20);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
        LOG_RED("1 Hz. Telemetri Oluşturma->Başarısız");
        //ACK::getErrorCodeMessage(subscribeStatus, __func__);
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    }
    pkgIndex = 2;
    freq = 10;
    numTopic = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    enableTimestamp = false;
    pkgStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, 20);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
        LOG_RED("10 Hz. Telemetri Oluşturma->Başarısız");
        //ACK::getErrorCodeMessage(subscribeStatus, __func__);
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    }
    pkgIndex = 3;
    freq = 50;
    numTopic = sizeof(topicListGo) / sizeof(topicListGo[0]);
    enableTimestamp = false;
    pkgStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopic, topicListGo, enableTimestamp, freq);
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, 20);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
        LOG_RED("Telemetri Oluşturma->Başarısız");
        //ACK::getErrorCodeMessage(subscribeStatus, __func__);
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    }
     */
    LOG_GREEN("Drone Oluşturma->Başarılı");
    return vehicle;
}



sio::socket::event_listener_aux onInitDevice(sio::socket::event_listener_aux([](string const &name, message::ptr const &data, bool isAck, message::list &ack_resp) {
    int drone_id            = data->get_map()["drone_id"]->get_int();
    int app_id              = data->get_map()["app_id"]->get_int();
    string serial_number    = data->get_map()["serial"]->get_string();
    string app_key          = data->get_map()["app_key"]->get_string();
    string device           = data->get_map()["device"]->get_string();
    int baudrate            = data->get_map()["baudrate"]->get_int();

    Vehicle * v;
    v=find_vehicle(drone_id);
    string status="0";
    if (v==NULL){
        v=init_drone(app_id,app_key,device,baudrate);
        if (v!=NULL){
            araclar_drone.push_back(v);
            araclar_id.push_back(drone_id);
            araclar_serial.push_back(serial_number);
            string dronversion=v->getHwSerialNum();
            status="1";
        }
    }else{
        status = "1";
    }
    mesajgonder("initdrone",drone_id,{std::string("serial"),std::string("status")},{std::string(serial_number),std::string(status)});
}));
int init_socket(){
    h.connect("http://127.0.0.1:5338");
    w.connect("http://127.0.0.1:5339");
    current_socket = h.socket();
    current_socket->on("gettelemetry", ongetTelemetry);
    current_socket->on("isonline", onisonline);
    current_socket->on("takeoff", ontakeoff);
    current_socket->on("land", onland);
    current_socket->on("arm", onarm);
    current_socket->on("disarm", ondisarm);
    current_socket->on("golocation", ongolocation);
    current_socket->on("go", ongo);
    current_socket->on("deneme", onDeneme);
    current_socket->on("startmission", onStartMission);
    current_socket->on("initdrone", onInitDevice);
};
int main(int argc, char **argv) {
    init_socket();
    while (1) {
        for (int i = 0; i < araclar_drone.size(); i++) {
            sio::message::ptr gonderilecek_paket(sio::object_message::create());
            std::map<std::string, sio::message::ptr>& gonderilecek_data = gonderilecek_paket->get_map();
            gonderilecek_data.insert(std::make_pair("drone_id", sio::int_message::create(araclar_id[i])));
            gonderilecek_data.insert(std::make_pair("data", getData(&araclar_drone[i],araclar_id[i])));
            w.socket()->emit("telemetryData", gonderilecek_paket);
        }
        usleep(10000);
    }
    return 1;
}
