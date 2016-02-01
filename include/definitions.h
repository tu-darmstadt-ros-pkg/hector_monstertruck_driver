#ifndef MONSTERTRUCK_INTERFACEDEFINTIONS_H
#define MONSTERTRUCK_INTERFACEDEFINTIONS_H

#include <stdint.h>

// individual structure alignment (linux- gcc)
#ifdef __GNUC__
        #ifndef GCC3X_PACK8
                #define GCC3X_PACK8 __attribute__((aligned(1),packed)) // tell gcc to do structure alignment on byte boundary
        #endif
        #ifndef UNUSED
                #define UNUSED __attribute__((unused)) // tell gcc that this variable might not be used
        #endif
#else
        #ifndef GCC3X_PACK8
                #define GCC3X_PACK8
        #endif
        #pragma pack(push)
        #ifndef UNUSED
                #define UNUSED
        #endif
#endif

  //! Xenomai priority of the interface task
  #define NAV_INTERFACE_TASK_PRIORITY     99

  //! Start address of D-RAM
  #define NAV_INTERFACE_BASIS             0xc8000
  #define NAV_INTERFACE_DRAM_SIZE         1024

  //! concerning the communication with the interface-board
  const char NAV_INTERFACE_COM_SYNC[] = { 0xB5, 0x62 };
  #define NAV_INTERFACE_VEHICLE_CLASS     0x10      //!< Class for UGV1
  #define NAV_INTERFACE_COM_ID_STATUS     0x00      //!< ID for Status msg
  #define NAV_INTERFACE_COM_ID_IMU        0x01      //!< ID for IMU msg
  #define NAV_INTERFACE_COM_ID_GPS_IN     0x02      //!< ID for GPS_In msg
  #define NAV_INTERFACE_COM_ID_COMPASS_2D 0x03      //!< ID for 2D-compass msg
  #define NAV_INTERFACE_COM_ID_HODOMETER  0x04      //!< ID for Hodometer msg
  #define NAV_INTERFACE_COM_ID_PRELOAD    0x11      //!< ID for Preload msg
  #define NAV_INTERFACE_COM_ID_GPS_OUT    0x12      //!< ID for GPS_Out msg
  #define NAV_INTERFACE_COM_ID_SERVO_OUT  0x13      //!< ID for Servo_out msg
  #define NAV_INTERFACE_COM_ID_BAUDRATE   0x14      //!< ID for Baudrate msg
  #define NAV_INTERFACE_COM_ID_PC_OUT     0x15      //!< ID for PC_Out msg
  #define NAV_INTERFACE_COM_ID_ERROR_IN   0x20      //!< ID for Error_in msg

  //! Interface preload for 50Hz is 64881
  #define NAV_INTERFACE_COM_PRELOAD_50HZ_LOW    113
  #define NAV_INTERFACE_COM_PRELOAD_50HZ_HIGH   253

  #define NAV_API_COMPASS2D_OFFSET_Y 			1000
  #define NAV_API_COMPASS2D_OFFSET_X 			1800

  // Set Baudrate to 9600
  // NAV_INTERFACE_VEHICLE_CLASS, NAV_INTERFACE_COM_ID_BAUDRATE,

  static unsigned char NAV_INTERFACE_COMMAND_BAUDRATE_4800[] UNUSED = {
    0xc0,0x12,0x00,0x00};

  static unsigned char NAV_INTERFACE_COMMAND_BAUDRATE_9600[] UNUSED = {
    0x80,0x25,0x00,0x00};

  static unsigned char NAV_INTERFACE_COMMAND_BAUDRATE_19200[] UNUSED = {
    0x00,0x4b,0x00,0x00};

  static unsigned char NAV_INTERFACE_COMMAND_BAUDRATE_38400[] UNUSED = {
    0x00,0x96,0x00,0x00};

  static unsigned char NAV_INTERFACE_COMMAND_BAUDRATE_57600[] UNUSED = {
    0x00,0xE1,0x00,0x00};


  static unsigned char NAV_INTERFACE_COMMAND_START_50HZ[] UNUSED = {
    NAV_INTERFACE_COM_PRELOAD_50HZ_LOW, NAV_INTERFACE_COM_PRELOAD_50HZ_HIGH};

  static unsigned char NAV_INTERFACE_COMMAND_STOP[] UNUSED = {
    0x00,0x00};


  /*
  static unsigned char NAV_INTERFACE_COMMAND_START_50HZ[] UNUSED = {
    NAV_INTERFACE_COM_SYNC[0], NAV_INTERFACE_COM_SYNC[1],
    NAV_INTERFACE_VEHICLE_CLASS, NAV_INTERFACE_COM_ID_PRELOAD,0x02,0x00,
    NAV_INTERFACE_COM_PRELOAD_50HZ_LOW, NAV_INTERFACE_COM_PRELOAD_50HZ_HIGH,
    0x00,0x00};
  */
  static unsigned char NAV_INTERFACE_COMMAND_START_SLOW[] UNUSED = {
    NAV_INTERFACE_COM_SYNC[0], NAV_INTERFACE_COM_SYNC[1],
    NAV_INTERFACE_VEHICLE_CLASS, NAV_INTERFACE_COM_ID_PRELOAD,0x02,0x00,
    0x00,0xA5,
    0x00,0x00};

  /*
  static unsigned char NAV_INTERFACE_COMMAND_STOP[] UNUSED = {
    NAV_INTERFACE_COM_SYNC[0], NAV_INTERFACE_COM_SYNC[1],
    NAV_INTERFACE_VEHICLE_CLASS, NAV_INTERFACE_COM_ID_PRELOAD,0x02,0x00,
    0x00,0x00,
    0x00,0x00};
  */

  /*
  static unsigned char NAV_INTERFACE_COMMAND_BAUDRATE_0[] UNUSED = {
    NAV_INTERFACE_COM_SYNC[0], NAV_INTERFACE_COM_SYNC[1],
    NAV_INTERFACE_VEHICLE_CLASS, NAV_INTERFACE_COM_ID_BAUDRATE,0x02,0x00,
    0x00,0x00,
    0x00,0x00};
  */
  /*
  static unsigned char NAV_INTERFACE_COMMAND_BAUDRATE_9600[] UNUSED = {
    NAV_INTERFACE_COM_SYNC[0], NAV_INTERFACE_COM_SYNC[1],
    NAV_INTERFACE_VEHICLE_CLASS, NAV_INTERFACE_COM_ID_BAUDRATE,0x02,0x00,MonsterTruckI
    0x80,0x25,
    0x00,0x00};
  */

  static unsigned char NAV_INTERFACE_COMMAND_STATUS[] UNUSED = {
    NAV_INTERFACE_COM_SYNC[0], NAV_INTERFACE_COM_SYNC[1],
    NAV_INTERFACE_VEHICLE_CLASS, NAV_INTERFACE_COM_ID_STATUS,0x04,0x00,
    0x00,0x00,0x00,0x00,
    0x00,0x00};

  /*
  static unsigned char NAV_INTERFACE_COMMAND_TEST[] UNUSED = {
    NAV_INTERFACE_COM_SYNC[0], NAV_INTERFACE_COM_SYNC[1],
    NAV_INTERFACE_VEHICLE_CLASS, NAV_INTERFACE_COM_ID_PRELOAD,0x02,0x00,
  //  NAV_INTERFACE_COM_PRELOAD_50HZ_LOW, NAV_INTERFACE_COM_PRELOAD_50HZ_HIGH,
    0x00,0xF5,
    0x00,0x00,
    NAV_INTERFACE_COM_SYNC[0], NAV_INTERFACE_COM_SYNC[1],
    NAV_INTERFACE_VEHICLE_CLASS, NAV_INTERFACE_COM_ID_BAUDRATE,0x02,0x00,
    0x80,0x25,
    0x00,0x00};

  static unsigned char NAV_INTERFACE_COMMAND_TESTSTOP[] UNUSED = {
    NAV_INTERFACE_COM_SYNC[0], NAV_INTERFACE_COM_SYNC[1],
    NAV_INTERFACE_VEHICLE_CLASS, NAV_INTERFACE_COM_ID_PRELOAD,0x02,0x00,
    0x00,0x00,
    0x00,0x00,
    NAV_INTERFACE_COM_SYNC[0], NAV_INTERFACE_COM_SYNC[1],
    NAV_INTERFACE_VEHICLE_CLASS, NAV_INTERFACE_COM_ID_BAUDRATE,0x02,0x00,
    0x00,0x00,
    0x00,0x00};
  */


  // Port 1, out UBX, 9600 8N1, autobauding off
  // Port 2, in RTCM/UBX, out UBX, 9600 8N1, autobauding off
  static unsigned char NAV_UBXCFGPRT[] UNUSED = {
    0xB5,0x62,0x06,0x00,0x28,0x00,
    0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,        // Port 1
    0x80,0x25,0x00,0x00,0x05,0x00,0x01,0x00,
    0x00,0x00,0x00,0x00,
    0x02,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,        // Port 2
    0x80,0x25,0x00,0x00,0x05,0x00,0x01,0x00,
    0x00,0x00,0x00,0x00,
    0x37,0x9E};

  // Port 1, out UBX, 9600 8N1, autobauding off
  static unsigned char NAV_UBXCFGPRT_PRT_9600[] UNUSED = {
    0xB5,0x62,0x06,0x00,0x14,0x00,
    0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,        // Port 1
    0x80,0x25,0x00,0x00,0x05,0x00,0x01,0x00,
    0x00,0x00,0x00,0x00,
    0x9E,0x99};

  // Port 1, out UBX, 57600 8N1, autobauding off
  static unsigned char NAV_UBXCFGPRT_PRT_57600[] UNUSED = {
    0xB5,0x62,0x06,0x00,0x14,0x00,
    0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,        // Port 1
    0x00,0xE1,0x00,0x00,0x05,0x00,0x01,0x00,
    0x00,0x00,0x00,0x00,
    0x9E,0x99};

  static unsigned char NAV_UBXCFGPRT_PRT0[] UNUSED = {
    0xB5,0x62,0x06,0x00,0x14,0x00,
    0x00,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,        // Port 0
    0x80,0x25,0x00,0x00,0x05,0x00,0x01,0x00,
    0x00,0x00,0x00,0x00,
    0x9D,0x85};

  static unsigned char NAV_UBXCFGPRT_PRT2[] UNUSED = {
    0xB5,0x62,0x06,0x00,0x14,0x00,
    0x02,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,        // Port 2
    0x80,0x25,0x00,0x00,0x05,0x00,0x01,0x00,
    0x00,0x00,0x00,0x00,
    0x9F,0xAD};

  // Nav-Solution Port 1&2
  static unsigned char NAV_UBXCFGMSGNAVSOL[] UNUSED = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x01,0x06,0x00,0x01,0x01,0x00,
    0x16,0x9E};

  // No SatInfo
  static unsigned char NAV_UBXCFGMSGNAVSVINFO0[] UNUSED = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x01,0x30,0x00,0x00,0x00,0x00,
    0x3E,0x6B};

  // SatInfo on Port 1, every measurement
  static unsigned char NAV_UBXCFGMSGNAVSVINFO1[] UNUSED = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x01,0x30,0x00,0x01,0x00,0x00,
    0x3F,0x6E};

  // SatInfo on Port 1, every 4th measurement
  static unsigned char NAV_UBXCFGMSGNAVSVINFO4[] UNUSED = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x01,0x30,0x00,0x04,0x00,0x00,
    0x42,0x77};

  // SBASInfo on Port 1
  static unsigned char NAV_UBXCFGMSGNAVSBAS[] UNUSED = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x01,0x32,0x00,0x02,0x00,0x00,
    0x42,0x7B};

  // DGPSInfo on Port 1
  static unsigned char NAV_UBXCFGMSGNAVDGPS[] UNUSED = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x01,0x31,0x00,0x02,0x00,0x00,
    0x41,0x76};

  // Fast Acquisition, Continuous Tracking
  static unsigned char NAV_UBXCFGRXM[] UNUSED = {
    0xB5,0x62,0x06,0x11,0x02,0x00,0x02,0x00,
    0x1B,0x85};

  // SBAS enabled, (Rng,Cor,Int), max. 3, autoscan
  static unsigned char NAV_UBXCFGSBASAUTOSCAN[] UNUSED = {
    0xB5,0x62,0x06,0x16,0x08,0x00,
    0x01,0x07,0x03,0x00,0x00,0x00,0x00,0x00,
    0x2F,0xD5};

  // SBAS enabled, (Rng,Cor,Int), max. 3, 120, 124, 126, 131
  static unsigned char NAV_UBXCFGSBASEGNOS[] UNUSED = {
    0xB5,0x62,0x06,0x16,0x08,0x00,
    0x01,0x07,0x03,0x00,0x51,0x08,0x00,0x00,
    0x88,0x31};

  // Measurement rate 250ms, Navigation rate 4 Hz, UTC align
  static unsigned char NAV_UBXCFGRATE4HZ[] UNUSED = {
    0xB5,0x62,0x06,0x08,0x06,0x00,
    0xFA,0x00,0x01,0x00,0x00,0x00,
    0x0F,0x94};

  /*
  // Timepuls every 250ms, 50ms long
  // rising edge, align GPS time
  // 50ns antenna, 820ns RF, 0ns group,
  static unsigned char NAV_UBXCFGTP4HZ[] UNUSED = {
    0xB5,0x62,0x06,0x07,0x14,0x00,
    0x90,0xD0,0x03,0x00,0x50,0xC3,
    0x00,0x00,0x01,0x01,0x00,0x00,
    0x32,0x00,0x34,0x03,0x00,0x00,
    0x00,0x00,
    0x02,0x2A};
  */

  //! @brief Structure holding 2Dcompass data
  typedef struct GCC3X_PACK8 NAV_API_RAW_COMPASS_2D_DATA
  {
    int16_t    comp_x;                         //!< x component of field [?]
    int16_t    comp_y;                         //!< y component of field [?]
  } NAV_API_RAW_COMPASS_2D_DATA;


  //! @brief Structure holding interface status data
  typedef struct GCC3X_PACK8 NAV_API_RAW_STATUS_DATA
  {
    uint32_t    Systemticks;                  //!< ticks [1/32768 s]
    uint8_t     CommandsProcessed;            //!< Number of sucessfully processed packets
    uint8_t     ErrorId;                      //!< Id: error Id; 0: no error
    uint8_t     ErrorValue;                   //!< Value to add information to the ErrorId
    uint8_t     GpsSendQueueLoad;             //!< Bytes in the GPS send queue
    uint8_t     AutonomousModeFlag;           //!< 1: autonoumous; <>1: not autonomous
    uint16_t    InputVoltage1;                //!< Input voltage Bat1 [mV]
    uint16_t    InputVoltage2;                //!< Input voltage Bat2 [mV]
  } NAV_API_RAW_STATUS_DATA;


  //! @brief Structure holding raw IMU-data
  typedef struct GCC3X_PACK8 NAV_API_RAW_IMU_DATA
  {
    uint16_t   RGyroX;                        //!< A/D discrete: turn rate X gyro
    uint16_t   RGyroY;                        //!< A/D discrete: turn rate Y gyro
    uint16_t   RGyroZ;                        //!< A/D discrete: turn rate Z gyro
    uint16_t   RAccelX;                       //!< A/D discrete: acceleration X acceleration
    uint16_t   RAccelY;                       //!< A/D discrete: acceleration Y acceleration
    uint16_t   RAccelZ;                       //!< A/D discrete: acceleration Z acceleration
    uint16_t   RGyroXRef;                     //!< A/D discrete: X gyro reference voltage
    uint16_t   RGyroYRef;                     //!< A/D discrete: Y gyro reference voltage
    uint16_t   RGyroZRef;                     //!< A/D discrete: Z gyro reference voltage
    uint16_t   RGyroXTmp;                     //!< A/D discrete: X gyro temperature output voltage
    uint16_t   RGyroYTmp;                     //!< A/D discrete: Y gyro temperature output voltage
    uint16_t   RGyroZTmp;                     //!< A/D discrete: Z gyro temperature output voltage
    uint16_t   RBaro;                         //!< A/D discrete: static barometric pressure
    uint16_t   RAux1;                         //!< unused
    uint16_t   RAux2;                         //!< unused
    uint16_t   RAux3;                         //!< unused
  } NAV_API_RAW_IMU_DATA;

  //! @brief Structure holding raw servo-data
  //!
  typedef struct GCC3X_PACK8 NAV_API_RAW_SERVO_DATA
  {
      static const uint32_t   CHANNELS = 7;
      uint16_t   Servo[CHANNELS];              //!< interface range [0<=1000<=2000]
                                               //!< input range [-32768 to 32767] -> interface range [0<=1000<=2000]
  } NAV_API_RAW_SERVO_DATA;


  //! @brief Structure holding hodometer data
  //!
  typedef struct GCC3X_PACK8 NAV_API_RAW_HODOMETER_DATA
  {
      uint32_t   hodometer_count[4];           //!< 4 hodometer counts
  } NAV_API_RAW_HODOMETER_DATA;


  #define UBLOX_NAVSOL_CLASS      0x01
  #define UBLOX_NAVSOL_ID         0x06
  #define UBLOX_NAV_SVINFO_CLASS  0x01
  #define UBLOX_NAV_SVINFO_ID     0x30
  #define UBLOX_ACK_CLASS         0x05
  #define UBLOX_NACK_ID           0x00
  #define UBLOX_ACK_ID            0x01


  //! space vehicle data
  typedef struct GCC3X_PACK8 NAV_UBLOX_SVData
  {
      uint8_t  chn;                             //!< channel number, range 0..NCH-1
      uint8_t  SVID;                            //!< Satellite ID
      uint8_t  Flags;                           //!< bitmask see 'SvFlags'
      int8_t   QI;                              //!< constants see 'SvQualityInicator'
      uint8_t  CNO;                             //!< [dbHz ] Carrier to Noise Ratio (Signal Strength)
      int8_t   Elev;                            //!< [deg] Elevation in integer degrees
      int16_t  Azim;                            //!< [deg] Azimuth in integer degrees
      int32_t  PRRres;                          //!< [cm]  Pseudo range residual in centimetres
  } NAV_UBLOX_SVData;

  //! ubx satellite frame NAV-SVINFO (0x01 0x30)
  typedef struct GCC3X_PACK8 NAV_UBLOX_UBXSvInfo
  {
    uint32_t   ITOW;                          //! [ms]  GPS Millisecond time of week
    uint8_t    NCH;                           //! Number of channels range 0..16
    uint8_t    RES1;                          //! Reserved
    uint8_t    RES2;                          //! Reserved
    NAV_UBLOX_SVData sv[16];
  } NAV_UBLOX_UBXSvInfo;

  //! ubx gps frame NAV-SOL (0x01 0x06 )
  // Navigation Solution Information  This message combining Position, velocity and time solution in ECEF
  typedef struct GCC3X_PACK8 NAV_UBLOX_UBXGpsNavSol
  {
    uint32_t     ITOW ;                         //! [ms] GPS Millisecond Time of Week
    int32_t       Frac;                          //! [ns] Nanoseconds remainder of rounded ms above, range -500000 .. 500000
    int16_t     week;                          //! GPS week (GPS time)
    uint8_t    GPSfix;                        //! details see 'GpsFix' constants
    uint8_t    Flags;                         //! details see 'GpsFlags' constants
    int32_t       ECEF_X;                        //! [cm]  ECEF X coordinate
    int32_t       ECEF_Y;                        //! [cm]  ECEF Y coordinate
    int32_t       ECEF_Z;                        //! [cm]  ECEF Z coordinate
    uint32_t     Pacc;                          //! [cm]  3D Position Accuracy Estimate
    uint32_t       ECEFVX;                        //! [cm/s]  ECEF X velocity
    int32_t       ECEFVY;                        //! [cm/s]  ECEF Y velocity
    int32_t       ECEFVZ;                        //! [cm/s]  ECEF Z velocity
    uint32_t     SAcc;                          //! [cm/s] Speed Accuracy Estimate
    uint16_t   PDOP;                          //! PDOP  -  Position DOP
    uint8_t    res1;                          //! Reserved
    uint8_t    numSV;                         //! Number of SVs used in Nav Solution
    uint32_t     res2;                          //! Reserved
  } NAV_UBLOX_UBXGpsNavSol;

#ifndef __GNUC__
#pragma pack(pop)
#endif

#endif // MONSTERTRUCK_INTERFACEDEFINTIONS_H
