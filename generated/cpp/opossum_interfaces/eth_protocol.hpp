// =========================================================================
// GENERE AUTOMATIQUEMENT par opossum_interfaces/gen.py depuis protocol.yaml
// NE PAS MODIFIER A LA MAIN : editer protocol.yaml puis lancer `python3 gen.py`.
// =========================================================================
#pragma once

#include <cstddef>
#include <cstdint>

namespace opossum_interfaces {
namespace eth {

constexpr uint16_t FRAME_MAGIC      = 0xC0DE;
constexpr uint8_t  PROTOCOL_VERSION = 1;
constexpr uint32_t PROTOCOL_HASH    = 0xEAE76C2Fu;
constexpr uint16_t MAX_PAYLOAD      = 512;
constexpr uint8_t  MSG_RAW_TEXT     = 0xFF;
constexpr size_t   FRAME_CRC_OFFSET = 12;

// Direction vue du Zynq : TX = le Zynq emet (le PC ecoute), RX = le Zynq ecoute
enum class ChannelDir : uint8_t { TX = 0, RX = 1 };

enum class Channel : uint8_t {
    DEBUG = 0,
    TELEMETRY = 1,
    CMD = 2,
    RAW_CMD = 3,
};
constexpr size_t CHANNEL_COUNT = 4;

struct ChannelInfo { const char *name; uint16_t port; ChannelDir dir; bool framed; };
constexpr ChannelInfo CHANNELS[CHANNEL_COUNT] = {
    {"DEBUG", 5000, ChannelDir::TX, true},
    {"TELEMETRY", 5001, ChannelDir::TX, true},
    {"CMD", 5002, ChannelDir::RX, true},
    {"RAW_CMD", 5003, ChannelDir::RX, false},
};

enum MsgId : uint8_t {
    MSG_HEARTBEAT = 0x01,
    MSG_DEBUG_TEXT = 0x02,
    MSG_ODOM = 0x10,
    MSG_IMU = 0x11,
    MSG_MOTOR_STATE = 0x12,
    MSG_ROBOT_STATE = 0x13,
    MSG_AU = 0x14,
    MSG_LEASH = 0x15,
    MSG_LIDAR_SCAN_CHUNK = 0x16,
    MSG_CALIB_SAMPLE = 0x17,
    MSG_CMD_GENERIC = 0x20,
    MSG_CMD_GOAL_POSITION = 0x21,
    MSG_CMD_SET_LIDAR = 0x22,
    MSG_CMD_SET_CAMERA_1 = 0x23,
    MSG_CMD_SET_CAMERA_2 = 0x24,
    MSG_CMD_SET_CAMERA_3 = 0x25,
    MSG_CMD_SET_SPEED_MAX = 0x26,
    MSG_CMD_BLOCK = 0x27,
    MSG_CMD_FREE = 0x28,
};

// kind : 0 = format non defini, 1 = texte, 2 = structure
struct MessageInfo { const char *name; uint8_t id; Channel channel; uint8_t kind; uint16_t size; };
constexpr size_t MESSAGE_COUNT = 19;
constexpr MessageInfo MESSAGES[MESSAGE_COUNT] = {
    {"HEARTBEAT", 0x01, Channel::TELEMETRY, 2, 10},
    {"DEBUG_TEXT", 0x02, Channel::DEBUG, 1, 0},
    {"ODOM", 0x10, Channel::TELEMETRY, 2, 24},
    {"IMU", 0x11, Channel::TELEMETRY, 2, 40},
    {"MOTOR_STATE", 0x12, Channel::TELEMETRY, 0, 0},
    {"ROBOT_STATE", 0x13, Channel::TELEMETRY, 2, 29},
    {"AU", 0x14, Channel::TELEMETRY, 2, 5},
    {"LEASH", 0x15, Channel::TELEMETRY, 2, 5},
    {"LIDAR_SCAN_CHUNK", 0x16, Channel::TELEMETRY, 2, 415},
    {"CALIB_SAMPLE", 0x17, Channel::TELEMETRY, 2, 82},
    {"CMD_GENERIC", 0x20, Channel::CMD, 1, 0},
    {"CMD_GOAL_POSITION", 0x21, Channel::CMD, 2, 12},
    {"CMD_SET_LIDAR", 0x22, Channel::CMD, 2, 16},
    {"CMD_SET_CAMERA_1", 0x23, Channel::CMD, 2, 28},
    {"CMD_SET_CAMERA_2", 0x24, Channel::CMD, 2, 28},
    {"CMD_SET_CAMERA_3", 0x25, Channel::CMD, 2, 28},
    {"CMD_SET_SPEED_MAX", 0x26, Channel::CMD, 0, 0},
    {"CMD_BLOCK", 0x27, Channel::CMD, 2, 1},
    {"CMD_FREE", 0x28, Channel::CMD, 2, 1},
};

inline const MessageInfo *message_by_id(uint8_t id) {
    for (const auto &m : MESSAGES) { if (m.id == id) return &m; }
    return nullptr;
}

inline const MessageInfo *message_by_name(const char *name) {
    for (const auto &m : MESSAGES) {
        const char *a = m.name, *b = name;
        while (*a && *a == *b) { ++a; ++b; }
        if (*a == *b) return &m;
    }
    return nullptr;
}

inline const ChannelInfo &channel_info(Channel c) { return CHANNELS[static_cast<size_t>(c)]; }

#pragma pack(push, 1)
struct FrameHeader {
    uint16_t magic;         // ETH_FRAME_MAGIC, permet de resynchroniser le flux
    uint8_t version;        // ETH_PROTOCOL_VERSION
    uint8_t msg_type;       // eth_msg_type_t
    uint16_t seq;           // compteur roulant, un par canal
    uint32_t timestamp_us;  // [us] horodatage Zynq, cf eth_get_timestamp_us()
    uint16_t payload_len;   // longueur du payload qui suit
    uint16_t crc16;         // CRC16-CCITT sur (header, crc16 mis a 0) + payload
};
static_assert(sizeof(FrameHeader) == 14, "FrameHeader");

// Point lidar LD19
struct LidarPoint {
    uint16_t distance_mm;  // [mm] 0 = point invalide/filtre
    uint16_t angle_cdeg;   // [cdeg] centiemes de degre, 0..35999
};
static_assert(sizeof(LidarPoint) == 4, "LidarPoint");

// Miroir de Set_camera (common_type.h) : pose mesuree par une camera
struct SetCamera {
    float x;         // [m]
    float y;         // [m]
    float t;         // [rad]
    uint32_t delay;  // [ms] latence de calcul
    float noise_x;   // [m] ecart-type
    float noise_y;   // [m] ecart-type
    float noise_t;   // [rad] ecart-type
};
static_assert(sizeof(SetCamera) == 28, "SetCamera");

// HEARTBEAT (0x01) : Emis a 1 Hz par CPU0. Le PC compare protocol_hash au sien avant de piloter.
struct PayloadHeartbeat {
    uint32_t uptime_ms;      // [ms]
    uint8_t state;           // 0 = boucle principale active
    uint8_t flags;           // bit0 = AU appuye, bit1 = laisse tiree
    uint32_t protocol_hash;  // ETH_PROTOCOL_HASH du firmware
};
static_assert(sizeof(PayloadHeartbeat) == 10, "PayloadHeartbeat");

// ODOM (0x10)
struct PayloadOdom {
    float x;      // [m]
    float y;      // [m]
    float theta;  // [rad]
    float vx;     // [m/s]
    float vy;     // [m/s]
    float omega;  // [rad/s]
};
static_assert(sizeof(PayloadOdom) == 24, "PayloadOdom");

// IMU (0x11)
struct PayloadImu {
    float qw;
    float qx;
    float qy;
    float qz;
    float gyro_x;   // [rad/s]
    float gyro_y;   // [rad/s]
    float gyro_z;   // [rad/s]
    float accel_x;  // [m/s2]
    float accel_y;  // [m/s2]
    float accel_z;  // [m/s2]
};
static_assert(sizeof(PayloadImu) == 40, "PayloadImu");

// ROBOT_STATE (0x13) : Pose et vitesse estimees (Kalman CPU1), emis periodiquement
struct PayloadRobotState {
    uint32_t timestamp_ms;  // [ms]
    float x;                // [m]
    float y;                // [m]
    float theta;            // [rad]
    float speed_linear;     // [m/s]
    float speed_direction;  // [rad]
    float speed_angular;    // [rad/s]
    uint8_t motion_done;
};
static_assert(sizeof(PayloadRobotState) == 29, "PayloadRobotState");

// AU (0x14) : Emis sur chaque changement d'etat. `state` DOIT rester le 1er octet (comm_node lit payload[0]).
struct PayloadAu {
    uint8_t state;          // 0 = AU relache, 1 = AU appuye
    uint32_t timestamp_ms;  // [ms]
};
static_assert(sizeof(PayloadAu) == 5, "PayloadAu");

// LEASH (0x15) : Emis sur chaque changement d'etat. comm_node publie "LEASH" (depart du match) si state == 1.
struct PayloadLeash {
    uint8_t state;          // 0 = laisse relachee, 1 = laisse tiree
    uint32_t timestamp_ms;  // [ms]
};
static_assert(sizeof(PayloadLeash) == 5, "PayloadLeash");

// LIDAR_SCAN_CHUNK (0x16) : Scan LD19 complet decoupe en morceaux ; seuls point_count points sont emis
struct PayloadLidarScanChunk {
    uint8_t lidar_id;
    uint32_t scan_id;
    uint32_t timestamp_ms;   // [ms]
    uint16_t chunk_index;
    uint16_t chunk_count;
    uint16_t point_count;
    LidarPoint points[100];
};
static_assert(sizeof(PayloadLidarScanChunk) == 415, "PayloadLidarScanChunk");

// CALIB_SAMPLE (0x17) : Log de calibration (CALIBLOG 1 [periode_ms]), emis a 100 Hz par defaut. Miroir de Calib_sample (libs/opossum_control/src/control_types.h). Grandeurs integrees : travailler sur les differences.
struct PayloadCalibSample {
    uint32_t tick_ms;      // [ms] pas rapides (1 ms) depuis control_init (CPU1)
    float wheel_dist[4];   // [m] distance integree par roue, rayon nominal, signe odometrie
    float gyro_z;          // [rad/s] derniere mesure gyro brute
    float gyro_angle;      // [rad] integrale du gyro brut, non repliee
    uint32_t gyro_seq;     // compteur de mesures gyro
    float odo_x;           // [m]
    float odo_y;           // [m]
    float odo_t;           // [rad]
    float kal_x;           // [m]
    float kal_y;           // [m]
    float kal_t;           // [rad]
    float lidar_x;         // [m] derniere mesure lidar recue (SETLIDAR)
    float lidar_y;         // [m]
    float lidar_t;         // [rad]
    uint32_t lidar_delay;  // [ms] retard annonce par cette mesure
    uint32_t lidar_rx_ms;  // [ms] tick_ms a sa reception
    uint32_t lidar_count;  // nombre de mesures lidar recues
    uint8_t motion_done;   // 0 arrive, 1 en mouvement, 2 freine
    uint8_t flags;         // bit0 Kalman initialise, bit1 arret d'urgence
};
static_assert(sizeof(PayloadCalibSample) == 82, "PayloadCalibSample");

// CMD_GOAL_POSITION (0x21) : Miroir de Position (common_type.h)
struct PayloadCmdGoalPosition {
    float x;  // [m]
    float y;  // [m]
    float t;  // [rad]
};
static_assert(sizeof(PayloadCmdGoalPosition) == 12, "PayloadCmdGoalPosition");

// CMD_SET_LIDAR (0x22) : Miroir de Set_lidar (common_type.h) : pose mesuree par le lidar
struct PayloadCmdSetLidar {
    float x;         // [m]
    float y;         // [m]
    float t;         // [rad]
    uint32_t delay;  // [ms] latence de calcul
};
static_assert(sizeof(PayloadCmdSetLidar) == 16, "PayloadCmdSetLidar");

// CMD_SET_CAMERA_1 (0x23)
using PayloadCmdSetCamera1 = SetCamera;

// CMD_SET_CAMERA_2 (0x24)
using PayloadCmdSetCamera2 = SetCamera;

// CMD_SET_CAMERA_3 (0x25)
using PayloadCmdSetCamera3 = SetCamera;

// CMD_BLOCK (0x27) : Bloque le robot (asserv_mode = 4)
struct PayloadCmdBlock {
    uint8_t value;  // 1
};
static_assert(sizeof(PayloadCmdBlock) == 1, "PayloadCmdBlock");

// CMD_FREE (0x28) : Libere le robot (asserv_mode = 0)
struct PayloadCmdFree {
    uint8_t value;  // 1
};
static_assert(sizeof(PayloadCmdFree) == 1, "PayloadCmdFree");

#pragma pack(pop)

// CRC16-CCITT (poly 0x1021, init 0xFFFF) sur l'en-tete (crc16 = 0) puis le payload
inline uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF) {
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                  : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

}  // namespace eth
}  // namespace opossum_interfaces
