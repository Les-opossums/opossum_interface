/* =========================================================================
 * GENERE AUTOMATIQUEMENT par opossum_interfaces/gen.py depuis protocol.yaml
 * NE PAS MODIFIER A LA MAIN : editer protocol.yaml puis lancer `python3 gen.py`.
 * ========================================================================= */

#ifndef OPOSSUM_ETH_PROTOCOL_H
#define OPOSSUM_ETH_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* --- IDENTIFIANTS RESEAU --- */
#define ETH_FRAME_MAGIC      0xC0DE
#define ETH_PROTOCOL_VERSION 1
#define ETH_PROTOCOL_HASH    0xEAE76C2Fu /* hash de protocol.yaml, envoye dans HEARTBEAT */

#define ETH_DEBUG_PORT       5000   /* Zynq -> PC, texte libre (printf-like) */
#define ETH_TELEMETRY_PORT   5001   /* Zynq -> PC, trames structurees periodiques */
#define ETH_CMD_PORT         5002   /* PC -> Zynq, commandes */
#define ETH_RAW_CMD_PORT     5003   /* PC -> Zynq, commandes brutes (debug) */

#define ETH_MAX_PAYLOAD      512

/* ------------------------------------------------------------------------
 * Table des canaux UDP. Colonnes : NOM, PORT, DIRECTION, FRAMED
 *   ETH_DIR_TX -> le Zynq emet vers ce port distant
 *   ETH_DIR_RX -> le Zynq ecoute sur ce port
 *   FRAMED (canaux RX) : false -> payload brut, handler appele avec ETH_MSG_RAW_TEXT
 * ------------------------------------------------------------------------ */
typedef enum {
    ETH_DIR_TX = 0,
    ETH_DIR_RX = 1,
} eth_channel_dir_t;

#define ETH_CHANNEL_LIST(X) \
    X(DEBUG,     ETH_DEBUG_PORT,      ETH_DIR_TX, true) \
    X(TELEMETRY, ETH_TELEMETRY_PORT,  ETH_DIR_TX, true) \
    X(CMD,       ETH_CMD_PORT,        ETH_DIR_RX, true) \
    X(RAW_CMD,   ETH_RAW_CMD_PORT,    ETH_DIR_RX, false)

typedef enum {
#define X(name, port, dir, framed) ETH_CHANNEL_##name,
    ETH_CHANNEL_LIST(X)
#undef X
    ETH_CHANNEL_COUNT
} eth_channel_id_t;

/* ------------------------------------------------------------------------
 * Liste des messages UDP. Colonnes : NOM, ID_HEXA, CANAL
 * ------------------------------------------------------------------------ */
#define ETH_MSG_RAW_TEXT 0xFF /* type factice des canaux non framés */

#define ETH_MESSAGE_LIST(X) \
    /* --- MESSAGES TX (Zynq -> PC) --- */ \
    X(HEARTBEAT,         0x01, ETH_CHANNEL_TELEMETRY) \
    X(DEBUG_TEXT,        0x02, ETH_CHANNEL_DEBUG) \
    X(ODOM,              0x10, ETH_CHANNEL_TELEMETRY) \
    X(IMU,               0x11, ETH_CHANNEL_TELEMETRY) \
    X(MOTOR_STATE,       0x12, ETH_CHANNEL_TELEMETRY) \
    X(ROBOT_STATE,       0x13, ETH_CHANNEL_TELEMETRY) \
    X(AU,                0x14, ETH_CHANNEL_TELEMETRY) \
    X(LEASH,             0x15, ETH_CHANNEL_TELEMETRY) \
    X(LIDAR_SCAN_CHUNK,  0x16, ETH_CHANNEL_TELEMETRY) \
    X(CALIB_SAMPLE,      0x17, ETH_CHANNEL_TELEMETRY) \
    /* --- MESSAGES RX (PC -> Zynq) --- */ \
    X(CMD_GENERIC,       0x20, ETH_CHANNEL_CMD) \
    X(CMD_GOAL_POSITION, 0x21, ETH_CHANNEL_CMD) \
    X(CMD_SET_LIDAR,     0x22, ETH_CHANNEL_CMD) \
    X(CMD_SET_CAMERA_1,  0x23, ETH_CHANNEL_CMD) \
    X(CMD_SET_CAMERA_2,  0x24, ETH_CHANNEL_CMD) \
    X(CMD_SET_CAMERA_3,  0x25, ETH_CHANNEL_CMD) \
    X(CMD_SET_SPEED_MAX, 0x26, ETH_CHANNEL_CMD) \
    X(CMD_BLOCK,         0x27, ETH_CHANNEL_CMD) \
    X(CMD_FREE,          0x28, ETH_CHANNEL_CMD)

typedef enum {
#define X(name, id, channel) ETH_MSG_##name = id,
    ETH_MESSAGE_LIST(X)
#undef X
} eth_msg_type_t;

/* ------------------------------------------------------------------------
 * En-tete de trame UDP. CRC16-CCITT (poly 0x1021, init 0xFFFF) sur l'en-tete (crc16 = 0) puis le payload
 * ------------------------------------------------------------------------ */
typedef struct __attribute__((packed)) {
    uint16_t magic;         /* ETH_FRAME_MAGIC, permet de resynchroniser le flux */
    uint8_t version;        /* ETH_PROTOCOL_VERSION */
    uint8_t msg_type;       /* eth_msg_type_t */
    uint16_t seq;           /* compteur roulant, un par canal */
    uint32_t timestamp_us;  /* [us] horodatage Zynq, cf eth_get_timestamp_us() */
    uint16_t payload_len;   /* longueur du payload qui suit */
    uint16_t crc16;         /* CRC16-CCITT sur (header, crc16 mis a 0) + payload */
} eth_frame_header_t; /* 14 octets */

#define ETH_FRAME_CRC_OFFSET 12 /* offset du champ crc16 */

#endif /* OPOSSUM_ETH_PROTOCOL_H */
