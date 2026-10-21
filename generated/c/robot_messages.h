/* =========================================================================
 * GENERE AUTOMATIQUEMENT par opossum_interfaces/gen.py depuis protocol.yaml
 * NE PAS MODIFIER A LA MAIN : editer protocol.yaml puis lancer `python3 gen.py`.
 * ========================================================================= */

#ifndef OPOSSUM_ROBOT_MESSAGES_H
#define OPOSSUM_ROBOT_MESSAGES_H

#include <stdint.h>

/* Structures des payloads, identiques au bit pres cote Zynq et cote PC. */

/* Point lidar LD19 */
typedef struct __attribute__((packed)) {
    uint16_t distance_mm;  /* [mm] 0 = point invalide/filtre */
    uint16_t angle_cdeg;   /* [cdeg] centiemes de degre, 0..35999 */
} eth_lidar_point_t; /* 4 octets */

/* Miroir de Set_camera (common_type.h) : pose mesuree par une camera */
typedef struct __attribute__((packed)) {
    float x;         /* [m] */
    float y;         /* [m] */
    float t;         /* [rad] */
    uint32_t delay;  /* [ms] latence de calcul */
    float noise_x;   /* [m] ecart-type */
    float noise_y;   /* [m] ecart-type */
    float noise_t;   /* [rad] ecart-type */
} eth_set_camera_t; /* 28 octets */

/* HEARTBEAT (0x01) : Emis a 1 Hz par CPU0. Le PC compare protocol_hash au sien avant de piloter. */
typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;      /* [ms] */
    uint8_t state;           /* 0 = boucle principale active */
    uint8_t flags;           /* bit0 = AU appuye, bit1 = laisse tiree */
    uint32_t protocol_hash;  /* ETH_PROTOCOL_HASH du firmware */
} eth_payload_heartbeat_t; /* 10 octets */

/* DEBUG_TEXT (0x02) : texte libre */

/* ODOM (0x10) */
typedef struct __attribute__((packed)) {
    float x;      /* [m] */
    float y;      /* [m] */
    float theta;  /* [rad] */
    float vx;     /* [m/s] */
    float vy;     /* [m/s] */
    float omega;  /* [rad/s] */
} eth_payload_odom_t; /* 24 octets */

/* IMU (0x11) */
typedef struct __attribute__((packed)) {
    float qw;
    float qx;
    float qy;
    float qz;
    float gyro_x;   /* [rad/s] */
    float gyro_y;   /* [rad/s] */
    float gyro_z;   /* [rad/s] */
    float accel_x;  /* [m/s2] */
    float accel_y;  /* [m/s2] */
    float accel_z;  /* [m/s2] */
} eth_payload_imu_t; /* 40 octets */

/* MOTOR_STATE (0x12) : format non defini */

/* ROBOT_STATE (0x13) : Pose et vitesse estimees (Kalman CPU1), emis periodiquement */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;  /* [ms] */
    float x;                /* [m] */
    float y;                /* [m] */
    float theta;            /* [rad] */
    float speed_linear;     /* [m/s] */
    float speed_direction;  /* [rad] */
    float speed_angular;    /* [rad/s] */
    uint8_t motion_done;
} eth_payload_robot_state_t; /* 29 octets */

/* AU (0x14) : Emis sur chaque changement d'etat. `state` DOIT rester le 1er octet (comm_node lit payload[0]). */
typedef struct __attribute__((packed)) {
    uint8_t state;          /* 0 = AU relache, 1 = AU appuye */
    uint32_t timestamp_ms;  /* [ms] */
} eth_payload_au_t; /* 5 octets */

/* LEASH (0x15) : Emis sur chaque changement d'etat. comm_node publie "LEASH" (depart du match) si state == 1. */
typedef struct __attribute__((packed)) {
    uint8_t state;          /* 0 = laisse relachee, 1 = laisse tiree */
    uint32_t timestamp_ms;  /* [ms] */
} eth_payload_leash_t; /* 5 octets */

/* LIDAR_SCAN_CHUNK (0x16) : Scan LD19 complet decoupe en morceaux ; seuls point_count points sont emis */
typedef struct __attribute__((packed)) {
    uint8_t lidar_id;
    uint32_t scan_id;
    uint32_t timestamp_ms;          /* [ms] */
    uint16_t chunk_index;
    uint16_t chunk_count;
    uint16_t point_count;
    eth_lidar_point_t points[100];  /* seuls `point_count` elements sont emis */
} eth_payload_lidar_scan_chunk_t; /* 415 octets */

#define ETH_LIDAR_SCAN_CHUNK_POINTS_MAX 100
#define ETH_LIDAR_SCAN_CHUNK_FIXED_SIZE 15 /* taille sans le tableau `points` */

/* CMD_GENERIC (0x20) : texte libre */

/* CMD_GOAL_POSITION (0x21) : Miroir de Position (common_type.h) */
typedef struct __attribute__((packed)) {
    float x;  /* [m] */
    float y;  /* [m] */
    float t;  /* [rad] */
} eth_payload_cmd_goal_position_t; /* 12 octets */

/* CMD_SET_LIDAR (0x22) : Miroir de Set_lidar (common_type.h) : pose mesuree par le lidar */
typedef struct __attribute__((packed)) {
    float x;         /* [m] */
    float y;         /* [m] */
    float t;         /* [rad] */
    uint32_t delay;  /* [ms] latence de calcul */
} eth_payload_cmd_set_lidar_t; /* 16 octets */

/* CMD_SET_CAMERA_1 (0x23) */
typedef eth_set_camera_t eth_payload_cmd_set_camera_1_t; /* 28 octets */

/* CMD_SET_CAMERA_2 (0x24) */
typedef eth_set_camera_t eth_payload_cmd_set_camera_2_t; /* 28 octets */

/* CMD_SET_CAMERA_3 (0x25) */
typedef eth_set_camera_t eth_payload_cmd_set_camera_3_t; /* 28 octets */

/* CMD_SET_SPEED_MAX (0x26) : format non defini */

/* CMD_BLOCK (0x27) : Bloque le robot (asserv_mode = 4) */
typedef struct __attribute__((packed)) {
    uint8_t value;  /* 1 */
} eth_payload_cmd_block_t; /* 1 octet */

/* CMD_FREE (0x28) : Libere le robot (asserv_mode = 0) */
typedef struct __attribute__((packed)) {
    uint8_t value;  /* 1 */
} eth_payload_cmd_free_t; /* 1 octet */

/* Verification de la taille des structures a la compilation */
_Static_assert(sizeof(eth_lidar_point_t) == 4, "eth_lidar_point_t");
_Static_assert(sizeof(eth_set_camera_t) == 28, "eth_set_camera_t");
_Static_assert(sizeof(eth_payload_heartbeat_t) == 10, "eth_payload_heartbeat_t");
_Static_assert(sizeof(eth_payload_odom_t) == 24, "eth_payload_odom_t");
_Static_assert(sizeof(eth_payload_imu_t) == 40, "eth_payload_imu_t");
_Static_assert(sizeof(eth_payload_robot_state_t) == 29, "eth_payload_robot_state_t");
_Static_assert(sizeof(eth_payload_au_t) == 5, "eth_payload_au_t");
_Static_assert(sizeof(eth_payload_leash_t) == 5, "eth_payload_leash_t");
_Static_assert(sizeof(eth_payload_lidar_scan_chunk_t) == 415, "eth_payload_lidar_scan_chunk_t");
_Static_assert(sizeof(eth_payload_cmd_goal_position_t) == 12, "eth_payload_cmd_goal_position_t");
_Static_assert(sizeof(eth_payload_cmd_set_lidar_t) == 16, "eth_payload_cmd_set_lidar_t");
_Static_assert(sizeof(eth_payload_cmd_set_camera_1_t) == 28, "eth_payload_cmd_set_camera_1_t");
_Static_assert(sizeof(eth_payload_cmd_set_camera_2_t) == 28, "eth_payload_cmd_set_camera_2_t");
_Static_assert(sizeof(eth_payload_cmd_set_camera_3_t) == 28, "eth_payload_cmd_set_camera_3_t");
_Static_assert(sizeof(eth_payload_cmd_block_t) == 1, "eth_payload_cmd_block_t");
_Static_assert(sizeof(eth_payload_cmd_free_t) == 1, "eth_payload_cmd_free_t");

#endif /* OPOSSUM_ROBOT_MESSAGES_H */
