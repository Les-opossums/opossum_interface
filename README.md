# opossum_interfaces

Interface entre la carte Zynq et le PC ROS 2 (LattePanda), en **source unique**.
Ex `com_config_table`.

Le dépôt est un sous-module de deux dépôts :

| Dépôt | Chemin du sous-module | Ce qu'il utilise |
|---|---|---|
| Firmware Zynq (`Zynq_SW`) | `opossum_interfaces/` | `generated/c/*.h` |
| ROS 2 (`Ros2_base_cdf`) | `interfaces/opossum_interfaces/` | package ROS `opossum_interfaces` : en-tête C++, module Python, YAML texte |

## Contenu

| Fichier | Rôle |
|---|---|
| `protocol.yaml` | **ICD Ethernet** : en-tête de trame, canaux UDP, identifiants et payloads des messages. Seul fichier à éditer. |
| `gen.py` | Générateur (dépend uniquement de PyYAML) |
| `generated/c/ETH_protocol.h` | Firmware : canaux, identifiants, en-tête de trame, `ETH_PROTOCOL_HASH` |
| `generated/c/robot_messages.h` | Firmware : structures des payloads `eth_payload_*_t` |
| `generated/cpp/opossum_interfaces/eth_protocol.hpp` | `comm_node` : mêmes structures, tables canaux/messages, CRC |
| `generated/python/opossum_interfaces/eth_protocol.py` | Outils, tests, simulation : `encode`, `decode`, `build_frame`, `parse_frame` |
| `com_msgs.yaml`, `format_msgs.yaml` | Protocole **texte** historique (commandes `MOVE`, `PINCE`…), lu par `comm.py` et `zynq_simu.py` |
| `tests/` | Code généré à jour, aller-retour Python, disposition mémoire C et C++ comparée au YAML |

Le code généré est commité : le firmware se compile dans Vitis sans Python.

## Modifier le protocole

```bash
# 1. éditer protocol.yaml
python3 gen.py              # 2. régénérer generated/
python3 -m pytest -q tests  # 3. vérifier
# 4. commit ici, puis mettre à jour le sous-module dans Zynq_SW ET dans Ros2_base_cdf
```

- Changement incompatible (identifiant, champ, type, ordre) : incrémenter `version`.
- Le **hash** du protocole (`ETH_PROTOCOL_HASH`, CRC32 du contenu sémantique, sans `doc` ni `unit`)
  change dès qu'un élément du protocole change. Le firmware l'envoie dans `HEARTBEAT` (1 Hz) ;
  `comm_node` refuse d'envoyer des commandes si le hash reçu diffère du sien.
- `python3 gen.py --check` échoue si `generated/` n'est pas à jour (utilisé par la CI).
- `python3 gen.py --hash` affiche le hash courant.

## Format de `protocol.yaml`

```yaml
messages:
  ROBOT_STATE:
    id: 0x13
    channel: TELEMETRY          # canal défini dans `channels`
    doc: "texte libre"          # hors hash
    payload:
      - { name: x, type: f32, unit: m }
      - { name: motion_done, type: u8 }
```

- Types : `u8 u16 u32 i8 i16 i32 f32`, un type défini dans `types`, ou un tableau `T[N]`.
- Dernier champ tableau avec `count_field: <champ>` : seuls `<champ>` éléments sont émis (ex. `LIDAR_SCAN_CHUNK`).
- `payload: <type>` réutilise une structure de `types` (ex. `CMD_SET_CAMERA_1..3`).
- `payload: text` : texte ASCII ; `payload: null` : message réservé, format pas encore défini.
- Little-endian, structures packées.

Noms générés : `eth_payload_<message>_t` en C, `Payload<Message>` en C++
(`opossum_interfaces::eth`), `eth_protocol.encode("<MESSAGE>", ...)` en Python.

## Utilisation côté ROS

```cpp
#include <opossum_interfaces/eth_protocol.hpp>
namespace eth = opossum_interfaces::eth;
```

```python
from opossum_interfaces import eth_protocol as eth
frame = eth.build_frame("CMD_GOAL_POSITION", eth.encode("CMD_GOAL_POSITION", x=1.0, y=0.5, t=0.0))
```
