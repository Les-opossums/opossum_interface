#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generateur de code de l'interface Zynq <-> ROS a partir de protocol.yaml.

  python3 gen.py           # regenere generated/
  python3 gen.py --check   # code retour 1 si generated/ n'est pas a jour (CI)
  python3 gen.py --hash    # affiche le hash du protocole

Sorties :
  generated/c/ETH_protocol.h                       firmware Zynq (canaux, ids, en-tete)
  generated/c/robot_messages.h                     firmware Zynq (payloads)
  generated/cpp/opossum_interfaces/eth_protocol.hpp  comm_node (ROS 2)
  generated/python/opossum_interfaces/eth_protocol.py  outils, tests, simulation

Dependance unique : PyYAML.
"""

import argparse
import json
import os
import re
import sys
import zlib

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "protocol.yaml")
OUT = os.path.join(HERE, "generated")

# type yaml -> (C, taille, format struct Python)
PRIM = {
    "u8": ("uint8_t", 1, "B"), "u16": ("uint16_t", 2, "H"), "u32": ("uint32_t", 4, "I"),
    "i8": ("int8_t", 1, "b"), "i16": ("int16_t", 2, "h"), "i32": ("int32_t", 4, "i"),
    "f32": ("float", 4, "f"),
}
ARRAY_RE = re.compile(r"^(\w+)\[(\d+)\]$")


class ProtocolError(Exception):
    pass


# --------------------------------------------------------------------------- modele
def camel(name):
    return "".join(p.capitalize() for p in name.lower().split("_"))


class Field:
    def __init__(self, d, types):
        self.name = d["name"]
        self.doc = d.get("doc", "")
        self.unit = d.get("unit", "")
        self.count_field = d.get("count_field")
        t = d["type"]
        m = ARRAY_RE.match(t)
        self.array = int(m.group(2)) if m else 0
        self.base = m.group(1) if m else t
        if self.base in PRIM:
            self.size1 = PRIM[self.base][1]
            self.struct_type = None
        elif self.base in types:
            self.struct_type = types[self.base]
            self.size1 = self.struct_type.size
        else:
            raise ProtocolError(f"type inconnu '{t}' (champ {self.name})")
        self.size = self.size1 * max(1, self.array)
        self.offset = 0

    def comment(self):
        parts = [p for p in (f"[{self.unit}]" if self.unit else "", self.doc) if p]
        return " ".join(parts)


class Struct:
    def __init__(self, name, fields_yaml, types, doc=""):
        self.name = name
        self.doc = doc
        self.fields = [Field(f, types) for f in fields_yaml]
        off = 0
        for f in self.fields:
            f.offset = off
            off += f.size
        self.size = off
        names = [f.name for f in self.fields]
        if len(set(names)) != len(names):
            raise ProtocolError(f"champ en double dans {name}")
        for i, f in enumerate(self.fields):
            if f.count_field:
                if i != len(self.fields) - 1 or not f.array:
                    raise ProtocolError(f"{name}.{f.name} : count_field reserve au dernier champ tableau")
                if f.count_field not in names[:i]:
                    raise ProtocolError(f"{name}.{f.name} : count_field '{f.count_field}' introuvable")

    @property
    def variable(self):
        return bool(self.fields) and self.fields[-1].count_field is not None

    @property
    def fixed_size(self):
        """Taille hors tableau variable final."""
        return self.fields[-1].offset if self.variable else self.size


class Message:
    def __init__(self, name, d, types, channels):
        self.name = name
        self.id = int(d["id"])
        self.channel = d["channel"]
        self.doc = d.get("doc", "")
        if self.channel not in channels:
            raise ProtocolError(f"{name} : canal inconnu '{self.channel}'")
        p = d.get("payload")
        self.kind = "none" if p is None else "text" if p == "text" else "struct"
        self.alias = None
        if isinstance(p, str) and p != "text":
            if p not in types:
                raise ProtocolError(f"{name} : type de payload inconnu '{p}'")
            self.alias = p
            self.struct = types[p]
        else:
            self.struct = Struct(name, p, types, self.doc) if self.kind == "struct" else None

    @property
    def c_type(self):
        return f"eth_payload_{self.name.lower()}_t"

    @property
    def cpp_type(self):
        return "Payload" + camel(self.name)


class Protocol:
    def __init__(self, raw):
        self.raw = raw
        self.version = int(raw["version"])
        fr = raw["frame"]
        self.magic = int(fr["magic"])
        self.max_payload = int(fr["max_payload"])
        self.raw_text_id = int(fr["raw_text_id"])
        self.crc_doc = fr.get("crc", "")
        self.types = {}
        for tname, td in (raw.get("types") or {}).items():
            self.types[tname] = Struct(tname, td["fields"], self.types, td.get("doc", ""))
        self.header = Struct("header", fr["header"], self.types)
        if [f.name for f in self.header.fields][-1] != "crc16":
            raise ProtocolError("le dernier champ de l'en-tete doit etre crc16")
        self.channels = raw["channels"]
        for cn, c in self.channels.items():
            if c["direction"] not in ("tx", "rx"):
                raise ProtocolError(f"canal {cn} : direction tx|rx")
        self.messages = [Message(n, d, self.types, self.channels) for n, d in raw["messages"].items()]
        ids = [m.id for m in self.messages]
        if len(set(ids)) != len(ids):
            raise ProtocolError("identifiant de message en double")
        if self.raw_text_id in ids:
            raise ProtocolError("raw_text_id utilise par un message")
        for m in self.messages:
            if m.struct and m.struct.size > self.max_payload:
                raise ProtocolError(f"{m.name} : payload {m.struct.size} > max_payload")
        self.hash = protocol_hash(raw)

    @property
    def crc_offset(self):
        return self.header.fields[-1].offset


def _strip(o):
    if isinstance(o, dict):
        return {k: _strip(v) for k, v in o.items() if k not in ("doc", "unit", "crc")}
    if isinstance(o, list):
        return [_strip(v) for v in o]
    return o


def protocol_hash(raw):
    """CRC32 du contenu semantique (sans doc/unit), independant de la mise en forme."""
    canon = json.dumps(_strip(raw), sort_keys=True, separators=(",", ":"))
    return zlib.crc32(canon.encode()) & 0xFFFFFFFF


def load(path=SRC):
    with open(path, encoding="utf-8") as f:
        return Protocol(yaml.safe_load(f))


# --------------------------------------------------------------------------- C
BANNER_C = """/* =========================================================================
 * GENERE AUTOMATIQUEMENT par opossum_interfaces/gen.py depuis protocol.yaml
 * NE PAS MODIFIER A LA MAIN : editer protocol.yaml puis lancer `python3 gen.py`.
 * ========================================================================= */
"""


def c_decl(f):
    ctype = PRIM[f.base][0] if f.base in PRIM else f"eth_{f.base}_t"
    return f"{ctype} {f.name}" + (f"[{f.array}]" if f.array else "") + ";"


def c_field(f, pad=0):
    decl = c_decl(f)
    com = f.comment()
    if f.count_field:
        com = (com + " " if com else "") + f"seuls `{f.count_field}` elements sont emis"
    return f"    {decl:<{pad}} /* {com} */" if com else f"    {decl}"


def octets(n):
    return f"{n} octet" + ("s" if n > 1 else "")


def c_struct(name, s, doc=""):
    pad = max(len(c_decl(f)) for f in s.fields) + 1
    lines = []
    if doc:
        lines.append(f"/* {doc} */")
    lines.append("typedef struct __attribute__((packed)) {")
    lines += [c_field(f, pad) for f in s.fields]
    lines.append(f"}} {name}; /* {octets(s.size)} */")
    return "\n".join(lines)


def gen_c_protocol(p):
    tx = [m for m in p.messages if p.channels[m.channel]["direction"] == "tx"]
    rx = [m for m in p.messages if p.channels[m.channel]["direction"] == "rx"]
    w = max(len(m.name) for m in p.messages) + 1
    cw = max(len(c) for c in p.channels)
    o = [BANNER_C, "#ifndef OPOSSUM_ETH_PROTOCOL_H", "#define OPOSSUM_ETH_PROTOCOL_H", "",
         "#include <stdint.h>", "#include <stdbool.h>", "",
         "/* --- IDENTIFIANTS RESEAU --- */",
         f"#define ETH_FRAME_MAGIC      0x{p.magic:04X}",
         f"#define ETH_PROTOCOL_VERSION {p.version}",
         f"#define ETH_PROTOCOL_HASH    0x{p.hash:08X}u /* hash de protocol.yaml, envoye dans HEARTBEAT */", ""]
    for cn, c in p.channels.items():
        o.append(f"#define {'ETH_' + cn + '_PORT':<20} {c['port']:<6} /* {c.get('doc', '')} */")
    o += ["", f"#define ETH_MAX_PAYLOAD      {p.max_payload}", "",
          "/* ------------------------------------------------------------------------",
          " * Table des canaux UDP. Colonnes : NOM, PORT, DIRECTION, FRAMED",
          " *   ETH_DIR_TX -> le Zynq emet vers ce port distant",
          " *   ETH_DIR_RX -> le Zynq ecoute sur ce port",
          " *   FRAMED (canaux RX) : false -> payload brut, handler appele avec ETH_MSG_RAW_TEXT",
          " * ------------------------------------------------------------------------ */",
          "typedef enum {", "    ETH_DIR_TX = 0,", "    ETH_DIR_RX = 1,", "} eth_channel_dir_t;", "",
          "#define ETH_CHANNEL_LIST(X) \\"]
    rows = []
    for cn, c in p.channels.items():
        port = "ETH_" + cn + "_PORT"
        rows.append(f"    X({cn + ',':<{cw + 1}} {port + ',':<20} ETH_DIR_{c['direction'].upper()}, "
                    f"{'true' if c['framed'] else 'false'})")
    o.append(" \\\n".join(rows))
    o += ["", "typedef enum {", "#define X(name, port, dir, framed) ETH_CHANNEL_##name,",
          "    ETH_CHANNEL_LIST(X)", "#undef X", "    ETH_CHANNEL_COUNT", "} eth_channel_id_t;", "",
          "/* ------------------------------------------------------------------------",
          " * Liste des messages UDP. Colonnes : NOM, ID_HEXA, CANAL",
          " * ------------------------------------------------------------------------ */",
          f"#define ETH_MSG_RAW_TEXT 0x{p.raw_text_id:02X} /* type factice des canaux non framés */", "",
          "#define ETH_MESSAGE_LIST(X) \\"]
    rows = ["    /* --- MESSAGES TX (Zynq -> PC) --- */"]
    rows += [f"    X({m.name + ',':<{w}} 0x{m.id:02X}, ETH_CHANNEL_{m.channel})" for m in tx]
    rows += ["    /* --- MESSAGES RX (PC -> Zynq) --- */"]
    rows += [f"    X({m.name + ',':<{w}} 0x{m.id:02X}, ETH_CHANNEL_{m.channel})" for m in rx]
    o.append(" \\\n".join(rows))
    o += ["", "typedef enum {", "#define X(name, id, channel) ETH_MSG_##name = id,",
          "    ETH_MESSAGE_LIST(X)", "#undef X", "} eth_msg_type_t;", "",
          "/* ------------------------------------------------------------------------",
          f" * En-tete de trame UDP. {p.crc_doc}",
          " * ------------------------------------------------------------------------ */",
          c_struct("eth_frame_header_t", p.header), "",
          f"#define ETH_FRAME_CRC_OFFSET {p.crc_offset} /* offset du champ crc16 */", "",
          "#endif /* OPOSSUM_ETH_PROTOCOL_H */", ""]
    return "\n".join(o)


def gen_c_messages(p):
    o = [BANNER_C, "#ifndef OPOSSUM_ROBOT_MESSAGES_H", "#define OPOSSUM_ROBOT_MESSAGES_H", "",
         "#include <stdint.h>", "",
         "/* Structures des payloads, identiques au bit pres cote Zynq et cote PC. */", ""]
    for tname, s in p.types.items():
        o += [c_struct(f"eth_{tname}_t", s, s.doc), ""]
    for m in p.messages:
        if m.alias:
            o += [f"/* {m.name} (0x{m.id:02X})" + (f" : {m.doc}" if m.doc else "") + " */",
                  f"typedef eth_{m.alias}_t {m.c_type}; /* {octets(m.struct.size)} */", ""]
        elif m.kind == "struct":
            o += [c_struct(m.c_type, m.struct, f"{m.name} (0x{m.id:02X}) : {m.doc}" if m.doc
                           else f"{m.name} (0x{m.id:02X})"), ""]
            for f in m.struct.fields:
                if f.array:
                    o.append(f"#define ETH_{m.name}_{f.name.upper()}_MAX {f.array}")
            if m.struct.variable:
                o.append(f"#define ETH_{m.name}_FIXED_SIZE {m.struct.fixed_size} "
                         f"/* taille sans le tableau `{m.struct.fields[-1].name}` */")
            if m.struct.variable or any(f.array for f in m.struct.fields):
                o.append("")
        else:
            what = "texte libre" if m.kind == "text" else "format non defini"
            o += [f"/* {m.name} (0x{m.id:02X}) : {what} */", ""]
    o += ["/* Verification de la taille des structures a la compilation */"]
    o += [f"_Static_assert(sizeof(eth_{t}_t) == {s.size}, \"eth_{t}_t\");" for t, s in p.types.items()]
    o += [f"_Static_assert(sizeof({m.c_type}) == {m.struct.size}, \"{m.c_type}\");"
          for m in p.messages if m.struct]
    o += ["", "#endif /* OPOSSUM_ROBOT_MESSAGES_H */", ""]
    return "\n".join(o)


# --------------------------------------------------------------------------- C++
def cpp_decl(f):
    ctype = PRIM[f.base][0] if f.base in PRIM else camel(f.base)
    return f"{ctype} {f.name}" + (f"[{f.array}]" if f.array else "") + ";"


def cpp_field(f, pad=0):
    decl = cpp_decl(f)
    com = f.comment()
    return f"    {decl:<{pad}} // {com}" if com else f"    {decl}"


def cpp_struct(name, s, doc=""):
    pad = max(len(cpp_decl(f)) for f in s.fields) + 1
    o = [f"// {doc}"] if doc else []
    o.append(f"struct {name} {{")
    o += [cpp_field(f, pad) for f in s.fields]
    o += ["};", f"static_assert(sizeof({name}) == {s.size}, \"{name}\");"]
    return "\n".join(o)


def gen_cpp(p):
    ch_names = list(p.channels)
    o = ["// =========================================================================",
         "// GENERE AUTOMATIQUEMENT par opossum_interfaces/gen.py depuis protocol.yaml",
         "// NE PAS MODIFIER A LA MAIN : editer protocol.yaml puis lancer `python3 gen.py`.",
         "// =========================================================================",
         "#pragma once", "", "#include <cstddef>", "#include <cstdint>", "",
         "namespace opossum_interfaces {", "namespace eth {", "",
         f"constexpr uint16_t FRAME_MAGIC      = 0x{p.magic:04X};",
         f"constexpr uint8_t  PROTOCOL_VERSION = {p.version};",
         f"constexpr uint32_t PROTOCOL_HASH    = 0x{p.hash:08X}u;",
         f"constexpr uint16_t MAX_PAYLOAD      = {p.max_payload};",
         f"constexpr uint8_t  MSG_RAW_TEXT     = 0x{p.raw_text_id:02X};",
         f"constexpr size_t   FRAME_CRC_OFFSET = {p.crc_offset};", "",
         "// Direction vue du Zynq : TX = le Zynq emet (le PC ecoute), RX = le Zynq ecoute",
         "enum class ChannelDir : uint8_t { TX = 0, RX = 1 };", "",
         "enum class Channel : uint8_t {"]
    o += [f"    {cn} = {i}," for i, cn in enumerate(ch_names)]
    o += ["};", f"constexpr size_t CHANNEL_COUNT = {len(ch_names)};", "",
          "struct ChannelInfo { const char *name; uint16_t port; ChannelDir dir; bool framed; };",
          "constexpr ChannelInfo CHANNELS[CHANNEL_COUNT] = {"]
    for cn, c in p.channels.items():
        o.append(f"    {{\"{cn}\", {c['port']}, ChannelDir::{c['direction'].upper()}, "
                 f"{'true' if c['framed'] else 'false'}}},")
    o += ["};", "", "enum MsgId : uint8_t {"]
    o += [f"    MSG_{m.name} = 0x{m.id:02X}," for m in p.messages]
    o += ["};", "", "// kind : 0 = format non defini, 1 = texte, 2 = structure",
          "struct MessageInfo { const char *name; uint8_t id; Channel channel; uint8_t kind; uint16_t size; };",
          f"constexpr size_t MESSAGE_COUNT = {len(p.messages)};",
          "constexpr MessageInfo MESSAGES[MESSAGE_COUNT] = {"]
    kinds = {"none": 0, "text": 1, "struct": 2}
    for m in p.messages:
        size = m.struct.size if m.struct else 0
        o.append(f"    {{\"{m.name}\", 0x{m.id:02X}, Channel::{m.channel}, {kinds[m.kind]}, {size}}},")
    o += ["};", "",
          "inline const MessageInfo *message_by_id(uint8_t id) {",
          "    for (const auto &m : MESSAGES) { if (m.id == id) return &m; }",
          "    return nullptr;", "}", "",
          "inline const MessageInfo *message_by_name(const char *name) {",
          "    for (const auto &m : MESSAGES) {",
          "        const char *a = m.name, *b = name;",
          "        while (*a && *a == *b) { ++a; ++b; }",
          "        if (*a == *b) return &m;", "    }", "    return nullptr;", "}", "",
          "inline const ChannelInfo &channel_info(Channel c) { return CHANNELS[static_cast<size_t>(c)]; }", "",
          "#pragma pack(push, 1)", cpp_struct("FrameHeader", p.header), ""]
    for tname, s in p.types.items():
        o += [cpp_struct(camel(tname), s, s.doc), ""]
    for m in p.messages:
        if m.alias:
            o += [f"// {m.name} (0x{m.id:02X})" + (f" : {m.doc}" if m.doc else ""),
                  f"using {m.cpp_type} = {camel(m.alias)};", ""]
        elif m.struct:
            o += [cpp_struct(m.cpp_type, m.struct, f"{m.name} (0x{m.id:02X})" + (f" : {m.doc}" if m.doc else "")), ""]
    o += ["#pragma pack(pop)", "",
          f"// {p.crc_doc}",
          "inline uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF) {",
          "    for (size_t i = 0; i < len; ++i) {",
          "        crc ^= static_cast<uint16_t>(data[i]) << 8;",
          "        for (int b = 0; b < 8; ++b) {",
          "            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)",
          "                                  : static_cast<uint16_t>(crc << 1);",
          "        }", "    }", "    return crc;", "}", "",
          "}  // namespace eth", "}  // namespace opossum_interfaces", ""]
    return "\n".join(o)


# --------------------------------------------------------------------------- Python
def py_layout(s, types):
    """Format struct (sans '<') et liste des noms ; les sous-structures sont aplaties."""
    fmt, names = "", []
    for f in s.fields:
        if f.count_field:
            continue
        n = max(1, f.array)
        if f.base in PRIM:
            fmt += (str(n) if n > 1 else "") + PRIM[f.base][2]
            names += [f.name] if n == 1 else [f"{f.name}[{i}]" for i in range(n)]
        else:
            sub_fmt, sub_names = py_layout(types[f.base], types)
            for i in range(n):
                fmt += sub_fmt
                pre = f.name if n == 1 else f"{f.name}[{i}]"
                names += [f"{pre}.{sn}" for sn in sub_names]
    return fmt, names


def gen_py(p):
    o = ['# -*- coding: utf-8 -*-',
         '"""GENERE AUTOMATIQUEMENT par opossum_interfaces/gen.py depuis protocol.yaml.',
         'NE PAS MODIFIER A LA MAIN : editer protocol.yaml puis lancer `python3 gen.py`.',
         '',
         'Exemple :',
         '    from opossum_interfaces import eth_protocol as eth',
         '    frame = eth.build_frame("CMD_GOAL_POSITION", eth.encode("CMD_GOAL_POSITION", x=1.0, y=0.5, t=0.0))',
         '    hdr, name, payload = eth.parse_frame(frame)',
         '    fields = eth.decode(name, payload)',
         '"""', '', 'import struct', '',
         f"FRAME_MAGIC = 0x{p.magic:04X}",
         f"PROTOCOL_VERSION = {p.version}",
         f"PROTOCOL_HASH = 0x{p.hash:08X}",
         f"MAX_PAYLOAD = {p.max_payload}",
         f"MSG_RAW_TEXT = 0x{p.raw_text_id:02X}",
         f"FRAME_CRC_OFFSET = {p.crc_offset}", "",
         "# nom -> (port, direction vue du Zynq, framed)", "CHANNELS = {"]
    for cn, c in p.channels.items():
        o.append(f"    {cn!r}: ({c['port']}, {c['direction']!r}, {bool(c['framed'])}),")
    o += ["}", "", "# nom -> (id, canal, kind) ; kind : None | 'text' | 'struct'", "MESSAGES = {"]
    for m in p.messages:
        kind = None if m.kind == "none" else m.kind
        o.append(f"    {m.name!r}: (0x{m.id:02X}, {m.channel!r}, {kind!r}),")
    o += ["}", "MESSAGE_BY_ID = {v[0]: k for k, v in MESSAGES.items()}", "",
          "_HEADER = struct.Struct(\"<" + py_layout(p.header, p.types)[0] + "\")",
          "HEADER_SIZE = _HEADER.size",
          "HEADER_FIELDS = " + repr(py_layout(p.header, p.types)[1]), "",
          "# nom -> (format fixe, noms aplatis, tableau variable (nom, champ compteur, format, noms) ou None)",
          "PAYLOADS = {"]
    for m in p.messages:
        if not m.struct:
            continue
        fmt, names = py_layout(m.struct, p.types)
        var = None
        if m.struct.variable:
            f = m.struct.fields[-1]
            if f.base in PRIM:
                vfmt, vnames = PRIM[f.base][2], [""]
            else:
                vfmt, vnames = py_layout(p.types[f.base], p.types)
            var = (f.name, f.count_field, vfmt, vnames)
        o.append(f"    {m.name!r}: ({'<' + fmt!r}, {names!r}, {var!r}),")
    o += ["}", "", "",
          "def crc16_ccitt(data, crc=0xFFFF):",
          f'    """{p.crc_doc}"""',
          "    for byte in data:",
          "        crc ^= byte << 8",
          "        for _ in range(8):",
          "            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF",
          "    return crc", "", "",
          "def _nest(flat):",
          '    """{"a.b": 1, "p[0].x": 2} -> {"a": {"b": 1}, "p": [{"x": 2}]}"""',
          "    out = {}",
          "    for key, val in flat.items():",
          "        cur = out",
          "        parts = key.split('.')",
          "        for i, part in enumerate(parts):",
          "            last = i == len(parts) - 1",
          "            if '[' in part:",
          "                name, idx = part[:-1].split('[')",
          "                lst = cur.setdefault(name, [])",
          "                idx = int(idx)",
          "                while len(lst) <= idx:",
          "                    lst.append({})",
          "                if last:",
          "                    lst[idx] = val",
          "                else:",
          "                    cur = lst[idx]",
          "            elif last:",
          "                cur[part] = val",
          "            else:",
          "                cur = cur.setdefault(part, {})",
          "    return out", "", "",
          "def _flatten(obj, prefix=''):",
          "    out = {}",
          "    if isinstance(obj, dict):",
          "        for k, v in obj.items():",
          "            out.update(_flatten(v, f'{prefix}.{k}' if prefix else k))",
          "    elif isinstance(obj, (list, tuple)):",
          "        for i, v in enumerate(obj):",
          "            out.update(_flatten(v, f'{prefix}[{i}]'))",
          "    else:",
          "        out[prefix] = obj",
          "    return out", "", "",
          "def encode(name, **fields):",
          '    """Encode le payload d\'un message structure. Les tableaux variables se passent en liste."""',
          "    fmt, names, var = PAYLOADS[name]",
          "    items = []",
          "    if var:",
          "        items = list(fields.pop(var[0], []))",
          "        fields[var[1]] = len(items)",
          "    flat = _flatten(fields)",
          "    missing = [n for n in names if n not in flat]",
          "    if missing:",
          "        raise KeyError(f'{name} : champs manquants {missing}')",
          "    data = struct.pack(fmt, *(flat[n] for n in names))",
          "    if var:",
          "        vs = struct.Struct('<' + var[2])",
          "        for it in items:",
          "            f = _flatten(it) if isinstance(it, dict) else {'': it}",
          "            data += vs.pack(*(f[n] for n in var[3]))",
          "    return data", "", "",
          "def decode(name, payload):",
          '    """Decode un payload ; retourne un dict (texte : str, format non defini : bytes)."""',
          "    kind = MESSAGES[name][2]",
          "    if kind == 'text':",
          "        return payload.decode('ascii', errors='replace')",
          "    if kind is None:",
          "        return bytes(payload)",
          "    fmt, names, var = PAYLOADS[name]",
          "    size = struct.calcsize(fmt)",
          "    if len(payload) < size:",
          "        raise ValueError(f'{name} : payload trop court ({len(payload)} < {size})')",
          "    out = _nest(dict(zip(names, struct.unpack_from(fmt, payload))))",
          "    if var:",
          "        vs = struct.Struct('<' + var[2])",
          "        n = out[var[1]]",
          "        if len(payload) != size + n * vs.size:",
          "            raise ValueError(f'{name} : taille {len(payload)} incoherente avec {var[1]}={n}')",
          "        items = []",
          "        for i in range(n):",
          "            vals = vs.unpack_from(payload, size + i * vs.size)",
          "            items.append(vals[0] if var[3] == [''] else _nest(dict(zip(var[3], vals))))",
          "        out[var[0]] = items",
          "    elif len(payload) != size:",
          "        raise ValueError(f'{name} : taille {len(payload)} != {size}')",
          "    return out", "", "",
          "def build_frame(name, payload=b'', seq=0, timestamp_us=0):",
          '    """Construit une trame complete (en-tete + CRC + payload)."""',
          "    if isinstance(payload, str):",
          "        payload = payload.encode('ascii')",
          "    if len(payload) > MAX_PAYLOAD:",
          "        raise ValueError('payload trop grand')",
          "    hdr = dict(magic=FRAME_MAGIC, version=PROTOCOL_VERSION, msg_type=MESSAGES[name][0],",
          "               seq=seq & 0xFFFF, timestamp_us=timestamp_us & 0xFFFFFFFF,",
          "               payload_len=len(payload), crc16=0)",
          "    raw = bytearray(_HEADER.pack(*(hdr[n] for n in HEADER_FIELDS)) + payload)",
          "    struct.pack_into('<H', raw, FRAME_CRC_OFFSET, crc16_ccitt(raw))",
          "    return bytes(raw)", "", "",
          "def parse_frame(frame):",
          '    """Verifie et decoupe une trame ; retourne (en-tete dict, nom du message, payload)."""',
          "    if len(frame) < HEADER_SIZE:",
          "        raise ValueError('trame trop courte')",
          "    hdr = dict(zip(HEADER_FIELDS, _HEADER.unpack_from(frame)))",
          "    if hdr['magic'] != FRAME_MAGIC:",
          "        raise ValueError('magic invalide')",
          "    if HEADER_SIZE + hdr['payload_len'] != len(frame):",
          "        raise ValueError('payload_len incoherent')",
          "    raw = bytearray(frame)",
          "    struct.pack_into('<H', raw, FRAME_CRC_OFFSET, 0)",
          "    if crc16_ccitt(raw) != hdr['crc16']:",
          "        raise ValueError('CRC invalide')",
          "    name = MESSAGE_BY_ID.get(hdr['msg_type'], 'UNKNOWN')",
          "    return hdr, name, bytes(frame[HEADER_SIZE:])", ""]
    return "\n".join(o)


# --------------------------------------------------------------------------- main
def outputs(p):
    return {
        "c/ETH_protocol.h": gen_c_protocol(p),
        "c/robot_messages.h": gen_c_messages(p),
        "cpp/opossum_interfaces/eth_protocol.hpp": gen_cpp(p),
        "python/opossum_interfaces/eth_protocol.py": gen_py(p),
        "python/opossum_interfaces/__init__.py":
            '"""Interface Zynq <-> ROS generee depuis protocol.yaml (voir gen.py)."""\n',
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="verifie que generated/ est a jour")
    ap.add_argument("--hash", action="store_true", help="affiche le hash du protocole")
    args = ap.parse_args()
    try:
        p = load()
    except (ProtocolError, KeyError, ValueError) as e:
        sys.exit(f"protocol.yaml invalide : {e}")
    if args.hash:
        print(f"0x{p.hash:08X}")
        return
    stale = []
    for rel, content in outputs(p).items():
        path = os.path.join(OUT, rel)
        old = open(path, encoding="utf-8").read() if os.path.exists(path) else None
        if old == content:
            continue
        stale.append(rel)
        if not args.check:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8", newline="\n") as f:
                f.write(content)
    if args.check:
        if stale:
            print("generated/ n'est pas a jour, lancer `python3 gen.py` :", *stale, sep="\n  ")
            sys.exit(1)
        print(f"generated/ a jour (hash 0x{p.hash:08X})")
    else:
        print(f"protocole v{p.version}, hash 0x{p.hash:08X}, {len(stale)} fichier(s) regenere(s)")


if __name__ == "__main__":
    main()
