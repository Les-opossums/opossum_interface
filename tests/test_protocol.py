# -*- coding: utf-8 -*-
"""Tests de l'ICD : code genere a jour, Python aller-retour, disposition memoire C/C++."""

import os
import shutil
import subprocess
import sys
import tempfile

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "generated", "python"))

import gen  # noqa: E402
from opossum_interfaces import eth_protocol as eth  # noqa: E402

P = gen.load()


def test_generated_up_to_date():
    r = subprocess.run([sys.executable, os.path.join(ROOT, "gen.py"), "--check"], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


def test_hash_matches_generated():
    assert eth.PROTOCOL_HASH == P.hash
    assert eth.PROTOCOL_VERSION == P.version


def test_hash_ignores_doc_and_unit():
    raw = yaml.safe_load(open(gen.SRC, encoding="utf-8"))
    raw["messages"]["ODOM"]["doc"] = "autre commentaire"
    raw["messages"]["ODOM"]["payload"][0]["unit"] = "mm"
    assert gen.protocol_hash(raw) == P.hash
    raw["messages"]["ODOM"]["id"] = 0x7F
    assert gen.protocol_hash(raw) != P.hash


def test_rejects_duplicate_id():
    raw = yaml.safe_load(open(gen.SRC, encoding="utf-8"))
    raw["messages"]["IMU"]["id"] = raw["messages"]["ODOM"]["id"]
    with pytest.raises(gen.ProtocolError):
        gen.Protocol(raw)


def test_crc_reference_vector():
    # CRC-16/CCITT-FALSE de "123456789"
    assert eth.crc16_ccitt(b"123456789") == 0x29B1


def _sample(struct, value=1):
    out = {}
    for f in struct.fields:
        if f.count_field:
            continue
        if f.base in gen.PRIM:
            v = 1.5 if f.base == "f32" else value
            out[f.name] = [v] * f.array if f.array else v
        else:
            sub = _sample(P.types[f.base], value)
            out[f.name] = [sub] * f.array if f.array else sub
    return out


@pytest.mark.parametrize("m", [m for m in P.messages if m.struct], ids=lambda m: m.name)
def test_roundtrip(m):
    fields = _sample(m.struct)
    if m.struct.variable:
        last = m.struct.fields[-1]
        fields.pop(last.count_field)
        fields[last.name] = [_sample(P.types[last.base], 7)] * 3
    payload = eth.encode(m.name, **fields)
    expected = m.struct.fixed_size + (3 * m.struct.fields[-1].size1 if m.struct.variable else 0)
    assert len(payload) == expected
    frame = eth.build_frame(m.name, payload, seq=42)
    hdr, name, pl = eth.parse_frame(frame)
    assert name == m.name and hdr["seq"] == 42 and pl == payload
    back = eth.decode(name, pl)
    for k, v in fields.items():
        assert back[k] == v


def test_corrupted_frame_rejected():
    frame = bytearray(eth.build_frame("CMD_GENERIC", "MOVE 1 2 3"))
    frame[-1] ^= 0xFF
    with pytest.raises(ValueError):
        eth.parse_frame(bytes(frame))


def _offsets_program(lang):
    lines = []
    structs = [("eth_frame_header_t" if lang == "c" else "FrameHeader", P.header)]
    for m in P.messages:
        if m.struct:
            structs.append((m.c_type if lang == "c" else m.cpp_type, m.struct))
    for cname, s in structs:
        lines.append(f'printf("{cname} %zu\\n", sizeof({cname}));')
        for f in s.fields:
            lines.append(f'printf("{cname}.{f.name} %zu\\n", offsetof({cname}, {f.name}));')
    expected = []
    for cname, s in structs:
        expected.append(f"{cname} {s.size}")
        expected += [f"{cname}.{f.name} {f.offset}" for f in s.fields]
    return "\n    ".join(lines), expected


@pytest.mark.skipif(shutil.which("gcc") is None, reason="gcc absent")
def test_c_layout():
    body, expected = _offsets_program("c")
    src = f"""#include <stdio.h>
#include <stddef.h>
#include "ETH_protocol.h"
#include "robot_messages.h"
int main(void) {{
    {body}
    return 0;
}}
"""
    out = _compile_run("gcc", src, ".c", ["-std=c11", "-Wall", "-Wextra", "-pedantic", "-Werror",
                                          "-I", os.path.join(ROOT, "generated", "c")])
    assert out.split("\n") == expected


@pytest.mark.skipif(shutil.which("g++") is None, reason="g++ absent")
def test_cpp_layout():
    body, expected = _offsets_program("cpp")
    src = f"""#include <cstdio>
#include <cstddef>
#include "opossum_interfaces/eth_protocol.hpp"
using namespace opossum_interfaces::eth;
int main() {{
    {body}
    return message_by_name("ROBOT_STATE")->id == MSG_ROBOT_STATE ? 0 : 1;
}}
"""
    out = _compile_run("g++", src, ".cpp", ["-std=c++17", "-Wall", "-Wextra", "-Werror",
                                            "-Wno-invalid-offsetof",
                                            "-I", os.path.join(ROOT, "generated", "cpp")])
    assert out.split("\n") == expected


def _compile_run(cc, src, ext, flags):
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "t" + ext)
        with open(path, "w") as f:
            f.write(src)
        exe = os.path.join(d, "t")
        subprocess.run([cc, *flags, path, "-o", exe], check=True, capture_output=True, text=True)
        return subprocess.run([exe], check=True, capture_output=True, text=True).stdout.strip()
