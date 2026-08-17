#!/usr/bin/env python3
"""Trunca un vocabulario DBoW2 de texto y remapea su arbol de forma valida."""

from __future__ import annotations

import argparse
import hashlib
import os
from array import array
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--depth", type=int, default=5)
    return parser.parse_args()


def first_pass(source: Path, target_depth: int):
    depths = array("B", [0])
    child_weight_sum = array("d", [0.0])
    child_count = array("I", [0])

    with source.open("r", encoding="ascii") as stream:
        header = stream.readline().split()
        if len(header) != 4:
            raise ValueError("cabecera DBoW2 invalida")
        k, source_depth, scoring, weighting = map(int, header)
        if target_depth < 1 or target_depth >= source_depth:
            raise ValueError("la profundidad destino debe estar entre 1 y L-1")

        for node_id, line in enumerate(stream, start=1):
            fields = line.split()
            if not fields:
                continue
            if len(fields) != 35:
                raise ValueError(f"nodo {node_id}: se esperaban 35 campos")
            parent = int(fields[0])
            if parent >= len(depths):
                raise ValueError(f"nodo {node_id}: padre {parent} no disponible")
            depth = depths[parent] + 1
            depths.append(depth)
            child_weight_sum.append(0.0)
            child_count.append(0)
            if depth == target_depth + 1:
                child_weight_sum[parent] += float(fields[-1])
                child_count[parent] += 1

    return (k, source_depth, scoring, weighting), depths, child_weight_sum, child_count


def write_compact(
    source: Path,
    output: Path,
    target_depth: int,
    header,
    depths,
    child_weight_sum,
    child_count,
):
    k, _, scoring, weighting = header
    old_to_new = array("I", [0])
    compact_nodes = 0
    compact_words = 0
    temporary = output.with_suffix(output.suffix + ".tmp")
    output.parent.mkdir(parents=True, exist_ok=True)

    with source.open("r", encoding="ascii") as source_stream, temporary.open(
        "w", encoding="ascii"
    ) as output_stream:
        source_stream.readline()
        output_stream.write(f"{k} {target_depth} {scoring} {weighting}\n")

        for old_id, line in enumerate(source_stream, start=1):
            fields = line.split()
            if not fields:
                continue
            depth = depths[old_id]
            if depth > target_depth:
                old_to_new.append(0)
                continue

            parent = int(fields[0])
            old_to_new.append(compact_nodes + 1)
            is_leaf = int(fields[1]) != 0 or depth == target_depth
            weight = float(fields[-1])
            if depth == target_depth and int(fields[1]) == 0:
                count = child_count[old_id]
                if count == 0:
                    raise ValueError(f"nodo {old_id}: hoja truncada sin hijos")
                weight = child_weight_sum[old_id] / count

            descriptor = " ".join(fields[2:-1])
            output_stream.write(
                f"{old_to_new[parent]} {1 if is_leaf else 0} "
                f"{descriptor} {weight:.17g}\n"
            )
            compact_nodes += 1
            compact_words += int(is_leaf)

    os.replace(temporary, output)
    return compact_nodes, compact_words


def validate(path: Path, expected_depth: int):
    depths = [0]
    nodes = words = 0
    with path.open("r", encoding="ascii") as stream:
        header = list(map(int, stream.readline().split()))
        if len(header) != 4 or header[1] != expected_depth:
            raise ValueError("cabecera compacta invalida")
        for node_id, line in enumerate(stream, start=1):
            fields = line.split()
            if len(fields) != 35:
                raise ValueError(f"nodo compacto {node_id}: formato invalido")
            parent = int(fields[0])
            if parent >= node_id:
                raise ValueError(f"nodo compacto {node_id}: padre no anterior")
            depth = depths[parent] + 1
            if depth > expected_depth:
                raise ValueError(f"nodo compacto {node_id}: profundidad excedida")
            is_leaf = int(fields[1]) != 0
            if depth == expected_depth and not is_leaf:
                raise ValueError(f"nodo compacto {node_id}: nivel final no es hoja")
            depths.append(depth)
            nodes += 1
            words += int(is_leaf)
    return nodes, words


def main() -> int:
    args = parse_args()
    header, depths, weight_sum, child_count = first_pass(args.input, args.depth)
    nodes, words = write_compact(
        args.input,
        args.output,
        args.depth,
        header,
        depths,
        weight_sum,
        child_count,
    )
    validated_nodes, validated_words = validate(args.output, args.depth)
    if (nodes, words) != (validated_nodes, validated_words):
        raise RuntimeError("los conteos de validacion no coinciden")
    digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
    print(
        f"[ORB-VOC-COMPACT] depth={args.depth} nodes={nodes} words={words} "
        f"bytes={args.output.stat().st_size} sha256={digest}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
