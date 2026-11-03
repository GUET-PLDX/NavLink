#!/usr/bin/env python3

from pathlib import Path


HEADER = Path(__file__).resolve().parents[1] / "NavLink.hpp"


def main():
    source = HEADER.read_text(encoding="utf-8")
    assert "FindOrCreate<Data>(name)" in source
    assert "FindOrCreate<Data>(name, nullptr, true)" not in source
    print("PASS: named callbacks do not require multi-publisher topics")


if __name__ == "__main__":
    main()
