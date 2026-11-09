"""ezdxf's audit of every DXF file in a folder: the files DxfTests.WritesTheAuditSetWhenAsked writes.

Usage: dxf_audit.py folder

Prints each file's entity count, errors and fixes, and exits with 1 if any file has an error or a fix, or can't be read.
"""

import pathlib
import sys

import ezdxf
from ezdxf import recover


def main(folder):
    bad = 0
    files = sorted(pathlib.Path(folder).glob("*.dxf"))
    for path in files:
        try:
            # The strict reader first: it refuses what recover would quietly mend
            doc = ezdxf.readfile(path)
            auditor = doc.audit()
        except Exception as e:  # noqa: BLE001 - any failure to read is a failure
            print(f"{path.name}: unreadable: {e}")
            bad += 1
            continue
        _, recovered = recover.readfile(path)
        entities = len(doc.modelspace())
        problems = len(auditor.errors) + len(auditor.fixes) + len(recovered.errors) + len(recovered.fixes)
        print(f"{path.name}: {doc.dxfversion}, {entities} entities, {len(auditor.errors)} errors, {len(auditor.fixes)} fixes")
        for entry in list(auditor.errors) + list(auditor.fixes) + list(recovered.errors) + list(recovered.fixes):
            print(f"    {entry.message}")
        if problems:
            bad += 1
    print(f"{len(files)} files, {bad} with errors or fixes")
    return 1 if bad or not files else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
