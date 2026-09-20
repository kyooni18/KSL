from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QApplication, QMessageBox

from .window import MainWindow


def project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_backend() -> Path:
    root = project_root()
    return root / "CLanding" / "build" / "landing_backend"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="KSP Shuttle Lander PyQt frontend")
    parser.add_argument("--backend", type=Path, default=default_backend())
    args = parser.parse_args(argv)

    app = QApplication(sys.argv[:1])
    app.setApplicationName("KSP Shuttle Lander")
    app.setOrganizationName("KSPShutltleLander")
    app.setApplicationVersion("1")
    app.setAttribute(Qt.ApplicationAttribute.AA_DontShowIconsInMenus, False)

    backend = args.backend.expanduser().resolve()
    if not backend.exists():
        QMessageBox.critical(
            None,
            "Backend not built",
            f"The C landing backend was not found at:\n{backend}\n\nRun: make -C CLanding",
        )
        return 2

    window = MainWindow(backend)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
