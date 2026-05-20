"""
AGV Traffic Control Visualizer
==============================

README / 실행 방법
------------------
1. Python 3 환경을 준비합니다.
2. Qt 바인딩을 설치합니다.
   - 권장: python -m pip install PySide6
   - 대안: python -m pip install PyQt6
3. 실행합니다.
   - python main.py

구현 메모
---------
- UI, 모델, traffic control 알고리즘을 별도 모듈로 분리했습니다.
- traffic_controller.py에는 RHCR 스타일 rolling-horizon 제어 로직이 있습니다.
- Canvas 위에서 마우스 휠을 굴리면 큰 맵을 확대/축소할 수 있습니다.
"""

from __future__ import annotations

import sys

from main_window import MainWindow
from qt_compat import QApplication


def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
