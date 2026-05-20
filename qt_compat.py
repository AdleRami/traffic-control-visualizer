from __future__ import annotations

# PySide6를 우선 사용하고, 설치되어 있지 않으면 PyQt6로 fallback합니다.
try:
    QT_API = "PySide6"
    from PySide6.QtCore import QPointF, QRectF, QTimer, Qt
    from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPen
    from PySide6.QtWidgets import (
        QApplication,
        QAbstractItemView,
        QComboBox,
        QFrame,
        QGraphicsLineItem,
        QGraphicsScene,
        QGraphicsView,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QHeaderView,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QSizePolicy,
        QSpinBox,
        QTableWidget,
        QTableWidgetItem,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
except ImportError as pyside_error:
    QT_API = "PyQt6"
    try:
        from PyQt6.QtCore import QPointF, QRectF, QTimer, Qt
        from PyQt6.QtGui import QBrush, QColor, QFont, QPainter, QPen
        from PyQt6.QtWidgets import (
            QApplication,
            QAbstractItemView,
            QComboBox,
            QFrame,
            QGraphicsLineItem,
            QGraphicsScene,
            QGraphicsView,
            QGridLayout,
            QGroupBox,
            QHBoxLayout,
            QHeaderView,
            QLabel,
            QMainWindow,
            QMessageBox,
            QPushButton,
            QSizePolicy,
            QSpinBox,
            QTableWidget,
            QTableWidgetItem,
            QTextEdit,
            QVBoxLayout,
            QWidget,
        )
    except ImportError as pyqt_error:
        raise SystemExit(
            "PySide6 또는 PyQt6가 필요합니다. 예: python -m pip install PySide6"
        ) from pyqt_error


# Qt6 계열 enum 이름을 한 곳에서만 다루기 위한 별칭입니다.
LEFT_BUTTON = Qt.MouseButton.LeftButton
NO_BUTTON = Qt.MouseButton.NoButton
ALIGN_CENTER = Qt.AlignmentFlag.AlignCenter
DASH_LINE = Qt.PenStyle.DashLine
DOT_LINE = Qt.PenStyle.DotLine
SOLID_LINE = Qt.PenStyle.SolidLine
NO_PEN = Qt.PenStyle.NoPen
ANTIALIASING = QPainter.RenderHint.Antialiasing
SELECT_ROWS = QAbstractItemView.SelectionBehavior.SelectRows
NO_EDIT = QAbstractItemView.EditTrigger.NoEditTriggers
STRETCH = QHeaderView.ResizeMode.Stretch
RESIZE_TO_CONTENTS = QHeaderView.ResizeMode.ResizeToContents


