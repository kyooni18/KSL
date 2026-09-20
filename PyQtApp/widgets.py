from __future__ import annotations

import math
from typing import Any, Iterable

from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QColor, QBrush, QFont, QLinearGradient, QPainter, QPainterPath, QPen
from PyQt6.QtWidgets import QWidget


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_degrees(value: float) -> float:
    wrapped = value % 360.0
    return wrapped + 360.0 if wrapped < 0 else wrapped


def signed_degrees(value: float) -> float:
    wrapped = normalize_degrees(value)
    return wrapped - 360.0 if wrapped > 180.0 else wrapped


def great_circle_distance(a: dict[str, Any], b: dict[str, Any], radius: float) -> float:
    lat1 = math.radians(float(a.get("latitude", 0.0)))
    lat2 = math.radians(float(b.get("latitude", 0.0)))
    dlat = lat2 - lat1
    dlon = math.radians(float(b.get("longitude", 0.0)) - float(a.get("longitude", 0.0)))
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return radius * 2 * math.atan2(math.sqrt(max(0.0, h)), math.sqrt(max(0.0, 1.0 - h)))


def initial_bearing(a: dict[str, Any], b: dict[str, Any]) -> float:
    lat1 = math.radians(float(a.get("latitude", 0.0)))
    lat2 = math.radians(float(b.get("latitude", 0.0)))
    dlon = math.radians(float(b.get("longitude", 0.0)) - float(a.get("longitude", 0.0)))
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return normalize_degrees(math.degrees(math.atan2(y, x)))


def local_offsets(site: dict[str, Any], point: dict[str, Any], radius: float) -> tuple[float, float]:
    distance = great_circle_distance(site, point, radius)
    bearing = math.radians(initial_bearing(site, point))
    return distance * math.sin(bearing), distance * math.cos(bearing)


class AttitudeHUD(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.telemetry: dict[str, Any] = {}
        self.command: dict[str, Any] = {}
        self.instrument_scale = 1.0
        self.setMinimumHeight(220)

    def set_state(self, telemetry: dict[str, Any], command: dict[str, Any]) -> None:
        self.telemetry = telemetry or {}
        self.command = command or {}
        self.update()

    def set_instrument_scale(self, scale: float) -> None:
        self.instrument_scale = clamp(scale, 0.7, 1.6)
        self.update()

    def paintEvent(self, _event) -> None:  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        background = QLinearGradient(0, 0, 0, self.height())
        background.setColorAt(0.0, QColor(8, 20, 31))
        background.setColorAt(0.48, QColor(19, 42, 58))
        background.setColorAt(1.0, QColor(8, 12, 19))
        painter.fillRect(self.rect(), QBrush(background))
        painter.setPen(QPen(QColor(100, 125, 145, 90), 1.0))
        painter.drawRect(self.rect().adjusted(0, 0, -1, -1))

        width = float(self.width())
        height = float(self.height())
        cx, cy = width / 2, height / 2
        ppd = max(2.2, min(width, height) / 65.0) * self.instrument_scale
        roll = math.radians(-float(self.telemetry.get("roll", 0.0)))
        pitch_offset = float(self.telemetry.get("pitch", 0.0)) * ppd

        def transform(x: float, y: float) -> QPointF:
            yy = y + pitch_offset
            return QPointF(cx + x * math.cos(roll) - yy * math.sin(roll), cy + x * math.sin(roll) + yy * math.cos(roll))

        # Paint only the ground half-plane in instrument coordinates. The sky
        # remains the dashboard gradient above, so roll/pitch can move the
        # horizon without a giant pen stroke repainting the whole widget.
        painter.save()
        painter.setClipRect(self.rect())
        painter.translate(cx, cy)
        painter.rotate(math.degrees(roll))
        painter.translate(0.0, pitch_offset)
        ground = QLinearGradient(0, 0, 0, max(height, 1.0) * 1.5)
        ground.setColorAt(0.0, QColor(78, 58, 45))
        ground.setColorAt(1.0, QColor(32, 27, 25))
        painter.fillRect(QRectF(-width * 2.0, 0.0, width * 4.0, height * 4.0), QBrush(ground))
        painter.setPen(QPen(QColor(235, 238, 244, 230), 2.0))
        painter.drawLine(QPointF(-width * 2.0, 0.0), QPointF(width * 2.0, 0.0))
        painter.restore()

        painter.save()
        painter.setClipRect(self.rect())
        painter.setPen(QPen(QColor(235, 238, 244, 230), 2.0))
        painter.drawLine(transform(-width, 0), transform(width, 0))

        label_font = QFont("Menlo", 9)
        painter.setFont(label_font)
        for pitch in range(-40, 41, 5):
            if pitch == 0:
                continue
            major = pitch % 10 == 0
            line_width = 62.0 if major else 34.0
            y = -pitch * ppd
            painter.setPen(QPen(QColor(230, 233, 240, 165), 1.4 if major else 0.8))
            painter.drawLine(transform(-line_width, y), transform(line_width, y))
            if major:
                painter.setPen(QColor(230, 233, 240, 190))
                lp = transform(-line_width - 24, y)
                rp = transform(line_width + 24, y)
                painter.drawText(QRectF(lp.x() - 16, lp.y() - 9, 32, 18), Qt.AlignmentFlag.AlignCenter, str(abs(pitch)))
                painter.drawText(QRectF(rp.x() - 16, rp.y() - 9, 32, 18), Qt.AlignmentFlag.AlignCenter, str(abs(pitch)))
        painter.restore()

        radius = min(width, height) * 0.37
        painter.setPen(QPen(QColor(235, 238, 244, 195), 1.0))
        for angle in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            radians = math.radians(angle - 90)
            outer = QPointF(cx + math.cos(radians) * radius, cy + math.sin(radians) * radius)
            inner_r = radius - (13 if angle in (0, -60, -30, 30, 60) else 8)
            inner = QPointF(cx + math.cos(radians) * inner_r, cy + math.sin(radians) * inner_r)
            painter.drawLine(inner, outer)

        actual_roll = float(self.telemetry.get("roll", 0.0))
        rr = math.radians(actual_roll - 90)
        marker = QPointF(cx + math.cos(rr) * (radius - 2), cy + math.sin(rr) * (radius - 2))
        triangle = QPainterPath(marker)
        triangle.lineTo(marker.x() - 6, marker.y() - 10)
        triangle.lineTo(marker.x() + 6, marker.y() - 10)
        triangle.closeSubpath()
        painter.fillPath(triangle, QColor(55, 225, 116))

        heading_diff = signed_degrees(float(self.telemetry.get("groundTrackHeading", 0.0)) - float(self.telemetry.get("heading", 0.0)))
        fpa = float(self.telemetry.get("flightPathAngle", 0.0))
        pitch = float(self.telemetry.get("pitch", 0.0))
        fx = cx + clamp(heading_diff, -25, 25) * ppd
        fy = cy - clamp(fpa - pitch, -20, 20) * ppd
        painter.setPen(QPen(QColor(55, 225, 116), 2.0))
        painter.drawEllipse(QRectF(fx - 8, fy - 8, 16, 16))
        painter.drawLine(QPointF(fx - 20, fy), QPointF(fx - 8, fy))
        painter.drawLine(QPointF(fx + 8, fy), QPointF(fx + 20, fy))
        painter.drawLine(QPointF(fx, fy - 16), QPointF(fx, fy - 8))

        painter.setPen(QPen(QColor(245, 210, 71), 2.5))
        aircraft = QPainterPath(QPointF(cx - 55, cy))
        aircraft.lineTo(cx - 13, cy)
        aircraft.lineTo(cx, cy + 8)
        aircraft.lineTo(cx + 13, cy)
        aircraft.lineTo(cx + 55, cy)
        painter.drawPath(aircraft)

        if self.command.get("autopilotEngaged") and not self.command.get("useInertialDirection"):
            pitch_error = clamp(float(self.command.get("targetPitch", 0.0)) - pitch, -20, 20)
            roll_error = clamp(signed_degrees(float(self.command.get("targetRoll", 0.0)) - actual_roll), -35, 35)
            qx = cx + roll_error * ppd * 0.45
            qy = cy - pitch_error * ppd
            diamond = QPainterPath(QPointF(qx, qy - 8))
            diamond.lineTo(qx + 8, qy)
            diamond.lineTo(qx, qy + 8)
            diamond.lineTo(qx - 8, qy)
            diamond.closeSubpath()
            painter.setPen(QPen(QColor(205, 108, 255), 1.7))
            painter.drawPath(diamond)

        mono = QFont("Menlo", 12)
        mono.setBold(True)
        painter.setFont(mono)

        # Glass-style readout panels keep important values legible over the horizon.
        panel_fill = QColor(6, 12, 18, 185)
        panel_edge = QColor(170, 190, 205, 85)
        painter.setBrush(QBrush(panel_fill))
        painter.setPen(QPen(panel_edge, 1.0))
        painter.drawRoundedRect(QRectF(10, cy - 28, 104, 55), 5, 5)
        painter.drawRoundedRect(QRectF(width - 114, cy - 28, 104, 55), 5, 5)
        painter.drawRoundedRect(QRectF(cx - 64, 7, 128, 30), 5, 5)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        painter.setPen(QColor(242, 244, 250))
        painter.drawText(QRectF(12, cy - 22, 100, 28), Qt.AlignmentFlag.AlignCenter, f"{float(self.telemetry.get('trueAirSpeed', 0.0)):.0f}")
        painter.drawText(QRectF(width - 112, cy - 22, 100, 28), Qt.AlignmentFlag.AlignCenter, f"{float(self.telemetry.get('radarAltitude', 0.0)):.0f}")
        small = QFont("Menlo", 8)
        painter.setFont(small)
        painter.setPen(QColor(210, 214, 224, 200))
        painter.drawText(QRectF(12, cy + 9, 100, 18), Qt.AlignmentFlag.AlignCenter, "TAS  m/s")
        painter.drawText(QRectF(width - 112, cy + 9, 100, 18), Qt.AlignmentFlag.AlignCenter, "RADAR  m")

        painter.setFont(QFont("Menlo", 11, QFont.Weight.DemiBold))
        painter.setPen(QColor(245, 246, 250))
        painter.drawText(QRectF(cx - 60, 10, 120, 22), Qt.AlignmentFlag.AlignCenter, f"{float(self.telemetry.get('heading', 0.0)):03.0f}°")
        painter.setFont(QFont("Menlo", 8))
        painter.setPen(QColor(215, 220, 230, 200))
        painter.drawText(QRectF(16, 12, 130, 18), Qt.AlignmentFlag.AlignLeft, f"M {float(self.telemetry.get('mach', 0.0)):.2f}")
        painter.drawText(QRectF(width - 160, 12, 144, 18), Qt.AlignmentFlag.AlignRight, f"q {float(self.telemetry.get('dynamicPressure', 0.0)) / 1000:.1f} kPa")
        painter.setPen(QColor(55, 225, 116, 220))
        mode = str(self.telemetry.get("navballSpeedMode", "unchanged")).upper()
        painter.drawText(QRectF(cx - 60, 34, 120, 18), Qt.AlignmentFlag.AlignCenter, mode)

        guidance = "AUTOPILOT  ACTIVE" if self.command.get("autopilotEngaged") else "MANUAL  FLIGHT"
        guidance_color = QColor(55, 225, 116) if self.command.get("autopilotEngaged") else QColor(245, 210, 71)
        status_box = QRectF(cx - 100, height - 30, 200, 22)
        painter.setBrush(QBrush(QColor(6, 12, 18, 205)))
        painter.setPen(QPen(guidance_color, 1.0))
        painter.drawRoundedRect(status_box, 4, 4)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(guidance_color)
        painter.drawText(status_box, Qt.AlignmentFlag.AlignCenter, guidance)


class LocalTrajectoryMap(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.site: dict[str, Any] = {}
        self.snapshot: dict[str, Any] = {}
        self.planet_radius = 600_000.0
        self.manual_range = 0.0
        self.setMinimumSize(220, 160)

    def set_state(self, site: dict[str, Any], snapshot: dict[str, Any], manual_range: float = 0.0) -> None:
        self.site = site or {}
        self.snapshot = snapshot or {}
        self.manual_range = manual_range
        self.update()

    def _point(self, trajectory_point: dict[str, Any]) -> dict[str, Any]:
        return {
            "latitude": trajectory_point.get("latitude", 0.0),
            "longitude": trajectory_point.get("longitude", 0.0),
        }

    def paintEvent(self, _event) -> None:  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), self.palette().alternateBase())
        snapshot = self.snapshot
        actual = snapshot.get("actualTrajectory", []) or []
        reference = snapshot.get("referenceTrajectory", []) or []
        plan = snapshot.get("plan") or {}
        planned = plan.get("trajectory", []) or []
        all_points = list(actual) + list(planned) + list(reference)
        offsets = [local_offsets(self.site, self._point(p), self.planet_radius) for p in all_points]
        telemetry = snapshot.get("telemetry", {}) or {}
        current = {"latitude": telemetry.get("latitude", 0.0), "longitude": telemetry.get("longitude", 0.0)}
        current_offset = local_offsets(self.site, current, self.planet_radius)
        auto_max = max(
            12_000.0,
            min(800_000.0, max((max(abs(e), abs(n)) for e, n in offsets), default=0.0) * 1.15),
            max(abs(current_offset[0]), abs(current_offset[1])) * 1.15,
        )
        maximum = clamp(self.manual_range, 10_000, 800_000) if self.manual_range > 0 else auto_max
        scale = min(self.width(), self.height()) * 0.44 / max(maximum, 1.0)
        cx, cy = self.width() / 2, self.height() / 2

        def mapped(offset: tuple[float, float]) -> QPointF:
            return QPointF(cx + offset[0] * scale, cy - offset[1] * scale)

        step = 2_000 if maximum < 20_000 else 10_000 if maximum < 60_000 else 25_000 if maximum < 150_000 else 50_000
        painter.setPen(QPen(self.palette().mid().color(), 0.7))
        value = -maximum
        while value <= maximum:
            x = cx + value * scale
            y = cy - value * scale
            painter.drawLine(QPointF(x, 0), QPointF(x, self.height()))
            painter.drawLine(QPointF(0, y), QPointF(self.width(), y))
            value += step
        axis_pen = QPen(self.palette().mid().color(), 1.1)
        painter.setPen(axis_pen)
        painter.drawLine(QPointF(cx, 0), QPointF(cx, self.height()))
        painter.drawLine(QPointF(0, cy), QPointF(self.width(), cy))

        heading = math.radians(float(self.site.get("runwayHeading", 90.0)))
        half = float(self.site.get("runwayLength", 2500.0)) * 0.5 * scale
        along = (math.sin(heading), -math.cos(heading))
        painter.setPen(QPen(self.palette().text().color(), max(2.0, float(self.site.get("runwayWidth", 70.0)) * scale)))
        painter.drawLine(QPointF(cx - along[0] * half, cy - along[1] * half), QPointF(cx + along[0] * half, cy + along[1] * half))

        self._draw_path(painter, planned, QColor(245, 158, 55), scale, cx, cy, dashed=True, width=1.5)
        self._draw_path(painter, reference, QColor(200, 204, 214, 180), scale, cx, cy, dashed=True, width=1.2)
        self._draw_path(painter, actual, QColor(50, 200, 225), scale, cx, cy, dashed=False, width=2.1)

        craft = mapped(current_offset)
        marker = QPainterPath(QPointF(craft.x(), craft.y() - 8))
        marker.lineTo(craft.x() + 6, craft.y() + 6)
        marker.lineTo(craft.x() - 6, craft.y() + 6)
        marker.closeSubpath()
        painter.fillPath(marker, QColor(55, 225, 116))
        painter.setFont(QFont("Menlo", 8))
        painter.setPen(self.palette().text().color())
        label = f"LOCAL MAP  ±{maximum / 1000:.0f} km" if maximum >= 100_000 else f"LOCAL MAP  ±{maximum / 1000:.1f} km"
        painter.drawText(QRectF(8, 8, 190, 20), Qt.AlignmentFlag.AlignLeft, label)

    def _draw_path(self, painter: QPainter, points: Iterable[dict[str, Any]], color: QColor, scale: float, cx: float, cy: float, dashed: bool, width: float) -> None:
        points = list(points)
        if not points:
            return
        path = QPainterPath()
        first = local_offsets(self.site, self._point(points[0]), self.planet_radius)
        path.moveTo(cx + first[0] * scale, cy - first[1] * scale)
        for point in points[1:]:
            east, north = local_offsets(self.site, self._point(point), self.planet_radius)
            path.lineTo(cx + east * scale, cy - north * scale)
        pen = QPen(color, width)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        if dashed:
            pen.setStyle(Qt.PenStyle.DashLine)
        painter.setPen(pen)
        painter.drawPath(path)


class TrajectoryProfile(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.site: dict[str, Any] = {}
        self.snapshot: dict[str, Any] = {}
        self.planet_radius = 600_000.0
        self.setMinimumSize(220, 160)

    def set_state(self, site: dict[str, Any], snapshot: dict[str, Any]) -> None:
        self.site = site or {}
        self.snapshot = snapshot or {}
        self.update()

    def paintEvent(self, _event) -> None:  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), self.palette().alternateBase())
        actual = self.snapshot.get("actualTrajectory", []) or []
        plan = self.snapshot.get("plan") or {}
        planned = plan.get("trajectory", []) or []
        all_points = list(actual) + list(planned)
        max_range = max(10_000.0, max((great_circle_distance(p, self.site, self.planet_radius) for p in all_points), default=10_000.0))
        max_alt = max(10_000.0, max((float(p.get("altitude", 0.0)) for p in all_points), default=10_000.0))
        plot = QRectF(34, 14, max(40, self.width() - 46), max(40, self.height() - 46))
        painter.setPen(QPen(self.palette().mid().color(), 1.0))
        painter.drawLine(plot.topLeft(), plot.bottomLeft())
        painter.drawLine(plot.bottomLeft(), plot.bottomRight())
        self._draw_profile(painter, planned, plot, max_range, max_alt, QColor(245, 158, 55), True)
        self._draw_profile(painter, actual, plot, max_range, max_alt, QColor(50, 200, 225), False)
        painter.setFont(QFont("Menlo", 8))
        painter.setPen(self.palette().text().color())
        painter.drawText(QRectF(plot.left(), plot.top(), 160, 18), Qt.AlignmentFlag.AlignLeft, "RANGE / ALTITUDE")
        painter.drawText(QRectF(plot.left(), plot.bottom() + 5, 100, 18), Qt.AlignmentFlag.AlignLeft, f"{max_range / 1000:.0f} km")
        painter.drawText(QRectF(0, plot.top(), 31, 18), Qt.AlignmentFlag.AlignRight, f"{max_alt / 1000:.0f}k")

    def _draw_profile(self, painter: QPainter, points: list[dict[str, Any]], plot: QRectF, max_range: float, max_alt: float, color: QColor, dashed: bool) -> None:
        if not points:
            return

        def mapped(point: dict[str, Any]) -> QPointF:
            range_value = great_circle_distance(point, self.site, self.planet_radius)
            x = plot.right() - clamp(range_value / max_range, 0, 1) * plot.width()
            y = plot.bottom() - clamp(float(point.get("altitude", 0.0)) / max_alt, 0, 1) * plot.height()
            return QPointF(x, y)

        path = QPainterPath(mapped(points[0]))
        for point in points[1:]:
            path.lineTo(mapped(point))
        pen = QPen(color, 1.8)
        if dashed:
            pen.setStyle(Qt.PenStyle.DashLine)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        painter.setPen(pen)
        painter.drawPath(path)
