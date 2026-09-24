(() => {
  const finite = (value) => value !== null && value !== undefined && value !== "" && Number.isFinite(Number(value));
  const number = (value, fallback = 0) => finite(value) ? Number(value) : fallback;
  const clampPositive = (value, fallback) => {
    const candidate = number(value, fallback);
    return candidate > 0 ? candidate : fallback;
  };
  const normalizedHeading = (value) => ((number(value) % 360) + 360) % 360;

  function build(site, destinationPoint, options = {}) {
    if (!site || !finite(site.latitude) || !finite(site.longitude) ||
        !finite(site.runwayHeading) || typeof destinationPoint !== "function") {
      return null;
    }

    const heading = normalizedHeading(site.runwayHeading);
    const length = clampPositive(site.runwayLength, 2500);
    const width = clampPositive(site.runwayWidth, 70);
    const halfWidth = width / 2;
    const baseAltitude = number(site.altitude, 70);
    const surfaceAltitude = baseAltitude + number(options.surfaceAltitudeOffset, 2);
    const markingAltitude = surfaceAltitude + number(options.markingAltitudeOffset, 1.5);
    const sectionStep = Math.max(25, Math.min(100, number(options.sectionStep, 50)));

    const withAltitude = (point, altitude) => point && ({ ...point, altitude });
    const destination = (origin, bearing, meters, altitude = surfaceAltitude) =>
      withAltitude(destinationPoint(origin, bearing, meters, altitude), altitude);
    const threshold = {
      latitude: number(site.latitude),
      longitude: number(site.longitude),
      altitude: surfaceAltitude,
    };
    const atDistance = (meters, altitude = surfaceAltitude) =>
      destination(threshold, heading, Math.max(0, Math.min(length, meters)), altitude);
    const offset = (base, lateralMeters, altitude = surfaceAltitude) =>
      destination(base, heading + (lateralMeters >= 0 ? 90 : -90), Math.abs(lateralMeters), altitude);

    const sections = [];
    for (let distance = 0; distance < length; distance += sectionStep) {
      const center = atDistance(distance);
      sections.push({
        distance,
        center,
        left: offset(center, -halfWidth),
        right: offset(center, halfWidth),
      });
    }
    const endCenter = atDistance(length);
    sections.push({
      distance: length,
      center: endCenter,
      left: offset(endCenter, -halfWidth),
      right: offset(endCenter, halfWidth),
    });

    const leftEdge = sections.map((section) => section.left);
    const rightEdge = sections.map((section) => section.right);
    const outline = [...leftEdge, ...rightEdge.slice().reverse(), leftEdge[0]];

    const markings = [];
    const addCrossbar = (distance, inset = Math.min(5, width * 0.08)) => {
      const center = atDistance(distance, markingAltitude);
      const half = Math.max(2, halfWidth - inset);
      markings.push(offset(center, -half, markingAltitude), offset(center, half, markingAltitude));
    };
    const addCenterDash = (startDistance, endDistance) => {
      markings.push(atDistance(startDistance, markingAltitude), atDistance(endDistance, markingAltitude));
    };

    // Keep the physical dimensions exact. Paint spacing is purely a visual
    // representation of the stock KSC runway markings.
    addCrossbar(Math.min(22, length * 0.02));
    addCrossbar(Math.max(0, length - Math.min(22, length * 0.02)));

    const paintMargin = Math.min(90, length * 0.08);
    const dashLength = Math.max(18, Math.min(36, length / 60));
    const dashGap = dashLength;
    for (let start = paintMargin; start + dashLength <= length - paintMargin; start += dashLength + dashGap) {
      addCenterDash(start, start + dashLength);
    }

    const aimingDistance = Math.min(420, length * 0.22);
    for (const distance of [aimingDistance, length - aimingDistance]) {
      if (distance <= paintMargin || distance >= length - paintMargin) continue;
      const center = atDistance(distance, markingAltitude);
      const barHalf = Math.max(8, halfWidth * 0.72);
      markings.push(offset(center, -barHalf, markingAltitude), offset(center, -barHalf * 0.35, markingAltitude));
      markings.push(offset(center, barHalf * 0.35, markingAltitude), offset(center, barHalf, markingAltitude));
    }

    const guidanceBack = Math.max(0, number(options.guidanceBack, 18000));
    const guidanceForward = Math.max(length, number(options.guidanceForward, 32000));
    const guidanceAxis = [
      destination(threshold, heading + 180, guidanceBack, markingAltitude),
      { ...threshold, altitude: markingAltitude },
      destination(threshold, heading, guidanceForward, markingAltitude),
    ];

    return {
      heading,
      length,
      width,
      threshold,
      end: endCenter,
      sections,
      outline,
      leftEdge,
      rightEdge,
      markings,
      guidanceAxis,
    };
  }

  window.ShuttleRunwayGeometry = Object.freeze({ build });
})();
