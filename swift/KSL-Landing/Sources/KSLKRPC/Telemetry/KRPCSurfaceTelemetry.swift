import Foundation
import KSLCore

/// Optional, read-only standard SpaceCenter sensors. No setters, overrides or
/// actuation enter this inventory. Unsupported servers retain nil evidence.
struct KRPCSurfaceTelemetry: Sendable {
  struct Surface: Sendable {
    let handle: UInt64, part: UInt64
    let name: String
  }
  enum Property: String, CaseIterable, Sendable {
    case angle = "DeflectionAngle", range = "ControlRange", speed = "ActuatorSpeed"
    case authority = "AuthorityLimiter", deployed = "Deployed", deployAngle = "DeployAngle"
    case pitch = "PitchEnabled", roll = "RollEnabled", yaw = "YawEnabled"
    var procedure: String { "ControlSurface_get_" + rawValue }
  }
  let surfaces: [Surface], properties: [Property]
  static func supportedProperties(services: Data) -> [Property] {
    guard services.range(of: Data("Parts_get_ControlSurfaces".utf8)) != nil,
      services.range(of: Data("ControlSurface_get_Part".utf8)) != nil,
      services.range(of: Data(Property.angle.procedure.utf8)) != nil else { return [] }
    return Property.allCases.filter { services.range(of: Data($0.procedure.utf8)) != nil }
  }
  static func discover(session: KRPCSession, parts: UInt64, properties: [Property]) async throws -> Self? {
    guard properties.contains(.angle) else { return nil }
    let handles = try KRPCValue.decodeTuple(try await session.call([
      .init(procedure: "Parts_get_ControlSurfaces", arguments: [KRPCValue.uint(parts)])])[0])
      .map(KRPCValue.decodeUInt)
    guard !handles.isEmpty else { return .init(surfaces: [], properties: properties) }
    let partHandles = try await session.call(handles.map {
      .init(procedure: "ControlSurface_get_Part", arguments: [KRPCValue.uint($0)]) })
      .map(KRPCValue.decodeUInt)
    let names = try await session.call(partHandles.map {
      .init(procedure: "Part_get_Name", arguments: [KRPCValue.uint($0)]) })
      .map(KRPCValue.decodeString)
    return .init(surfaces: handles.indices.map {
      .init(handle: handles[$0], part: partHandles[$0], name: names[$0]) }, properties: properties)
  }
  var calls: [KRPCCall] {
    surfaces.flatMap { surface in properties.map {
      .init(procedure: $0.procedure, arguments: [KRPCValue.uint(surface.handle)]) } }
  }
  func decode(_ values: ArraySlice<Data>, time: Double) -> (observations: [ControlSurfaceObservation]?, issues: [String]) {
    guard properties.contains(.angle), values.count == surfaces.count*properties.count else {
      return (nil, ["Actual control-surface telemetry response count mismatch"])
    }
    var observations: [ControlSurfaceObservation] = [], issues: [String] = []
    for (index, surface) in surfaces.enumerated() {
      let fields = Dictionary(uniqueKeysWithValues: properties.enumerated().map {
        ($0.element, values[values.startIndex+index*properties.count+$0.offset]) })
      do {
        func number(_ property: Property) throws -> Double? {
          try fields[property].map { Double(try KRPCValue.decodeFloat($0)) }
        }
        func angle(_ property: Property) throws -> Double? { try number(property).map(radians) }
        func boolean(_ property: Property) throws -> Bool? { try fields[property].map(KRPCValue.decodeBool) }
        guard let deflection = try angle(.angle) else { continue }
        let observation = try ControlSurfaceObservation(surfaceID: surface.handle, partID: surface.part,
          partName: surface.name, measuredAt: time, deflectionAngle: deflection,
          controlRange: angle(.range), actuatorSpeed: angle(.speed), authorityFraction: number(.authority).map { $0/100 },
          deployed: boolean(.deployed), deployAngle: angle(.deployAngle),
          pitchEnabled: boolean(.pitch), rollEnabled: boolean(.roll), yawEnabled: boolean(.yaw))
        guard observation.isCoherent else { throw FlightError.invalidTelemetry("Nonfinite/out-of-range surface data") }
        observations.append(observation)
      } catch {
        // Optional sensor failure does not discard fresh coherent kinematics.
        // Omit this sample; never carry forward a last-known actual angle.
        issues.append("Actual control-surface telemetry unavailable: \(surface.name) handle=\(surface.handle): \(error)")
      }
    }
    return (observations.isEmpty && !surfaces.isEmpty ? nil : observations, issues)
  }
}
