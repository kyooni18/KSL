import Foundation
import KSLCore

/// OV-100 ModuleControlSurface deployment fields. deployAngle is the requested
/// deployment state, not proof that the actuator has reached that angle.
struct STSSurfaceActuators: Sendable {
  struct Channel: Sendable {
    let deploy: UInt64, angle: UInt64, invert: UInt64
    // kRPC ControlSurface represents only the first module on a split fin.
    // Each rudder half must independently ignore pitch and roll, not yaw.
    let ignorePitch: UInt64?, ignoreRoll: UInt64?, ignoreYaw: UInt64?
    let rangeDegrees: Double
  }
  var rudder: [Channel] = []
  var bodyFlap: Channel?
  private(set) var lastBrake = 0.0, lastFlap = 0.0
  private var commandCacheValid = false

  private static func call(_ name: String, _ object: UInt64) -> KRPCCall {
    .init(procedure: name, arguments: [KRPCValue.uint(object)])
  }
  private static func set(_ name: String, _ field: UInt64, _ value: Data) -> KRPCCall {
    .init(procedure: name, arguments: [KRPCValue.uint(field), value])
  }

  static func discover(session: KRPCSession, parts: [UInt64]) async throws -> Self {
    var result = Self()
    let names = try await session.call(parts.map { call("Part_get_Name", $0) })
    for (part, encoded) in zip(parts, names) {
      let name = try KRPCValue.decodeString(encoded).lowercased().replacingOccurrences(of: "_", with: ".")
      let isRudder = name == "benjee10.shuttle.rudder"
      let isFlap = name == "benjee10.shuttle.bodyflap"
      guard isRudder || isFlap else { continue }
      let modules = try KRPCValue.decodeTuple(try await session.call([call("Part_get_Modules", part)])[0])
        .map(KRPCValue.decodeUInt)
      let moduleNames = try await session.call(modules.map { call("Module_get_Name", $0) })
      for (module, encodedModule) in zip(modules, moduleNames) {
        guard try KRPCValue.decodeString(encodedModule) == "ModuleControlSurface" else { continue }
        let fields = try KRPCValue.decodeTuple(try await session.call([call("Module_get_FieldList", module)])[0])
          .map(KRPCValue.decodeUInt)
        let fieldNames = try await session.call(fields.map { call("PartField_get_Name", $0) })
        var handles: [String: UInt64] = [:]
        for (field, encodedName) in zip(fields, fieldNames) {
          handles[try KRPCValue.decodeString(encodedName)] = field
        }
        guard let deploy = handles["deploy"], let angle = handles["deployAngle"],
          let invert = handles["deployInvert"], let range = handles["ctrlSurfaceRange"] else { continue }
        let degrees = Double(try KRPCValue.decodeFloat(try await session.call([
          call("PartField_get_FloatValue", range)
        ])[0]))
        guard degrees.isFinite, degrees > 0 else { continue }
        let channel = Channel(deploy: deploy, angle: angle, invert: invert,
          ignorePitch: handles["ignorePitch"], ignoreRoll: handles["ignoreRoll"],
          ignoreYaw: handles["ignoreYaw"], rangeDegrees: degrees)
        if isRudder {
          guard channel.ignorePitch != nil, channel.ignoreRoll != nil, channel.ignoreYaw != nil else {
            throw FlightError.invalidTelemetry("Split-rudder module lacks individual axis fields")
          }
          result.rudder.append(channel)
        } else {
          guard result.bodyFlap == nil else {
            throw FlightError.invalidTelemetry("Ambiguous OV-100 body-flap deployment modules")
          }
          result.bodyFlap = channel
        }
      }
    }
    return result
  }

  var telemetryCalls: [KRPCCall] {
    (rudder + [bodyFlap].compactMap { $0 }).flatMap {
      [Self.call("PartField_get_FloatValue", $0.angle), Self.call("PartField_get_BoolValue", $0.deploy)]
    }
  }
  func decodeFractions(_ values: ArraySlice<Data>) throws -> (brake: Double?, flap: Double?) {
    let channels = rudder + [bodyFlap].compactMap { $0 }
    guard values.count == 2*channels.count else {
      throw FlightError.protocolViolation("Surface deployment response count mismatch")
    }
    var fractions: [Double] = []
    for (i, channel) in channels.enumerated() {
      let angle = Double(try KRPCValue.decodeFloat(values[values.startIndex+2*i]))
      let deployed = try KRPCValue.decodeBool(values[values.startIndex+2*i+1])
      fractions.append(deployed ? angle/channel.rangeDegrees : 0)
    }
    let brake = rudder.count == 2 ? fractions.prefix(2).map(abs).min() : nil
    if rudder.count == 2 {
      let halves = fractions.prefix(2).map(abs)
      guard halves.allSatisfy({ $0.isFinite && (0...1).contains($0) }),
        abs(halves[0]-halves[1]) <= 1e-4 else {
        throw FlightError.invalidTelemetry("Split-rudder deployment is asymmetric or outside its configured range")
      }
    }
    let flap = bodyFlap == nil ? nil : fractions.last
    return (brake, flap)
  }

  mutating func armCalls() -> [KRPCCall] {
    var calls: [KRPCCall] = []
    // The installed split-rudder requires opposite deployment inversions and
    // independent yaw-only control on BOTH ModuleControlSurface halves. kRPC's
    // ControlSurface API only updates the first module; write the second here.
    if rudder.count == 2 {
      for (i, desired) in [false, true].enumerated() {
        let channel = rudder[i]
        calls.append(Self.set("PartField_set_BoolValue", channel.invert, KRPCValue.bool(desired)))
        if let pitch = channel.ignorePitch, let roll = channel.ignoreRoll, let yaw = channel.ignoreYaw {
          calls.append(Self.set("PartField_set_BoolValue", pitch, KRPCValue.bool(true)))
          calls.append(Self.set("PartField_set_BoolValue", roll, KRPCValue.bool(true)))
          calls.append(Self.set("PartField_set_BoolValue", yaw, KRPCValue.bool(false)))
        }
      }
    }
    calls += deploymentCalls(brake: 0, flap: 0)
    lastBrake = 0; lastFlap = 0; commandCacheValid = true
    return calls
  }
  func verifySplitGeometry(session: KRPCSession) async throws {
    guard rudder.count == 2 else {
      throw FlightError.invalidTelemetry("OV-100 split rudder must provide exactly two control modules")
    }
    for (i, channel) in rudder.enumerated() {
      guard let pitch = channel.ignorePitch, let roll = channel.ignoreRoll,
        let yaw = channel.ignoreYaw else {
        throw FlightError.invalidTelemetry("Split-rudder axis fields disappeared")
      }
      let values = try await session.call([channel.invert, pitch, roll, yaw].map {
        Self.call("PartField_get_BoolValue", $0)
      })
      let actual = try values.map(KRPCValue.decodeBool)
      guard actual == [i == 1, true, true, false] else {
        throw FlightError.invalidTelemetry("Split-rudder inversion/yaw-only allocation failed read-back")
      }
    }
  }
  private func deploymentCalls(brake: Double, flap: Double) -> [KRPCCall] {
    var calls: [KRPCCall] = []
    if rudder.count == 2 {
      let angle = brake * min(rudder[0].rangeDegrees, rudder[1].rangeDegrees)
      for channel in rudder {
        calls.append(Self.set("PartField_set_FloatValue", channel.angle, KRPCValue.float(Float(angle))))
        calls.append(Self.set("PartField_set_BoolValue", channel.deploy, KRPCValue.bool(brake > 0)))
      }
    }
    if let channel = bodyFlap {
      calls.append(Self.set("PartField_set_FloatValue", channel.angle, KRPCValue.float(Float(flap*channel.rangeDegrees))))
      calls.append(Self.set("PartField_set_BoolValue", channel.deploy, KRPCValue.bool(flap != 0)))
    }
    return calls
  }
  mutating func invalidateCachedCommands() { commandCacheValid = false }
  mutating func commandCalls(brake: Double, flap: Double) throws -> [KRPCCall] {
    guard brake.isFinite, (0...1).contains(brake), flap.isFinite, abs(flap) <= 1 else {
      throw FlightError.invalidCommand("Nonfinite or out-of-range surface deployment")
    }
    guard brake == 0 || rudder.count == 2 else {
      throw FlightError.invalidCommand("Split-rudder speedbrake was requested without two verified deployment channels")
    }
    guard flap == 0 || bodyFlap != nil else {
      throw FlightError.invalidCommand("Body-flap trim was requested without its verified deployment channel")
    }
    // Build one consistent full-state deployment batch. A transport failure
    // can leave a partial kRPC write; invalidate the cache and reassert BOTH
    // surfaces on the next accepted command, even if targets are unchanged.
    guard !commandCacheValid || abs(brake-lastBrake) > 1e-9 || abs(flap-lastFlap) > 1e-9 else { return [] }
    let calls = deploymentCalls(brake: brake, flap: flap)
    lastBrake = brake; lastFlap = flap; commandCacheValid = true
    return calls
  }
}
