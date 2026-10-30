import Foundation
import KSLCore

struct KRPCWheelInventory: Sendable {
  var mainFields: [UInt64] = []
  var noseFields: [UInt64] = []
  static func discover(session: KRPCSession, parts: [UInt64]) async throws -> Self {
    func call(_ name: String, _ object: UInt64) -> KRPCCall {
      .init(procedure: name, arguments: [KRPCValue.uint(object)])
    }
    var result = Self()
    let titles = try await session.call(parts.map { call("Part_get_Title", $0) })
    for (part, encodedTitle) in zip(parts, titles) {
      let title = try KRPCValue.decodeString(encodedTitle)
      // These are the STS hardware names used by the historical C adapter.
      // Unknown hardware stays unavailable; its situation is not substituted.
      guard title == "OV-100 Main Gear" || title == "OV-100 Nose Gear" else { continue }
      let modules = try KRPCValue.decodeTuple(
        try await session.call([call("Part_get_Modules", part)])[0]).map(KRPCValue.decodeUInt)
      let names = try await session.call(modules.map { call("Module_get_Name", $0) })
      for (module, name) in zip(modules, names) {
        guard try KRPCValue.decodeString(name) == "ModuleWheelBase" else { continue }
        let fields = try KRPCValue.decodeTuple(
          try await session.call([call("Module_get_FieldList", module)])[0]).map(KRPCValue.decodeUInt)
        let fieldNames = try await session.call(fields.map { call("PartField_get_Name", $0) })
        for (field, fieldName) in zip(fields, fieldNames) {
          if try KRPCValue.decodeString(fieldName) == "isGrounded" {
            if title == "OV-100 Main Gear" { result.mainFields.append(field) }
            else { result.noseFields.append(field) }
          }
        }
      }
    }
    return result
  }
  var calls: [KRPCCall] {
    (mainFields + noseFields).map {
      .init(procedure: "PartField_get_BoolValue", arguments: [KRPCValue.uint($0)])
    }
  }
  func decode(_ values: ArraySlice<Data>) throws -> WheelContactTelemetry {
    let flags = try values.map(KRPCValue.decodeBool)
    guard flags.count == mainFields.count + noseFields.count else {
      throw FlightError.protocolViolation("Wheel contact count mismatch")
    }
    return .init(mainGear: mainFields.isEmpty ? nil : flags.prefix(mainFields.count).contains(true),
                 noseGear: noseFields.isEmpty ? nil : flags.dropFirst(mainFields.count).contains(true),
                 wheelCount: flags.count, evidence: .measured)
  }
}
