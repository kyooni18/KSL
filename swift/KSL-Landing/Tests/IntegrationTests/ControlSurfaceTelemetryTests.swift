import Foundation
import XCTest
import KSLCore
@testable import KSLKRPC

final class ControlSurfaceTelemetryTests: XCTestCase {
  func testOnlyActualAngleCapabilityEnablesReadOnlyObservationAndUnitsRemainExplicit() throws {
    let legacy = Data("Parts_get_ControlSurfaces ControlSurface_get_Part ControlSurface_get_Deflection".utf8)
    XCTAssertTrue(KRPCSurfaceTelemetry.supportedProperties(services: legacy).isEmpty,
      "An override command is not actual deflection telemetry")
    let services = Data((String(decoding: legacy, as: UTF8.self) + " " +
      KRPCSurfaceTelemetry.Property.allCases.map(\.procedure).joined(separator: " ")).utf8)
    let properties = KRPCSurfaceTelemetry.supportedProperties(services: services)
    let inventory = KRPCSurfaceTelemetry(surfaces: [.init(handle: 8, part: 5, name: "benjee10.shuttle.bodyFlap")],
      properties: properties)
    XCTAssertTrue(inventory.calls.allSatisfy { $0.procedure.contains("_get_") })
    let encoded: [KRPCSurfaceTelemetry.Property: Data] = [
      .angle: KRPCValue.float(7), .range: KRPCValue.float(30), .speed: KRPCValue.float(15),
      .authority: KRPCValue.float(75), .deployed: KRPCValue.bool(true), .deployAngle: KRPCValue.float(10),
      .pitch: KRPCValue.bool(true), .roll: KRPCValue.bool(false), .yaw: KRPCValue.bool(false)]
    let result = inventory.decode(ArraySlice(properties.map { encoded[$0]! }), time: 100)
    let sample = try XCTUnwrap(result.observations?.first)
    XCTAssertTrue(result.issues.isEmpty)
    XCTAssertEqual(sample.deflectionAngle, radians(7))
    XCTAssertEqual(sample.deployAngle, radians(10), "Request and actual angle must remain separate")
    XCTAssertEqual(sample.controlRange, radians(30))
    XCTAssertEqual(sample.actuatorSpeed, radians(15))
    XCTAssertEqual(sample.authorityFraction, 0.75)
    XCTAssertEqual(sample.pitchEnabled, true)
    XCTAssertEqual(sample.evidence, .measured)
    XCTAssertEqual(try JSONDecoder().decode(ControlSurfaceObservation.self,
      from: JSONEncoder().encode(sample)), sample)
    var bad = encoded
    bad[.angle] = KRPCValue.float(.nan)
    let missing = inventory.decode(ArraySlice(properties.map { bad[$0]! }), time: 101)
    XCTAssertNil(missing.observations)
    XCTAssertEqual(missing.issues.count, 1)
    XCTAssertTrue(inventory.decode(ArraySlice<Data>(), time: 101).observations == nil)
  }
}
