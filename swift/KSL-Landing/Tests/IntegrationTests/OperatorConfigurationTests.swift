import Foundation
import KSLCore
import KSLRuntime
import XCTest

final class OperatorConfigurationTests: XCTestCase {
  func testCompleteSTSConfigurationAndEveryLeafDisposition() throws {
    var root = URL(fileURLWithPath: #filePath)
    for _ in 0..<5 { root.deleteLastPathComponent() }
    let data = try Data(contentsOf: root.appendingPathComponent("Configuration/sts.json"))
    let input = try XCTUnwrap(JSONSerialization.jsonObject(with: data) as? [String: [String: Any]])
    let mapped = try OperatorConfigurationMapper.map(data)
    let supplied = Set(input.flatMap { section, leaves in leaves.keys.map { section+"."+$0 } })
    XCTAssertEqual(Set(mapped.dispositions.map(\.field)), supplied)
    XCTAssertEqual(mapped.dispositions.count, supplied.count)
    XCTAssertEqual(mapped.connection.timeout, 10)
    XCTAssertEqual(mapped.mission.runway.elevation, 68.44)
    XCTAssertEqual(mapped.mission.vehicle.landingSpeed, 85)
    XCTAssertEqual(mapped.mission.guidance.finalSpeed, 120)
    XCTAssertEqual(mapped.mission.guidance.taemGlideAngle, radians(24))
    XCTAssertEqual(mapped.mission.guidance.terminalRollRate, radians(6))
    XCTAssertEqual(mapped.mission.guidance.rolloutParachutes, true)
    XCTAssertEqual(mapped.mission.entrySettings.nominalAlpha, radians(18))
    XCTAssertEqual(mapped.mission.guidance.hacAcquisitionAltitude, 10700)
    XCTAssertEqual(mapped.mission.guidance.hacAcquisitionMach, 0.95)
    XCTAssertEqual(mapped.mission.guidance.gearDeploymentAltitude, 1500)
    XCTAssertEqual(mapped.mission.deorbitSettings.maximumThrottle, 0.18)
    XCTAssertEqual(mapped.mission.deorbitSettings.pointingUncertainty, radians(2))
    XCTAssertTrue(mapped.allowReciprocal)
  }
  func testSupportedSTSProfilePreservesFlownEntryAndLandingSettings() throws {
    var root = URL(fileURLWithPath: #filePath)
    for _ in 0..<5 { root.deleteLastPathComponent() }
    let mapped = try OperatorConfigurationMapper.map(Data(contentsOf:
      root.appendingPathComponent("Configuration/sts-taem.json")))
    let flown = try JSONDecoder().decode(MissionConfiguration.self, from: Data(contentsOf:
      root.appendingPathComponent("Configuration/sts-taem-native.json")))
    XCTAssertEqual(mapped.mission.entrySettings, flown.entrySettings)
    XCTAssertEqual(mapped.mission.guidance.finalDistance, flown.guidance.finalDistance)
    XCTAssertEqual(mapped.mission.guidance.outerGlideAngle, flown.guidance.outerGlideAngle)
    XCTAssertEqual(mapped.mission.guidance.flareHeight, flown.guidance.flareHeight)
    XCTAssertEqual(mapped.mission.guidance.touchdownAlong, flown.guidance.touchdownAlong)
    XCTAssertEqual(mapped.mission.guidance.finalSpeed, flown.guidance.finalSpeed)
    XCTAssertEqual(mapped.mission.guidance.terminalReplanInterval, flown.guidance.terminalReplanInterval)
    XCTAssertEqual(mapped.mission.guidance.approachPredictionInterval ?? 1.5, flown.guidance.approachPredictionInterval ?? 1.5)
    XCTAssertEqual(mapped.mission.vehicle.maximumAngleOfAttack, flown.vehicle.maximumAngleOfAttack)
    XCTAssertEqual(mapped.mission.vehicle.maximumBank, flown.vehicle.maximumBank)
    XCTAssertEqual(mapped.mission.vehicle.landingSpeed, flown.vehicle.landingSpeed)
    XCTAssertEqual(mapped.mission.runway.elevation, flown.runway.elevation)
    XCTAssertEqual(mapped.mission.runway.latitude, flown.runway.latitude, accuracy: 1e-12)
    XCTAssertEqual(mapped.mission.runway.longitude, flown.runway.longitude, accuracy: 1e-12)
    XCTAssertNil(mapped.mission.guidance.taemGlideAngle)
    XCTAssertNil(mapped.mission.guidance.finalAlignmentSpeed)
    XCTAssertNil(mapped.mission.vehicle.speedbrakeFullDeploymentDragArea)
    XCTAssertNil(mapped.mission.physicsArchive)
    XCTAssertEqual(mapped.connection.timeout, 10)
  }

  func testUnknownFieldsWrongTypesAndForbiddenPropulsionAreRejected() {
    for json in [#"{"guidance":{"guidanceRate":true}}"#,
      #"{"vehicle":{"allowPoweredApproach":1}}"#,
      #"{"guidance":{"misspelledGain":12}}"#,
      #"{"vehicle":{"maximumApproachThrottle":0.1}}"#,
      #"{"vehicle":{"allowPoweredApproach":true}}"#,
      #"{"connection":{"rpcPort":50000.5}}"#,
      #"{"guidance":{"touchdownSinkRate":1.5}}"#] {
      XCTAssertThrowsError(try OperatorConfigurationMapper.map(Data(json.utf8)), json)
    }
  }
  func testPartialUpdatePreservesExplicitConnectionAndMission() throws {
    let first = try OperatorConfigurationMapper.map(Data(#"{"connection":{"rpcHost":"192.0.2.1","rpcPort":50001},"site":{"altitude":71,"allowReciprocalRunway":true},"vehicle":{"airbrakeActionGroup":3}}"#.utf8))
    let second = try OperatorConfigurationMapper.map(Data(#"{"guidance":{"flareAltitude":65}}"#.utf8),
      base: first.mission, baseConnection: first.connection, baseAirbrakeGroup: first.airbrakeGroup,
      baseAllowReciprocal: first.allowReciprocal)
    XCTAssertEqual(second.connection, first.connection)
    XCTAssertEqual(second.airbrakeGroup, 3)
    XCTAssertTrue(second.allowReciprocal)
    XCTAssertEqual(second.mission.runway.elevation, 71)
    XCTAssertEqual(second.mission.guidance.flareHeight, 65)
  }
}
