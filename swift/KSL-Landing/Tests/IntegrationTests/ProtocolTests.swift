import Foundation
import KSLCore
import Testing

@testable import KSLKRPC

private final class ScriptedTransport: RPCTransport {
  private var input: Data
  init(_ payloads: [Data]) {
    input = Data()
    for body in payloads {
      input.append(Protobuf.varint(UInt64(body.count)))
      input.append(body)
    }
  }
  func write(_ data: Data, deadline: ContinuousClock.Instant) throws {
    try Task.checkCancellation()
  }
  func read(count: Int, deadline: ContinuousClock.Instant) throws -> Data {
    try Task.checkCancellation()
    guard input.count >= count else { throw FlightError.disconnected("scripted EOF") }
    let bytes = Data(input.prefix(count))
    input = Data(input.dropFirst(count))
    return bytes
  }
  func close() { input.removeAll() }
}

struct ProtocolTests {
  @Test func testAnchorOffsetIsBodyRelativeAndInvariantToWorldAttitude() {
    let offset = Vector3(1, 2, 3)
    let part = Quaternion.axisAngle(.init(0, 1, 0), angle: radians(15))
    let vessel = Quaternion.axisAngle(.init(1, 0, 0), angle: radians(-20))
    let world = Quaternion.axisAngle(.init(0, 0, 1), angle: radians(70))
    let before = KRPCFrameConverter.anchorOffsetFRD(offset, partRotation: part, vesselRotation: vessel)
    let after = KRPCFrameConverter.anchorOffsetFRD(offset,
      partRotation: world*part, vesselRotation: world*vessel)
    #expect((after-before).length < 1e-12)
    #expect(abs(before.length-offset.length) < 1e-12)
    #expect(KRPCFrameConverter.anchorOffsetFRD(offset, partRotation: .init(), vesselRotation: .init())
      == .init(2, 1, 3))
  }

  @Test func testSTSSurfaceRolesKeepPitchRollAndYawSeparate() {
    #expect(STSSurfaceAllocation.axes(partName: "benjee10.shuttle.elevon2") == [true, false, false])
    #expect(STSSurfaceAllocation.axes(partName: "benjee10_shuttle_elevon1") == [false, true, false])
    #expect(STSSurfaceAllocation.axes(partName: "benjee10.shuttle.bodyFlap") == [true, false, false])
    #expect(STSSurfaceAllocation.axes(partName: "benjee10.shuttle.rudder") == [false, false, true])
    #expect(STSSurfaceAllocation.axes(partName: "unrelated.part") == nil)
  }
  @Test func testRailsWarpWireRejectsInvalidIndices() throws {
    for value in UInt64(0)...UInt64(7) {
      let encoded = try KRPCValue.encodeRailsWarpFactor(value)
      #expect(try KRPCValue.decodeRailsWarpFactor(encoded) == value)
    }
    #expect(throws: (any Error).self) {
      try KRPCValue.encodeRailsWarpFactor(8)
    }
    #expect(throws: (any Error).self) {
      try KRPCValue.decodeRailsWarpFactor(KRPCValue.sint(-1))
    }
  }

  @Test func testScalarAndTupleGoldenVectors() throws {
    #expect(Array(KRPCValue.double(1)) == [0, 0, 0, 0, 0, 0, 240, 63])
    #expect(Array(KRPCValue.float(1)) == [0, 0, 128, 63])
    #expect(Array(KRPCValue.sint(-1)) == [1])
    #expect(Array(KRPCValue.sint(5)) == [10])
    // kRPC SpaceCenter rails warp factor is sint32: raw uint gave 4 -> 2.
    #expect(Array(try KRPCValue.encodeRailsWarpFactor(4)) == [8])
    #expect(Array(try KRPCValue.encodeRailsWarpFactor(3)) == [6])
    #expect(try KRPCValue.decodeRailsWarpFactor(Data([6])) == 3)
    #expect(try KRPCValue.decodeRailsWarpFactor(Data([8])) == 4)
    #expect(try KRPCValue.decodeUInt(Data([6])) == 6)
    #expect(Array(KRPCValue.string("STS")) == [3, 83, 84, 83])
    let value = Vector3(1, -2, 3)
    let data = KRPCValue.tuple([KRPCValue.double(1), KRPCValue.double(-2), KRPCValue.double(3)])
    #expect(try KRPCValue.decodeVector(data) == value)
  }
  @Test func testMalformedWireIsRejected() {
    for bytes: [UInt8] in [
      [0], [10, 255], [0x08] + Array(repeating: 0xff, count: 10), [13, 0, 0], [0x0b],
    ] {
      #expect(throws: (any Error).self) { try Protobuf.decode(Data(bytes)) }
    }
  }
  @Test func testNativeHandshakeAndBatchedResponse() async throws {
    let handshake = Protobuf.bytesField(3, Data([1, 2, 3, 4]))
    let body = Protobuf.bytesField(2, Protobuf.bytesField(2, KRPCValue.double(42)))
    let session = KRPCSession(transport: ScriptedTransport([handshake, body]))
    try await session.connect()
    let values = try await session.call([.init(procedure: "get_UT")])
    #expect(try KRPCValue.decodeDouble(values[0]) == 42)
    let count = await session.requestCount
    #expect(count == 1)
    await session.close()
    do {
      _ = try await session.call([.init(procedure: "get_UT")])
      Issue.record("closed session accepted RPC")
    } catch {}
  }
  @Test func testFrameHandednessAndQuaternionRate() {
    let p = Vector3(600_000, 0, 0)
    #expect(KRPCFrameConverter.surfaceVector(.init(1, 2, 3), at: p) == Vector3(1, 3, 2))
    let previous = Quaternion()
    let current = previous.advanced(bodyRate: .init(0, 0.1, 0), dt: 0.1)
    #expect(
      abs(
        KRPCFrameConverter.measuredBodyRate(previous: previous, current: current, dt: 0.1).y - 0.1)
        < 1e-10)
  }
}
