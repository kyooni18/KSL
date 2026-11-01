import Foundation
import KSLCore

public struct KRPCCall: Sendable {
  public let service: String
  public let procedure: String
  public let arguments: [Data]
  public init(service: String = "SpaceCenter", procedure: String, arguments: [Data] = []) {
    self.service = service
    self.procedure = procedure
    self.arguments = arguments
  }
  public func encoded() -> Data {
    var data = Protobuf.stringField(1, service)
    data.append(Protobuf.stringField(2, procedure))
    for (i, value) in arguments.enumerated() {
      var argument = Protobuf.integerField(1, UInt64(i))
      argument.append(Protobuf.bytesField(2, value))
      data.append(Protobuf.bytesField(3, argument))
    }
    return data
  }
}

public enum KRPCValue {
  public static func uint(_ value: UInt64) -> Data { Protobuf.varint(value) }
  public static func bool(_ value: Bool) -> Data { uint(value ? 1 : 0) }
  public static func sint(_ value: Int64) -> Data {
    uint(UInt64(bitPattern: (value << 1) ^ (value >> 63)))
  }
  public static func float(_ value: Float) -> Data {
    let bits = value.bitPattern
    return Data((0..<4).map { UInt8(truncatingIfNeeded: bits >> (8 * $0)) })
  }
  public static func double(_ value: Double) -> Data {
    let bits = value.bitPattern
    return Data((0..<8).map { UInt8(truncatingIfNeeded: bits >> (8 * $0)) })
  }
  public static func string(_ value: String) -> Data {
    let bytes = Data(value.utf8)
    var data = uint(UInt64(bytes.count))
    data.append(bytes)
    return data
  }
  public static func tuple(_ values: [Data]) -> Data {
    var data = Data()
    for value in values { data.append(Protobuf.bytesField(1, value)) }
    return data
  }
  public static func decodeUInt(_ data: Data) throws -> UInt64 {
    var index = 0
    let value = try Protobuf.readVarint(data, index: &index)
    guard index == data.count else { throw FlightError.protocolViolation("Trailing integer bytes") }
    return value
  }
  public static func decodeSInt(_ data: Data) throws -> Int64 {
    let value = try decodeUInt(data)
    return Int64(bitPattern: (value >> 1) ^ (0 &- (value & 1)))
  }
  /// SpaceCenter.railsWarpFactor is kRPC sint32, NOT uint32.
  /// Encoding its index with uint changes the requested rate (4 -> 2).
  public static func encodeRailsWarpFactor(_ index: UInt64) throws -> Data {
    guard index <= 7 else {
      throw FlightError.invalidCommand("Rails warp factor must be in 0...7")
    }
    return sint(Int64(index))
  }
  public static func decodeRailsWarpFactor(_ bytes: Data) throws -> UInt64 {
    let index = try decodeSInt(bytes)
    guard (0...7).contains(index) else {
      throw FlightError.protocolViolation("Rails warp factor outside 0...7")
    }
    return UInt64(index)
  }
  public static func decodeBool(_ data: Data) throws -> Bool {
    let value = try decodeUInt(data)
    guard value <= 1 else { throw FlightError.protocolViolation("Invalid bool") }
    return value != 0
  }
  private static func bits(_ data: Data, count: Int) throws -> UInt64 {
    guard data.count == count else { throw FlightError.protocolViolation("Invalid scalar width") }
    var value: UInt64 = 0
    for (i, b) in data.enumerated() { value |= UInt64(b) << (8 * i) }
    return value
  }
  public static func decodeDouble(_ data: Data) throws -> Double {
    Double(bitPattern: try bits(data, count: 8))
  }
  public static func decodeFloat(_ data: Data) throws -> Double {
    Double(Float(bitPattern: UInt32(try bits(data, count: 4))))
  }
  public static func decodeString(_ data: Data) throws -> String {
    var index = 0
    let count = try Protobuf.readVarint(data, index: &index)
    guard count == UInt64(data.count - index),
      let value = String(data: data.dropFirst(index), encoding: .utf8)
    else { throw FlightError.protocolViolation("Invalid string") }
    return value
  }
  public static func decodeTuple(_ data: Data, count: Int? = nil) throws -> [Data] {
    let values = Protobuf.bytes(try Protobuf.decode(data), number: 1)
    if let count, values.count != count {
      throw FlightError.protocolViolation("Incorrect tuple arity")
    }
    return values
  }
  public static func decodeVector(_ data: Data) throws -> Vector3 {
    let values = try decodeTuple(data, count: 3).map { try decodeDouble($0) }
    return .init(values[0], values[1], values[2])
  }
}

public enum KRPCCodec {
  public static func connectionRequest(name: String) -> Data { Protobuf.stringField(2, name) }
  public static func decodeConnectionResponse(_ data: Data) throws -> Data {
    let fields = try Protobuf.decode(data)
    guard Protobuf.integer(fields, number: 1) == 0 else {
      throw FlightError.disconnected("kRPC rejected handshake")
    }
    guard let identifier = Protobuf.bytes(fields, number: 3).last, !identifier.isEmpty else {
      throw FlightError.protocolViolation("Missing client identifier")
    }
    return identifier
  }
  public static func request(_ calls: [KRPCCall]) -> Data {
    var data = Data()
    for call in calls { data.append(Protobuf.bytesField(1, call.encoded())) }
    return data
  }
  private static func checkError(_ data: Data) throws {
    let fields = try Protobuf.decode(data)
    let strings = (1...3).map { n in
      Protobuf.bytes(fields, number: n).last.flatMap { String(data: $0, encoding: .utf8) } ?? ""
    }
    throw FlightError.protocolViolation(strings.filter { !$0.isEmpty }.joined(separator: ": "))
  }
  public static func response(_ data: Data, expectedResults: Int) throws -> [Data] {
    let fields = try Protobuf.decode(data)
    if let error = Protobuf.bytes(fields, number: 1).last { try checkError(error) }
    let messages = Protobuf.bytes(fields, number: 2)
    guard messages.count == expectedResults else {
      throw FlightError.protocolViolation("Result count mismatch")
    }
    return try messages.map { message in
      let fields = try Protobuf.decode(message)
      if let error = Protobuf.bytes(fields, number: 1).last { try checkError(error) }
      return Protobuf.bytes(fields, number: 2).last ?? Data()
    }
  }
}
