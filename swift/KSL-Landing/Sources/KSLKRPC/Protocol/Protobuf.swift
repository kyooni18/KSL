import Foundation
import KSLCore

public enum ProtobufValue: Sendable, Equatable {
  case varint(UInt64)
  case fixed64(UInt64)
  case bytes(Data)
  case fixed32(UInt32)
}
public struct ProtobufField: Sendable, Equatable {
  public let number: Int
  public let value: ProtobufValue
}

/// Bounded, alignment-independent native wire codec for the supplied krpc.proto.
/// Unknown fields are preserved/skipped by wire type; deprecated groups are rejected.
public enum Protobuf {
  public static func varint(_ value: UInt64) -> Data {
    var v = value
    var result = Data()
    repeat {
      var b = UInt8(v & 0x7f)
      v >>= 7
      if v != 0 { b |= 0x80 }
      result.append(b)
    } while v != 0
    return result
  }
  public static func readVarint(_ data: Data, index: inout Int) throws -> UInt64 {
    var result: UInt64 = 0
    for byteIndex in 0..<10 {
      guard index < data.count else { throw FlightError.protocolViolation("Truncated varint") }
      let b = data[data.startIndex + index]
      index += 1
      if byteIndex == 9 && b > 1 { throw FlightError.protocolViolation("Varint overflow") }
      result |= UInt64(b & 0x7f) << (7 * byteIndex)
      if b & 0x80 == 0 { return result }
    }
    throw FlightError.protocolViolation("Overlong varint")
  }
  public static func bytesField(_ number: Int, _ bytes: Data) -> Data {
    var result = varint(UInt64(number << 3 | 2))
    result.append(varint(UInt64(bytes.count)))
    result.append(bytes)
    return result
  }
  public static func stringField(_ number: Int, _ string: String) -> Data {
    bytesField(number, Data(string.utf8))
  }
  public static func integerField(_ number: Int, _ value: UInt64) -> Data {
    var result = varint(UInt64(number << 3))
    result.append(varint(value))
    return result
  }
  public static func decode(_ data: Data, maximumBytes: Int = 4 * 1024 * 1024) throws
    -> [ProtobufField]
  {
    guard data.count <= maximumBytes else {
      throw FlightError.protocolViolation("Oversized protobuf message")
    }
    var index = 0
    var result: [ProtobufField] = []
    while index < data.count {
      let tag = try readVarint(data, index: &index)
      let number = Int(tag >> 3)
      let wire = tag & 7
      guard number > 0, number < 536_870_912 else {
        throw FlightError.protocolViolation("Invalid protobuf field tag")
      }
      let value: ProtobufValue
      switch wire {
      case 0: value = .varint(try readVarint(data, index: &index))
      case 1, 5:
        let count = wire == 1 ? 8 : 4
        guard data.count - index >= count else {
          throw FlightError.protocolViolation("Truncated fixed-width value")
        }
        var v: UInt64 = 0
        for j in 0..<count { v |= UInt64(data[data.startIndex + index + j]) << (8 * j) }
        index += count
        value = wire == 1 ? .fixed64(v) : .fixed32(UInt32(v))
      case 2:
        let count = try readVarint(data, index: &index)
        guard count <= UInt64(maximumBytes), count <= UInt64(data.count - index) else {
          throw FlightError.protocolViolation("Invalid length-delimited value")
        }
        value = .bytes(
          Data(data[(data.startIndex + index)..<(data.startIndex + index + Int(count))]))
        index += Int(count)
      default: throw FlightError.protocolViolation("Unsupported protobuf wire type \(wire)")
      }
      result.append(.init(number: number, value: value))
      if result.count > 100_000 { throw FlightError.protocolViolation("Too many protobuf fields") }
    }
    return result
  }
  public static func bytes(_ fields: [ProtobufField], number: Int) -> [Data] {
    fields.compactMap { field in
      if field.number == number, case .bytes(let data) = field.value { return data }
      return nil
    }
  }
  public static func integer(_ fields: [ProtobufField], number: Int, default fallback: UInt64 = 0)
    -> UInt64
  {
    for field in fields.reversed() where field.number == number {
      if case .varint(let v) = field.value { return v }
    }
    return fallback
  }
}
