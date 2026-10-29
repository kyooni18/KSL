import Foundation
import KSLCore

/// No await inside a wire exchange: requests and responses cannot interleave.
public actor KRPCSession {
  private let configuration: KRPCConnectionConfiguration
  private var transport: (any RPCTransport)?
  private let injected: Bool
  private var clientIdentifier: Data?
  public private(set) var requestCount = 0
  public init(configuration: KRPCConnectionConfiguration = .init()) {
    self.configuration = configuration
    injected = false
  }
  public init(
    configuration: KRPCConnectionConfiguration = .init(), transport: sending any RPCTransport
  ) {
    self.configuration = configuration
    self.transport = transport
    injected = true
  }
  public var isConnected: Bool { clientIdentifier != nil }
  public func connect() throws {
    try configuration.validate()
    if clientIdentifier != nil { return }
    if transport == nil {
      guard !injected else { throw FlightError.disconnected("Injected transport closed") }
      transport = try TCPTransport(configuration: configuration)
    }
    do {
      clientIdentifier = try KRPCCodec.decodeConnectionResponse(
        exchange(KRPCCodec.connectionRequest(name: configuration.clientName)))
    } catch {
      close()
      throw error
    }
  }
  public func call(_ calls: [KRPCCall]) throws -> [Data] {
    guard clientIdentifier != nil else { throw FlightError.disconnected("Session not connected") }
    do {
      try Task.checkCancellation()
      let result = try KRPCCodec.response(
        exchange(KRPCCodec.request(calls)), expectedResults: calls.count)
      requestCount += 1
      return result
    } catch {
      close()
      throw error
    }
  }
  private func exchange(_ body: Data) throws -> Data {
    guard let transport, body.count <= configuration.maximumFrameBytes else {
      throw FlightError.protocolViolation("Missing socket or oversized request")
    }
    let deadline = ContinuousClock.now.advanced(by: .seconds(configuration.timeout))
    var frame = Protobuf.varint(UInt64(body.count))
    frame.append(body)
    try transport.write(frame, deadline: deadline)
    var prefix = Data()
    for _ in 0..<5 {
      let byte = try transport.read(count: 1, deadline: deadline)
      prefix.append(byte)
      if byte[0] & 0x80 == 0 {
        var index = 0
        let length = try Protobuf.readVarint(prefix, index: &index)
        guard length <= configuration.maximumFrameBytes else {
          throw FlightError.protocolViolation("Oversized response frame")
        }
        return try transport.read(count: Int(length), deadline: deadline)
      }
    }
    throw FlightError.protocolViolation("Invalid frame-length prefix")
  }
  /// Explicit reconnection always uses a new handshake; commands are never replayed.
  public func reconnect() throws {
    close()
    try connect()
  }
  public func close() {
    transport?.close()
    transport = nil
    clientIdentifier = nil
  }
}
