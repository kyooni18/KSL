import Foundation
import KSLCore

#if os(Linux)
  import Glibc
#else
  import Darwin
#endif

public struct KRPCConnectionConfiguration: Codable, Sendable {
  public var host: String = "127.0.0.1"
  public var port: UInt16 = 50000
  public var timeout: Double = 2
  public var clientName: String = "KSL Swift Landing"
  public var maximumFrameBytes: Int = 4 * 1024 * 1024
  public init() {}
  public func validate() throws {
    guard port > 0, timeout.isFinite, timeout >= 0.05, timeout <= 30,
      maximumFrameBytes > 0, maximumFrameBytes <= 4 * 1024 * 1024,
      !host.isEmpty, clientName.utf8.count <= 1024
    else {
      throw FlightError.invalidConfiguration("Invalid kRPC connection limits")
    }
  }
}

public protocol RPCTransport: AnyObject {
  func write(_ data: Data, deadline: ContinuousClock.Instant) throws
  func read(count: Int, deadline: ContinuousClock.Instant) throws -> Data
  func close()
}

/// Owned exclusively by KRPCSession. Nonblocking socket + bounded poll deadlines.
public final class TCPTransport: RPCTransport {
  private var descriptor: Int32 = -1
  public init(configuration: KRPCConnectionConfiguration) throws {
    try configuration.validate()
    var hints = addrinfo()
    hints.ai_family = AF_UNSPEC
    #if os(Linux)
      hints.ai_socktype = Int32(SOCK_STREAM.rawValue)
    #else
      hints.ai_socktype = SOCK_STREAM
    #endif
    hints.ai_flags = AI_NUMERICHOST
    hints.ai_protocol = Int32(IPPROTO_TCP)
    var addresses: UnsafeMutablePointer<addrinfo>?
    let host = configuration.host == "localhost" ? "127.0.0.1" : configuration.host
    let status = getaddrinfo(host, String(configuration.port), &hints, &addresses)
    guard status == 0, let address = addresses else {
      throw FlightError.disconnected(
        "Use a numeric IPv4/IPv6 address: \(String(cString:gai_strerror(status)))")
    }
    defer { freeaddrinfo(addresses) }
    descriptor = socket(
      address.pointee.ai_family, address.pointee.ai_socktype, address.pointee.ai_protocol)
    guard descriptor >= 0 else { throw FlightError.disconnected("socket errno \(errno)") }
    do {
      guard fcntl(descriptor, F_SETFL, fcntl(descriptor, F_GETFL, 0) | O_NONBLOCK) >= 0 else {
        throw FlightError.disconnected("fcntl errno \(errno)")
      }
      var one: Int32 = 1
      _ = setsockopt(
        descriptor, Int32(IPPROTO_TCP), TCP_NODELAY, &one,
        socklen_t(MemoryLayout.size(ofValue: one)))
      #if !os(Linux)
        _ = setsockopt(
          descriptor, SOL_SOCKET, SO_NOSIGPIPE, &one, socklen_t(MemoryLayout.size(ofValue: one)))
      #endif
      let result = connect(descriptor, address.pointee.ai_addr, address.pointee.ai_addrlen)
      if result < 0 && errno != EINPROGRESS {
        throw FlightError.disconnected("connect errno \(errno)")
      }
      let deadline = ContinuousClock.now.advanced(by: .seconds(configuration.timeout))
      if result < 0 {
        try wait(events: Int16(POLLOUT), deadline: deadline)
        var error: Int32 = 0
        var size = socklen_t(MemoryLayout<Int32>.size)
        guard getsockopt(descriptor, SOL_SOCKET, SO_ERROR, &error, &size) == 0, error == 0 else {
          throw FlightError.disconnected("connect error \(error)")
        }
      }
    } catch {
      close()
      throw error
    }
  }
  private func wait(events: Int16, deadline: ContinuousClock.Instant) throws {
    while true {
      try Task.checkCancellation()
      guard descriptor >= 0 else { throw FlightError.disconnected("Socket closed") }
      let remaining = ContinuousClock.now.duration(to: deadline)
      guard remaining > .zero else { throw FlightError.deadlineExceeded }
      let parts = remaining.components
      let milliseconds = min(
        50, max(1, Int(parts.seconds * 1000 + parts.attoseconds / 1_000_000_000_000_000)))
      var fd = pollfd(fd: descriptor, events: events, revents: 0)
      let result = poll(&fd, 1, Int32(milliseconds))
      if result < 0 {
        if errno == EINTR { continue }
        throw FlightError.disconnected("poll errno \(errno)")
      }
      if result > 0 {
        if fd.revents & events != 0 { return }
        if fd.revents & Int16(POLLERR | POLLHUP | POLLNVAL) != 0 {
          throw FlightError.disconnected("Socket hangup")
        }
      }
    }
  }
  public func write(_ data: Data, deadline: ContinuousClock.Instant) throws {
    var offset = 0
    while offset < data.count {
      try wait(events: Int16(POLLOUT), deadline: deadline)
      let count = data.withUnsafeBytes { buffer -> Int in
        #if os(Linux)
          return send(
            descriptor, buffer.baseAddress!.advanced(by: offset), data.count - offset,
            Int32(MSG_NOSIGNAL))
        #else
          return send(descriptor, buffer.baseAddress!.advanced(by: offset), data.count - offset, 0)
        #endif
      }
      if count < 0 {
        if errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR { continue }
        throw FlightError.disconnected("send errno \(errno)")
      }
      if count == 0 { throw FlightError.disconnected("Socket write closed") }
      offset += count
    }
  }
  public func read(count: Int, deadline: ContinuousClock.Instant) throws -> Data {
    guard count >= 0, count <= 4 * 1024 * 1024 else {
      throw FlightError.protocolViolation("Invalid socket read size")
    }
    var result = Data(count: count)
    var offset = 0
    while offset < count {
      try wait(events: Int16(POLLIN), deadline: deadline)
      let n = result.withUnsafeMutableBytes { buffer in
        recv(descriptor, buffer.baseAddress!.advanced(by: offset), count - offset, 0)
      }
      if n < 0 {
        if errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR { continue }
        throw FlightError.disconnected("recv errno \(errno)")
      }
      if n == 0 { throw FlightError.disconnected("End of stream") }
      offset += n
    }
    return result
  }
  public func close() {
    guard descriptor >= 0 else { return }
    let fd = descriptor
    descriptor = -1
    #if os(Linux)
      _ = Glibc.shutdown(fd, Int32(SHUT_RDWR))
      _ = Glibc.close(fd)
    #else
      _ = Darwin.shutdown(fd, SHUT_RDWR)
      _ = Darwin.close(fd)
    #endif
  }
  deinit { close() }
}
