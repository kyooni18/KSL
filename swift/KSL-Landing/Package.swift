// swift-tools-version: 6.0
import PackageDescription

let package = Package(
  name: "KSL-Landing",
  platforms: [.macOS(.v13)],
  products: [
    .executable(name: "ksl-landing", targets: ["KSLLanding"]),
    .library(name: "KSLCore", targets: ["KSLCore"]),
    .library(name: "KSLPhysics", targets: ["KSLPhysics"]),
    .library(name: "KSLGuidance", targets: ["KSLGuidance"]),
    .library(name: "KSLRuntime", targets: ["KSLRuntime"]),
    .library(name: "KSLKRPC", targets: ["KSLKRPC"]),
  ],
  targets: [
    .target(name: "KSLCore"),
    .target(name: "KSLPhysics", dependencies: ["KSLCore"], resources: [.process("Resources")]),
    .target(name: "KSLGuidance", dependencies: ["KSLCore", "KSLPhysics"]),
    .target(name: "KSLControl", dependencies: ["KSLCore", "KSLPhysics"]),
    .target(name: "KSLKRPC", dependencies: ["KSLCore"]),
    .target(name: "KSLDiagnostics", dependencies: ["KSLCore"]),
    .target(
      name: "KSLRuntime",
      dependencies: ["KSLCore", "KSLPhysics", "KSLGuidance", "KSLControl", "KSLDiagnostics"]),
    .target(
      name: "KSLSimulation",
      dependencies: ["KSLCore", "KSLPhysics", "KSLRuntime", "KSLDiagnostics"]),
    .executableTarget(
      name: "KSLLanding",
      dependencies: ["KSLRuntime", "KSLKRPC", "KSLSimulation", "KSLDiagnostics"]),
    .testTarget(name: "CoreTests", dependencies: ["KSLCore"]),
    .testTarget(name: "PhysicsTests", dependencies: ["KSLPhysics"]),
    .testTarget(name: "GuidanceTests", dependencies: ["KSLGuidance"]),
    .testTarget(name: "ControlTests", dependencies: ["KSLControl"]),
    .testTarget(
      name: "IntegrationTests", dependencies: ["KSLKRPC", "KSLRuntime", "KSLSimulation"]),
    .testTarget(name: "SimulationTests", dependencies: ["KSLSimulation"]),
  ],
  swiftLanguageModes: [.v6]
)
