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
    .systemLibrary(name: "CSQLite", pkgConfig: "sqlite3"),
    .target(name: "KSLCore"),
    .target(name: "KSLPhysics", dependencies: ["KSLCore", "KSLControl", "CSQLite"], resources: [.process("Resources")]),
    .target(name: "KSLGuidance", dependencies: ["KSLCore", "KSLPhysics", "KSLControl"]),
    .target(name: "KSLControl", dependencies: ["KSLCore"]),
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
      dependencies: ["KSLRuntime", "KSLKRPC", "KSLSimulation", "KSLDiagnostics", "KSLControl"]),
    .testTarget(name: "CoreTests", dependencies: ["KSLCore"]),
    .testTarget(name: "PhysicsTests", dependencies: ["KSLPhysics", "CSQLite"]),
    .testTarget(name: "GuidanceTests", dependencies: ["KSLGuidance"]),
    .testTarget(name: "ControlTests", dependencies: ["KSLControl", "KSLPhysics"]),
    .testTarget(
      name: "IntegrationTests", dependencies: ["KSLKRPC", "KSLRuntime", "KSLSimulation"]),
    .testTarget(name: "SimulationTests", dependencies: ["KSLSimulation"]),
  ],
  swiftLanguageModes: [.v6]
)
