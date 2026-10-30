import Foundation
import KSLCore
import KSLControl
import KSLPhysics
@testable import KSLRuntime
struct Screen: Decodable { let candidates: [DeorbitWindow] }
@main struct Probe {
 static func main() throws {
  let root = CommandLine.arguments[1]
  let decoder = JSONDecoder()
  let c = try decoder.decode(MissionConfiguration.self, from: Data(contentsOf: URL(fileURLWithPath: root+"/configuration.json")))
  let s = try decoder.decode(VehicleState.self, from: Data(contentsOf: URL(fileURLWithPath: root+"/state.json")))
  let screen = try decoder.decode(Screen.self, from: Data(contentsOf: URL(fileURLWithPath: root+"/screen.json")))
  for (i, window) in screen.candidates.enumerated() {
   let t = Date()
   do {
    let r = try OrbitalExecutionModel(configuration: c).predict(source: s, window: window, propulsion: s.propulsion!, dispersion: DeorbitDispersion.cases(c)[0])
    print("candidate=\(i) burnUT=\(window.burnStart) completed=\(r.burnCompleted) entryUT=\(r.entry.time) fpa=\(r.entry.flightPathAngle) burnEndUT=\(r.actualBurnEnd) seconds=\(Date().timeIntervalSince(t))")
   } catch { print("candidate=\(i) burnUT=\(window.burnStart) error=\(error) seconds=\(Date().timeIntervalSince(t))") }
   fflush(stdout)
  }
 }
}
