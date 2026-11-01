import Foundation
import Testing
@testable import KSLKRPC

struct OrbitalRailsWarpScheduleTests {
  @Test func higherTimeAllowsHigherRailRate() {
    #expect(OrbitalRailsWarpSchedule.factor(remainingToStop: 2_000_000) == 7)
    #expect(OrbitalRailsWarpSchedule.factor(remainingToStop: 100_000) == 6)
    #expect(OrbitalRailsWarpSchedule.factor(remainingToStop: 20_000) == 5)
    #expect(OrbitalRailsWarpSchedule.factor(remainingToStop: 1_000) == 4)
    #expect(OrbitalRailsWarpSchedule.factor(remainingToStop: 500) == 3)
    #expect(OrbitalRailsWarpSchedule.factor(remainingToStop: 40) == 2)
    #expect(OrbitalRailsWarpSchedule.factor(remainingToStop: 12) == 1)
    #expect(OrbitalRailsWarpSchedule.factor(remainingToStop: 2) == 0)
  }

  @Test func scheduleDoesNotIncreaseNearCutoff() {
    var previous: UInt64 = 7
    for remaining in stride(from: 50_000.0, through: 0, by: -5.0) {
      let next = OrbitalRailsWarpSchedule.factor(remainingToStop: remaining)
      #expect(next <= previous)
      previous = next
    }
  }

  @Test func pollingShrinksWithRemainingTime() {
    let schedule = OrbitalRailsWarpSchedule.self
    #expect(schedule.pollIntervalMillis(remainingToStop: 20_000, requestedFactor: 5) == 350)
    #expect(schedule.pollIntervalMillis(remainingToStop: 20, requestedFactor: 2) == 100)
    #expect(schedule.pollIntervalMillis(remainingToStop: 0.1, requestedFactor: 1) == 40)
  }

  @Test func rejectedHighRateFallsBackToUsableLowerSlots() {
    #expect(OrbitalRailsWarpSchedule.nextLowerAfterRejection(7) == 6)
    #expect(OrbitalRailsWarpSchedule.nextLowerAfterRejection(5) == 4)
    #expect(OrbitalRailsWarpSchedule.nextLowerAfterRejection(4) == 3)
    #expect(OrbitalRailsWarpSchedule.nextLowerAfterRejection(3) == 2)
    #expect(OrbitalRailsWarpSchedule.nextLowerAfterRejection(2) == 1)
    #expect(OrbitalRailsWarpSchedule.nextLowerAfterRejection(1) == nil)
    #expect(OrbitalRailsWarpSchedule.nextLowerAfterRejection(0) == nil)
  }

  @Test func invalidRemainingTimeDisablesWarp() {
    #expect(OrbitalRailsWarpSchedule.factor(remainingToStop: -100) == 0)
    #expect(OrbitalRailsWarpSchedule.factor(remainingToStop: .nan) == 0)
  }
}
