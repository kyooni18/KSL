import Foundation
import KSLCore

/// One physical lift/drag envelope for MM304 and orbit-to-entry prediction.
/// STS 83-0001-34 4.5.2.13-14: vertical L/D controls ranging, roll sign
/// controls crossrange, and alpha modulates the drag profile.
/// KSL uses its actual aircraft polar and energy constraints, not Shuttle
/// mission-specific angle schedules or an arbitrary nominal-alpha hard cap.
struct EntryForcePoint {
  var alpha: Double
  var lift: Double
  var drag: Double
}

struct EntryForceAllocation {
  var alpha: Double
  var bank: Double
  var drag: Double
  var vertical: Double
  var lateral: Double
  var affordable: Bool
  var verticallySupported: Bool
}

/// Optimistic instantaneous polar test, not a proof of a reachable flight
/// trajectory. Every force/drag tuple comes from ONE AoA operating point;
/// finite attitude response, temperature, and future density are not assumed
/// away by the downstream planner.
struct EntryJointForceFeasibility {
  let requiredLift: Double
  let maximumAvailableLift: Double
  let maximumLateralWithVerticalSupport: Double
  let minimumSimultaneousDrag: Double?
  let dragBudgetDeficit: Double?
  var simultaneouslySupported: Bool { minimumSimultaneousDrag != nil }
}

enum EntryForceAllocator {
  static func assessJointForces(points: [EntryForcePoint],
                                verticalDemand: Double, lateralDemand: Double,
                                maximumBank: Double, loadLimit: Double,
                                uncertainty: Double, dragBudget: Double) -> EntryJointForceFeasibility {
    let vertical = max(0, verticalDemand)
    let lateral = abs(lateralDemand)
    let bankLimit = clamp(abs(maximumBank), 0, .pi / 2)
    let safe = points.filter { $0.lift.isFinite && $0.drag.isFinite
      && $0.lift >= 0 && $0.drag >= 0
      && $0.lift <= loadLimit * (1-uncertainty) }
    let maxLift = safe.map(\.lift).max() ?? 0
    let simultaneous = safe.filter { point in
      guard point.lift >= hypot(vertical, lateral) else { return false }
      // Resolve both lift components in the SAME polar point.
      let bank = asin(clamp(lateral / max(point.lift, Double.leastNormalMagnitude), 0, 1))
      return bank <= bankLimit && point.lift * cos(bank) >= vertical
    }
    let minDrag = simultaneous.map { $0.drag*(1+uncertainty) }.min()
    let availableLateral = safe.reduce(0.0) { best, point in
      max(best, NormalForceEnvelope(maximumLift: point.lift, maximumBank: bankLimit)
        .lateralAuthority(reserving: vertical))
    }
    return .init(requiredLift: hypot(vertical, lateral),
      maximumAvailableLift: maxLift,
      maximumLateralWithVerticalSupport: availableLateral,
      minimumSimultaneousDrag: minDrag,
      dragBudgetDeficit: minDrag.map { max(0, $0 - max(0, dragBudget)) })
  }

  static func decide(points: [EntryForcePoint], verticalDemand: Double,
                     lateralDemand: Double, referenceDrag: Double,
                     dragBudget: Double, maximumBank: Double,
                     loadLimit: Double, uncertainty: Double,
                     previousAlpha: Double, heightAboveCapture: Double = 0,
                     timeToCapture: Double = .infinity) -> EntryForceAllocation {
    precondition(!points.isEmpty)
    let safe = points.filter { $0.lift <= loadLimit * (1-uncertainty) }
    let permitted = safe.isEmpty ? points : safe
    let supporting = permitted.filter { $0.lift >= verticalDemand }
    let affordable = supporting.filter { $0.drag * (1+uncertainty) <= dragBudget }
    func authority(_ p: EntryForcePoint) -> Double {
      NormalForceEnvelope(maximumLift: p.lift, maximumBank: maximumBank)
        .lateralAuthority(reserving: max(0, verticalDemand))
    }
    let candidates = affordable.isEmpty ? supporting : affordable
    let lateralGoal = min(abs(lateralDemand),
      candidates.reduce(0.0) { max($0, authority($1)) })
    let sign = lateralDemand < 0 ? -1.0 : 1.0
    func project(_ p: EntryForcePoint) -> EntryForceAllocation {
      let lateral = min(lateralGoal, authority(p))
      let angle = p.lift > 0 ? asin(clamp(lateral / p.lift, 0, 1)) : 0
      let bank = sign*angle
      return .init(alpha: p.alpha, bank: bank, drag: p.drag,
        vertical: p.lift*cos(bank), lateral: p.lift*sin(bank),
        affordable: !affordable.isEmpty, verticallySupported: !supporting.isEmpty)
    }
    if !affordable.isEmpty {
      func residual(_ p: EntryForcePoint) -> Double {
        let f = project(p)
        // Same units (m/s2) in all three axes; no arbitrary relative gains.
        return hypot(f.drag-referenceDrag,
          hypot(f.vertical-verticalDemand, f.lateral-sign*lateralGoal))
      }
      return project(affordable.min { a,b in
        let left = residual(a), right = residual(b)
        return left == right ? abs(a.alpha-previousAlpha) < abs(b.alpha-previousAlpha)
          : left < right
      }!)
    }
    if !supporting.isEmpty {
      // Low-energy stretch: spend the least drag per achievable steering
      // acceleration without discarding vertical lift support.
      let recover = supporting.min { a,b in
        let al = min(abs(lateralDemand), authority(a))
        let bl = min(abs(lateralDemand), authority(b))
        if al > 0 || bl > 0 {
          let ae = al > 0 ? a.drag/al : Double.greatestFiniteMagnitude
          let be = bl > 0 ? b.drag/bl : Double.greatestFiniteMagnitude
          if ae != be { return ae < be }
        }
        return a.drag < b.drag
      }!
      return project(recover)
    }
    // No available incidence can meet the demanded vertical support.
    // Holding maximum CL then spends all remaining range on CD even though
    // the extra lift cannot arrest the descent (observed at 40--35 km in STS).
    // While there is altitude to exchange, maximize L/D only when maximum-CL
    // already violates the captured terminal energy budget. Do NOT trade the
    // last altitude at MM305's altitude floor for a nominal range gain.
    let maxLift = permitted.max { a,b in
      a.lift == b.lift ? a.drag > b.drag : a.lift < b.lift
    }!
    // Max-CL sits past best L/D: below capture height it still cannot arrest
    // the descent and only bleeds the speed MM305 needs. Prefer best L/D.
    let recover: EntryForcePoint
    if maxLift.lift > 0 {
      recover = permitted.max { a,b in
        let ratioA = a.lift/max(1e-6, a.drag)
        let ratioB = b.lift/max(1e-6, b.drag)
        return ratioA == ratioB ? a.lift < b.lift : ratioA < ratioB
      }!
    } else {
      recover = maxLift
    }
    // When *no* incidence meets the fixed vertical reference, an unbanked
    // recovery is not necessarily the best physically usable continuation.
    // Bound the additional loss of normal support by the still-available
    // altitude above the preferred inlet. Under a constant normal-accel
    // approximation, delta-h = 0.5 * delta-a * T^2. This is only a
    // local control allocation, never a terminal reachability certificate.
    //
    // Keep the model-uncertainty fraction of the height as a reserve. If
    // height is exhausted, or there is too little time/altitude to turn,
    // return the previous best-glide/maximum-lift unbanked recovery.
    let spareHeight = max(0, heightAboveCapture) * (1 - clamp(uncertainty, 0, 1))
    if spareHeight > 0 && timeToCapture.isFinite && timeToCapture > 0 &&
       abs(lateralDemand) > 0 {
      let normalDeficitAllowance = 2 * spareHeight / (timeToCapture * timeToCapture)
      let minimumVertical = max(0, verticalDemand - normalDeficitAllowance)
      var options: [(point: EntryForcePoint, bank: Double, lateral: Double)] = []
      for point in permitted where point.lift > minimumVertical && point.lift > 0 {
        let bankLimit = min(maximumBank, acos(clamp(minimumVertical / point.lift, 0, 1)))
        let lateral = min(abs(lateralDemand), point.lift * sin(bankLimit))
        guard lateral > 0 else { continue }
        let bank = asin(clamp(lateral / point.lift, 0, 1))
        options.append((point, bank, lateral))
      }
      // When capture's vertical-force request is already impossible, the
      // remaining altitude is explicitly budgeted for lateral steering.
      // Do NOT optimize lateral/drag on an unbounded polar set: that may
      // select a point which exceeds the *actual available energy budget*
      // even when another incidence can steer within it. A feasible budget
      // admits the greatest physical turn acceleration from ONE lift/drag
      // polar point; use lower drag only to break equal turn authority.
      let withinBudget = options.filter {
        $0.point.drag * (1 + uncertainty) <= dragBudget
      }
      let candidates = withinBudget.isEmpty ? options : withinBudget
      let best = candidates.max { a, b in
        if withinBudget.isEmpty {
          // Every available steering point exceeds the energy budget.
          // Keep maximum lateral turn per dissipated energy, and report the
          // impossible drag objective instead of claiming it was tracked.
          let ae = a.lateral / max(1e-6, a.point.drag * (1 + uncertainty))
          let be = b.lateral / max(1e-6, b.point.drag * (1 + uncertainty))
          if ae != be { return ae < be }
        } else if abs(a.lateral-b.lateral) > 1e-9 {
          return a.lateral < b.lateral
        }
        return a.point.drag > b.point.drag
      }
      if let best {
        let bank = (lateralDemand < 0 ? -1.0 : 1.0) * best.bank
        let vertical = best.point.lift * cos(bank)
        return .init(alpha: best.point.alpha, bank: bank, drag: best.point.drag,
          vertical: vertical, lateral: best.point.lift * sin(bank),
          affordable: false, verticallySupported: false)
      }
    }
    return .init(alpha: recover.alpha, bank: 0, drag: recover.drag,
      vertical: recover.lift, lateral: 0, affordable: false, verticallySupported: false)
  }
}
