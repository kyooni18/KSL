"""Read-only matched-state check. Does not connect to KSP or simulate a plant."""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[2]
package = root / 'swift/KSL-Landing'
tests = package / 'Tests/GuidanceTests'
baseline = tests / 'ZZLocalBaseline.swift'
probe = tests / 'ZZLocalReplayTests.swift'
if baseline.exists() or probe.exists():
    raise SystemExit('Refusing to overwrite existing local test files')
source = subprocess.check_output([
    'git', 'show', '4178a1a:swift/KSL-Landing/Sources/KSLGuidance/TAEM/TAEMGuidanceEngine.swift'
], cwd=root, text=True)
source = ('import Foundation\nimport KSLCore\nimport KSLPhysics\n@testable import KSLGuidance\n'
          + source[source.index('final class TAEMGuidanceEngine'):source.index('/// Offline placement')])
source = source.replace('TAEMGuidanceEngine', 'RecordedBaselineTAEM').replace('TAEMEnergyLine', 'BaselineEnergyLine')
try:
    baseline.write_text(source)
    probe.write_text(Path(__file__).with_name('replay.swift').read_text())
    subprocess.run(['swift', 'test', '--package-path', str(package), '--filter', 'ZZLocalReplayTests'],
                   cwd=root, check=True)
finally:
    baseline.unlink(missing_ok=True)
    probe.unlink(missing_ok=True)
