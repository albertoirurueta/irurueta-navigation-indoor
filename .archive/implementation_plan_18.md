# Implementation plan: Update dependencies

## Task summary

Bump the `com.irurueta` dependencies in `pom.xml` to the latest versions requested by the ticket: irurueta-numerical
1.6.0 → 1.6.1, irurueta-geometry 1.6.0 → 1.6.1, irurueta-units 1.4.0 → 1.4.1, irurueta-algebra 1.4.0 → 1.4.1 and
irurueta-navigation 1.8.1 → 1.8.2. No source changes are expected; the build and full test suite verify compatibility.

Approach chosen (not ambiguous, so not asked): a single pom-only change, followed by one validation pass. The
README's `1.6.0` is this library's own release version and is intentionally left untouched.

Source: GitHub issue #18
Base branch: develop
Open clarifications: none

## Acceptance criteria coverage

| # | Criterion | Covered by tasks |
|---|---|---|
| 1 | "Update dependencies in pom.xl file to latest versions of: irurueta-numerical 1.6.1, irurueta-geometry 1.6.1, irurueta-units 1.4.1, irurueta-algebra 1.4.1, irurueta-navigation 1.8.2" | Task 1 |
| 2 | derived: the project compiles and all existing tests pass against the new versions | Task 2 |
| 3 | derived: coverage and static analysis (Checkstyle) do not regress | Task 2 |

## Current code state

- Single-module Maven library (`java-library`, no wrapper), `pom.xml` at the repository root.
- `pom.xml` lines ~60-89 declare the five `com.irurueta` dependencies, all `compile` scope, at the old versions.
- Local `~/.m2` already holds numerical 1.6.1, units 1.4.1 and algebra 1.4.1; irurueta-navigation 1.8.2 (and possibly
  geometry 1.6.1) must be downloaded from Maven Central.
- Sources live in `src/main/java/com/irurueta/navigation/indoor`, tests in `src/test/java`.

## Implementation steps

### Group 1 — Bump versions (Parallelizable: yes)

- [x] Task 1. Update dependency versions in `pom.xml` _(java)_ — pom.xml only; no sources/tests changed (no API incompatibilities); full suite 3810 tests pass; no code changes so no new quality issues.
  - [x] Task 1.1. Set irurueta-numerical and irurueta-geometry `<version>` to `1.6.1`
  - [x] Task 1.2. Set irurueta-units and irurueta-algebra `<version>` to `1.4.1`
  - [x] Task 1.3. Set irurueta-navigation `<version>` to `1.8.2`
  - [x] Task 1.4. If the build reveals API incompatibilities in the new versions, adapt the affected sources/tests minimally

### Group 2 — Validate (Parallelizable: yes)

- [x] Task 2. Verify build, tests, coverage and quality _(java)_
  - [x] Task 2.1. Run the full unit test suite via the `iru-gate-runner` agent (`iru-java-test`) — 3810 tests, 0 failures.
  - [x] Task 2.2. Check coverage via `iru-gate-runner` (`iru-java-coverage`), no regression versus baseline — overall 96.97% line, 89.56% branch; no Java changed.
  - [x] Task 2.3. Run Checkstyle/PMD/SpotBugs via `iru-gate-runner` (`iru-java-code-quality`) — Checkstyle 26186, PMD 46, SpotBugs 804, equal to baseline.
