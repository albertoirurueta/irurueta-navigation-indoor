# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.6.0] - 2026-07-26

### Changed

- Raised the minimum Java version required to build and use this library from Java 17 to Java 21.
- Upgraded internal dependencies: `irurueta-navigation` to 1.8.1, `irurueta-numerical` to 1.6.0,
  `irurueta-geometry` to 1.6.0, `irurueta-units` to 1.4.0, and `irurueta-algebra` to 1.4.0.

### Added

- Documentation covering the RSSI-based, ranging-based, combined, fingerprint-based, and radio source estimator
  hierarchies, including algorithm explanations, equations, and overview diagrams.

## [1.5.0] - 2026-03-04

No user-facing changes in this release; it consists solely of an internal dependency version bump in `pom.xml`
and a minor test-only fix.

## [1.4.0] - 2025-12-18

No user-facing changes in this release; it consists solely of an internal dependency version bump in `pom.xml`
and a minor test-only fix.

## [1.3.2] - 2025-09-23

No user-facing changes in this release; it consists solely of a dependency version bump in `pom.xml` and a
version reference update in `README.md`.

## [1.3.1] - 2025-09-20

No user-facing changes in this release; it consists solely of CI workflow, `README.md`, and Maven build/plugin
tooling updates.

## [1.3.0] - 2025-03-12

### Changed

- Updated internal dependencies `irurueta-numerical`, `irurueta-geometry`, `irurueta-units`, and
  `irurueta-algebra` to 1.3.0, and `irurueta-navigation` to 1.5.0.

## [1.2.0] - 2024-02-09

### Changed

- Raised the minimum Java version required to build and use this library from Java 7 to Java 17.
- Upgraded internal dependencies: `irurueta-numerical` to 1.2.1, `irurueta-navigation` to 1.4.1, and
  `irurueta-geometry`, `irurueta-units`, and `irurueta-algebra` to 1.2.0.
- Renamed the `RobustEstimatorMethod` constants used by the position/radio-source estimator factory methods from
  `LMedS`/`PROMedS` to `LMEDS`/`PROMEDS` as a consequence of the `irurueta-numerical` upgrade — code referencing
  the old constant names will need to be updated (breaking change).

## [1.1.0] - 2021-12-11

### Changed

- Updated internal dependencies `irurueta-numerical`, `irurueta-geometry`, and `irurueta-units` to 1.1.0.
- Updated test dependencies: JUnit to 4.13.2, Hamcrest to 2.2, and Mockito to 4.1.0.

## [1.0.2] - 2021-12-10

No user-facing changes in this release; it consists solely of a CI workflow fix, a `README.md` update, and a
version bump in `pom.xml`.

## [1.0.1] - 2021-12-10

No user-facing changes in this release; it consists solely of a `README.md` update and a version bump in
`pom.xml`.

## [1.0.0] - 2021-12-10

### Added

- Initial release of the indoor positioning library, providing:
  - Core indoor-navigation data model types — readings, fingerprints, and radio sources — in
    `com.irurueta.navigation.indoor`.
  - Fingerprint-based indoor position estimators (`com.irurueta.navigation.indoor.fingerprint`) using RSSI and/or
    ranging readings.
  - Position estimators built on fingerprint and ranging/RSSI data (`com.irurueta.navigation.indoor.position`).
  - Radio source (e.g., WiFi access point) position and path-loss exponent estimators, including robust
    estimators (`com.irurueta.navigation.indoor.radiosource`).

[Unreleased]: https://github.com/albertoirurueta/irurueta-navigation-indoor/compare/1.6.0...HEAD
[1.6.0]: https://github.com/albertoirurueta/irurueta-navigation-indoor/compare/1.5.0...1.6.0
[1.5.0]: https://github.com/albertoirurueta/irurueta-navigation-indoor/compare/1.4.0...1.5.0
[1.4.0]: https://github.com/albertoirurueta/irurueta-navigation-indoor/compare/1.3.2...1.4.0
[1.3.2]: https://github.com/albertoirurueta/irurueta-navigation-indoor/compare/1.3.1...1.3.2
[1.3.1]: https://github.com/albertoirurueta/irurueta-navigation-indoor/compare/1.3.0...1.3.1
[1.3.0]: https://github.com/albertoirurueta/irurueta-navigation-indoor/compare/1.2.0...1.3.0
[1.2.0]: https://github.com/albertoirurueta/irurueta-navigation-indoor/compare/1.1.0...1.2.0
[1.1.0]: https://github.com/albertoirurueta/irurueta-navigation-indoor/compare/1.0.2...1.1.0
[1.0.2]: https://github.com/albertoirurueta/irurueta-navigation-indoor/compare/1.0.1...1.0.2
[1.0.1]: https://github.com/albertoirurueta/irurueta-navigation-indoor/compare/1.0.0...1.0.1
[1.0.0]: https://github.com/albertoirurueta/irurueta-navigation-indoor/releases/tag/1.0.0
