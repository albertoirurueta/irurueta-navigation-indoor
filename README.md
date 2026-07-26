# irurueta-navigation-indoor

An indoor GNSS/INS navigation library

[![Build Status](https://github.com/albertoirurueta/irurueta-navigation-indoor/actions/workflows/master.yml/badge.svg)](https://github.com/albertoirurueta/irurueta-navigation-indoor/actions/workflows/master.yml)
[![Build Status](https://github.com/albertoirurueta/irurueta-navigation-indoor/actions/workflows/develop.yml/badge.svg)](https://github.com/albertoirurueta/irurueta-navigation-indoor/actions/workflows/develop.yml)

[![Bugs](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-indoor&metric=bugs)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-indoor)
[![Code Smells](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-indoor&metric=code_smells)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-indoor)
[![Coverage](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-indoor&metric=coverage)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-indoor)

[![Duplicated lines](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-indoor&metric=duplicated_lines_density)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-indoor)
[![Lines of code](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-indoor&metric=ncloc)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-indoor)

[![Maintainability](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-indoor&metric=sqale_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-indoor)
[![Quality gate](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-indoor&metric=alert_status)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-indoor)
[![Reliability](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-indoor&metric=reliability_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-indoor)

[![Security](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-indoor&metric=security_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-indoor)
[![Technical debt](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-indoor&metric=sqale_index)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-indoor)
[![Vulnerabilities](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-indoor&metric=vulnerabilities)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-indoor)

## Project Status

| | |
|---|---|
| Language | Java 21 |
| Build tool | Maven |
| Current development version | 1.7.0-SNAPSHOT |
| Latest release | 1.6.0 |
| License | Apache License, Version 2.0 |
| CI | GitHub Actions — builds/tests `develop` on every push and runs the `master` release pipeline on every published GitHub release |
| Quality | SonarCloud, JaCoCo coverage, Checkstyle, PMD, SpotBugs, Javadoc |

## Documentation

* [Antora documentation site](https://albertoirurueta.github.io/irurueta-navigation-indoor) — concepts, installation guide, and generated report links.
* [Maven site report](https://albertoirurueta.github.io/irurueta-navigation-indoor/mvn-site) — Javadoc, unit test results, coverage, and static analysis, aggregated.
* [SonarCloud dashboard](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-indoor)
* [CHANGELOG](CHANGELOG.md)

## Installation

Add the following dependency to your project:

Latest release:
```xml
<dependency>
    <groupId>com.irurueta</groupId>
    <artifactId>irurueta-navigation-indoor</artifactId>
    <version>1.6.0</version>
    <scope>compile</scope>
</dependency>
```

Latest snapshot:
```xml
<dependency>
    <groupId>com.irurueta</groupId>
    <artifactId>irurueta-navigation-indoor</artifactId>
    <version>1.7.0-SNAPSHOT</version>
    <scope>compile</scope>
</dependency>
```

Snapshot versions are published to the Sonatype OSS snapshots repository. Add
`https://s01.oss.sonatype.org/content/repositories/snapshots/` as a repository to your build in order to resolve
them.

## How It Works

`irurueta-navigation-indoor` estimates a device's position indoors — where GPS is unavailable — from radio signals
(Wi-Fi access points, Bluetooth beacons) already present in a building, using well-established estimation and
robust-statistics techniques (least squares, non-linear optimization, LMedS, MSAC, PROSAC) so that noisy or
partially wrong measurements don't derail the result.

The library covers three complementary problems:

* **Fingerprint-based positioning** — matching a new signal reading against a database of readings taken at known
  locations, without needing to know where the radio sources themselves are.
* **Direct (lateration) positioning** — solving directly for a device's position from RSSI/ranging readings when
  the radio sources' locations *are* known.
* **Radio source estimation** — the inverse problem: estimating an unknown radio source's own position and
  transmission parameters from readings taken at known locations.

The example below uses `LinearRssiPositionEstimator2D` to estimate a device's 2D position from RSSI readings taken
against three Wi-Fi access points whose positions and transmitted power are already known:

```java
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.RssiFingerprint;
import com.irurueta.navigation.indoor.RssiReading;
import com.irurueta.navigation.indoor.WifiAccessPoint;
import com.irurueta.navigation.indoor.WifiAccessPointWithPowerAndLocated2D;
import com.irurueta.navigation.indoor.position.LinearRssiPositionEstimator2D;
import com.irurueta.navigation.indoor.position.PositionEstimationException;

import java.util.List;

public class IndoorPositioningExample {

    public static void main(final String[] args) throws LockedException, NotReadyException,
            PositionEstimationException {
        // known access points, with their transmitted power (dBm) and known 2D position
        final var ap1 = new WifiAccessPointWithPowerAndLocated2D(
                "ap1", 2.4e9, -50.0, new InhomogeneousPoint2D(0.0, 0.0));
        final var ap2 = new WifiAccessPointWithPowerAndLocated2D(
                "ap2", 2.4e9, -50.0, new InhomogeneousPoint2D(10.0, 0.0));
        final var ap3 = new WifiAccessPointWithPowerAndLocated2D(
                "ap3", 2.4e9, -50.0, new InhomogeneousPoint2D(0.0, 10.0));
        final var sources = List.of(ap1, ap2, ap3);

        // RSSI readings measured by the device, at an unknown location, from each access point
        final var readings = List.of(
                new RssiReading<>(new WifiAccessPoint("ap1", 2.4e9), -60.0),
                new RssiReading<>(new WifiAccessPoint("ap2", 2.4e9), -65.0),
                new RssiReading<>(new WifiAccessPoint("ap3", 2.4e9), -68.0));
        final var fingerprint = new RssiFingerprint<>(readings);

        final var estimator = new LinearRssiPositionEstimator2D(sources, fingerprint);
        estimator.estimate();

        final var estimatedPosition = estimator.getEstimatedPosition();
        System.out.println("Estimated position: " + estimatedPosition);
    }
}
```

See the [Antora documentation site](https://albertoirurueta.github.io/irurueta-navigation-indoor) for the
fingerprint-based and radio-source-estimation approaches, the path-loss distance model, and the robust (LMedS,
MSAC, PROSAC) estimator variants.

## License

This project is licensed under the [Apache License, Version 2.0](LICENSE.txt).
