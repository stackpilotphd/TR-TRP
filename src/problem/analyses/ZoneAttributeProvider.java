package problem.analyses;

import java.util.Objects;

/**
 * Supplies attributes for a zone identifier appearing in a tower route.
 */
public interface ZoneAttributeProvider {

    double weightOf(int zoneId);

    int priorityOf(int zoneId);

    static ZoneAttributeProvider fromArrays(
            double[] zoneWeights,
            int[] zonePriorities
    ) {
        Objects.requireNonNull(zoneWeights, "zoneWeights");
        Objects.requireNonNull(zonePriorities, "zonePriorities");

        return new ZoneAttributeProvider() {
            @Override
            public double weightOf(int zoneId) {
                validateZoneId(zoneId, zoneWeights.length, "weight");
                return zoneWeights[zoneId];
            }

            @Override
            public int priorityOf(int zoneId) {
                validateZoneId(zoneId, zonePriorities.length, "priority");
                return zonePriorities[zoneId];
            }
        };
    }

    private static void validateZoneId(
            int zoneId,
            int arrayLength,
            String attributeName
    ) {
        if (zoneId < 0 || zoneId >= arrayLength) {
            throw new IllegalArgumentException(
                    "Zone " + zoneId
                            + " is outside the " + attributeName
                            + " array of length " + arrayLength
            );
        }
    }
}
