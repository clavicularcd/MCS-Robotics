package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;

public class HuskyLensReader {

    private final HuskyLens huskyLens;

    // Optical constants for HuskyLens (OV2640)
    // F_PX = (F_mm * ImageWidth_px) / SensorWidth_mm = (4.6 * 320) / 3.52
    private static final double FOCAL_LENGTH_PX   = 418.18;
    private static final double REAL_TAG_WIDTH_MM  = 165.0; // 16.6cm DECODE tags - see https://ftc-resources.firstinspires.org/ftc/game/manual page 73
    private static final int    SCREEN_CENTER_X    = 160;
    private static final String[] PATTERN_IDS      = { "gpp", "pgp", "ppg" };

    // ── TagData inner class ───────────────────────────────────────────────────

    public static class TagData {
        public enum TagType { PATTERN, NAVIGATION, UNKNOWN }

        public TagType type;
        public int     id;
        public String  pattern;     // PATTERN tags only (IDs 1-3)
        public double  distanceMm;  // NAVIGATION tags only (IDs 4-5)
        public double  bearingDeg;
        public double[] vector;     // [X, Y] in mm

        @Override
        public String toString() {
            switch (type) {
                case PATTERN:    return "Pattern [" + id + "]: " + pattern;
                case NAVIGATION: return String.format("Nav [%d]: %.1fmm @ %.1f°", id, distanceMm, bearingDeg);
                default:         return "Unknown [" + id + "]";
            }
        }
    }

    // ── Constructor ───────────────────────────────────────────────────────────

    public HuskyLensReader(HardwareMap hwMap, String deviceName) {
        huskyLens = hwMap.get(HuskyLens.class, deviceName);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    /** Returns true if the HuskyLens is communicating correctly. */
    public boolean isConnected() {
        return huskyLens.knock();
    }

    // ── Main method ───────────────────────────────────────────────────────────

    /**
     * Call once per loop. Returns one TagData per visible tag.
     * Returns an empty list if nothing is detected.
     */
    public List<TagData> read() {
        List<TagData> results = new ArrayList<>();
        HuskyLens.Block[] blocks = huskyLens.blocks();

        for (HuskyLens.Block block : blocks) {
            TagData tag = new TagData();
            tag.id = block.id;

            if (block.id >= 1 && block.id <= 3) {
                tag.type    = TagData.TagType.PATTERN;
                tag.pattern = PATTERN_IDS[block.id - 1];

            } else if (block.id == 4 || block.id == 5) {
                tag.type       = TagData.TagType.NAVIGATION;
                tag.distanceMm = (REAL_TAG_WIDTH_MM * FOCAL_LENGTH_PX) / block.width;
                double offsetPx = block.x - SCREEN_CENTER_X;
                tag.bearingDeg = Math.toDegrees(Math.atan2(offsetPx, FOCAL_LENGTH_PX));
                tag.vector     = computeVector(tag.distanceMm, tag.bearingDeg);

            } else {
                tag.type = TagData.TagType.UNKNOWN;
            }

            results.add(tag);
        }
        return results;
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    private double[] computeVector(double dist, double angleDeg) {
        double rad = Math.toRadians(angleDeg);
        return new double[]{
                dist * Math.cos(rad), // X — forward component
                dist * Math.sin(rad)  // Y — lateral component
        };
    }
}
