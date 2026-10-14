package com.robotproject.server.routing;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Standalone proof of the obstruction lifecycle: FRESH -> STALE -> GONE ageing, the stale-probing
 * flag, passage verification, proximity corroboration, the world-event registry (moves, late and
 * out-of-order reports, tombstones), hygiene, history and concurrent use. Pure JVM; time is driven
 * by a MutableClock except in the concurrency case, which uses the real clock.
 */
public class ObstructionLifecycleDemo {

    private static final double EPS = 1e-6;
    private static final double INF = Double.POSITIVE_INFINITY;

    private static final Instant T0 = Instant.parse("2026-01-01T00:00:00Z");
    private static final MutableClock clock = new MutableClock(T0);
    private static final ExpiryPolicy P = policyP();

    // World points: A lies on the X-Y corridor, B on the X-Z corridor.
    private static final double AX = 5.0;
    private static final double AY = 0.1;
    private static final double BX = 5.0;
    private static final double BY = 5.1;

    private static int passCount = 0;
    private static int failCount = 0;

    private static ByteArrayOutputStream fullLog;
    private static PrintStream originalOut;
    private static PrintStream teeOut;

    public static void main(String[] args) throws InterruptedException {
        originalOut = System.out;
        fullLog = new ByteArrayOutputStream();
        teeOut = new PrintStream(new TeeOutputStream(originalOut, fullLog), true);
        System.setOut(teeOut);

        try {
            case1_stateMachine();
            case2_defaultOff();
            case3_kinds();
            case4_refreshAndConfirm();
            case5_orderingAndLateness();
            case6_penaltyComposition();
            case7_aStarCrossover();
            case8_passageVerification();
            case9_corroboration();
            case10_movingObstacle();
            case11_lateOlderOutOfOrder();
            case12_hygiene();
            case13_reportFields();
            case14_unknownWidth();
            case15_history();
            case16_concurrency();
        } finally {
            System.setOut(originalOut);
        }

        System.out.println();
        System.out.println("=== FINAL SUMMARY ===");
        System.out.println(passCount + " passed, " + failCount + " failed");
        if (failCount > 0) {
            System.exit(1);
        }
    }

    // ------------------------------------------------------------------
    // Case 1 - state machine (ON)
    // ------------------------------------------------------------------
    private static void case1_stateMachine() {
        header("CASE 1: state machine FRESH -> STALE -> GONE (stale probing ON)");
        clock.set(T0);
        RouteGraph g = g1();
        g.setStaleProbingEnabled(true);
        GraphEdge xy = g.getEdge("X", "Y");
        GraphEdge yx = g.getEdge("Y", "X");

        check("report 't1' accepted",
                xy.reportObstructionEvent("t1", 0.3, 0.5, ObstructionKind.TRANSIENT, null));

        probe("t=0", xy, yx, "t1", ExpiryPolicy.State.FRESH, 1.0, 0.0, 0.0, INF);
        at(5);
        probe("t=5s", xy, yx, "t1", ExpiryPolicy.State.FRESH, 1.0, 0.0, 0.0, INF);
        at(6);
        probe("t=6s", xy, yx, "t1", ExpiryPolicy.State.STALE, 0.95, 1.0, 19.0, 29.0);
        at(15);
        probe("t=15s", xy, yx, "t1", ExpiryPolicy.State.STALE, 0.5, 1.0, 10.0, 20.0);
        at(25);
        probe("t=25s", xy, yx, "t1", ExpiryPolicy.State.GONE, 0.0, 1.0, 0.0, 10.0);

        check("t=25s: physicalObstructionCount()==1 before purging (got "
                + xy.physicalObstructionCount() + ")", xy.physicalObstructionCount() == 1);
        List<String> purged = xy.purgeExpiredObstructions();
        check("purgeExpiredObstructions() returns [t1] (got " + purged + ")",
                purged.size() == 1 && "t1".equals(purged.get(0)));
        check("physicalObstructionCount()==0 after purging (got "
                + xy.physicalObstructionCount() + ")", xy.physicalObstructionCount() == 0);
    }

    // ------------------------------------------------------------------
    // Case 2 - default OFF
    // ------------------------------------------------------------------
    private static void case2_defaultOff() {
        header("CASE 2: stale probing OFF by default - a STALE obstruction keeps blocking until its hard TTL");
        clock.set(T0);
        RouteGraph g = g1();
        check("a fresh graph has isStaleProbingEnabled()==false", !g.isStaleProbingEnabled());
        GraphEdge xy = g.getEdge("X", "Y");
        xy.reportObstructionEvent("t1", 0.3, 0.5, ObstructionKind.TRANSIENT, null);

        at(6);
        checkState("t=6s: view says STALE", xy, "t1", ExpiryPolicy.State.STALE);
        checkNear("t=6s: cap still 0.0", cap(xy), 0.0);
        checkNear("t=6s: cost still INFINITY", cost(xy), INF);
        checkNear("t=6s: penalty 0", penalty(xy), 0.0);

        at(15);
        checkPath("t=15s: findPath(X,Y) avoids the stale obstruction", g, "X", "Z", "Y");

        at(24);
        checkState("t=24s: view says STALE", xy, "t1", ExpiryPolicy.State.STALE);
        checkNear("t=24s: still blocking, cap 0.0", cap(xy), 0.0);

        at(25);
        checkNear("t=25s: cap 1.0", cap(xy), 1.0);
        checkNear("t=25s: cost 10.0", cost(xy), 10.0);
        checkPath("t=25s: findPath(X,Y) is direct", g, "X", "Y");

        // Passage sub-check on a second graph: the ON call removes the event, which would
        // invalidate the t=24s/t=25s checks above if run on the same graph.
        clock.set(T0);
        RouteGraph g2 = g1();
        GraphEdge xy2 = g2.getEdge("X", "Y");
        xy2.reportObstructionEvent("t1", 0.3, 0.5, ObstructionKind.TRANSIENT, null);
        at(15);
        List<String> whileOff = xy2.clearStaleObstructionsPassedBy(0.99);
        check("t=15s, OFF: clearStaleObstructionsPassedBy(0.99) returns an empty list (got "
                + whileOff + ")", whileOff.isEmpty());
        g2.setStaleProbingEnabled(true);
        List<String> whileOn = xy2.clearStaleObstructionsPassedBy(0.99);
        check("t=15s, ON: the same call returns [t1] (got " + whileOn + ")",
                whileOn.size() == 1 && "t1".equals(whileOn.get(0)));
    }

    // ------------------------------------------------------------------
    // Case 3 - kinds
    // ------------------------------------------------------------------
    private static void case3_kinds() {
        header("CASE 3: each kind selects its own lifetime");

        clock.set(T0);
        GraphEdge perm = g1().getEdge("X", "Y");
        perm.reportObstructionEvent("perm", 0.3, 0.5, ObstructionKind.PERMANENT, null);
        clock.set(T0.plus(Duration.ofDays(400)));
        checkState("PERMANENT after +400 days: FRESH", perm, "perm", ExpiryPolicy.State.FRESH);
        checkNear("PERMANENT after +400 days: cap 0.0", cap(perm), 0.0);

        clock.set(T0);
        GraphEdge semi = g1().getEdge("X", "Y");
        semi.reportObstructionEvent("semi", 0.3, 0.5, ObstructionKind.SEMI_PERMANENT, null);
        at(59);
        checkState("SEMI_PERMANENT at +59s: FRESH", semi, "semi", ExpiryPolicy.State.FRESH);
        at(61);
        checkState("SEMI_PERMANENT at +61s: STALE", semi, "semi", ExpiryPolicy.State.STALE);

        clock.set(T0);
        GraphEdge unk = g1().getEdge("X", "Y");
        unk.reportObstructionEvent("unk", 0.3, 0.5, null, null);
        ObstructionView unkView = view(unk, "unk");
        check("kind null is recorded as UNKNOWN (got " + (unkView == null ? "no view" : unkView.kind) + ")",
                unkView != null && unkView.kind == ObstructionKind.UNKNOWN);
        at(10);
        checkState("UNKNOWN at +10s: FRESH", unk, "unk", ExpiryPolicy.State.FRESH);
        at(11);
        checkState("UNKNOWN at +11s: STALE", unk, "unk", ExpiryPolicy.State.STALE);
        at(100);
        checkState("UNKNOWN at +100s: GONE", unk, "unk", ExpiryPolicy.State.GONE);
    }

    // ------------------------------------------------------------------
    // Case 4 - refresh and confirm (ON)
    // ------------------------------------------------------------------
    private static void case4_refreshAndConfirm() {
        header("CASE 4: re-report and confirm bring a STALE obstruction back to FRESH (stale probing ON)");
        clock.set(T0);
        RouteGraph g = g1();
        g.setStaleProbingEnabled(true);
        GraphEdge xy = g.getEdge("X", "Y");

        xy.reportObstructionEvent("t1", 0.3, 0.5, ObstructionKind.TRANSIENT, null);
        at(6);
        checkState("+6s: STALE", xy, "t1", ExpiryPolicy.State.STALE);
        checkNear("+6s: cap 1.0", cap(xy), 1.0);

        check("re-report 't1' (TRANSIENT, observedAt null) is accepted",
                xy.reportObstructionEvent("t1", 0.3, 0.5, ObstructionKind.TRANSIENT, null));
        checkNear("after the re-report: cap 0.0", cap(xy), 0.0);

        at(12);
        checkState("+6s after the re-report: STALE", xy, "t1", ExpiryPolicy.State.STALE);
        checkNear("+6s after the re-report: cap 1.0", cap(xy), 1.0);

        check("confirmObstructionEvent(\"t1\") is true", xy.confirmObstructionEvent("t1"));
        checkNear("after confirm: cap 0.0", cap(xy), 0.0);
        check("confirmObstructionEvent(\"nope\") is false", !xy.confirmObstructionEvent("nope"));
    }

    // ------------------------------------------------------------------
    // Case 5 - ordering and lateness at edge level
    // ------------------------------------------------------------------
    private static void case5_orderingAndLateness() {
        header("CASE 5: ordering and lateness at edge level (clock T0+200s)");
        at(200);
        RouteGraph g = g1();
        GraphEdge xy = g.getEdge("X", "Y");

        check("'o1' width 0.3 observedAt T0+190s is accepted",
                xy.reportObstructionEvent("o1", 0.3, 0.5, ObstructionKind.SEMI_PERMANENT, T0.plusSeconds(190)));
        checkNear("cap 0.0", cap(xy), 0.0);

        check("'o1' width 1.0 observedAt T0+150s (older) is refused",
                !xy.reportObstructionEvent("o1", 1.0, 0.5, ObstructionKind.SEMI_PERMANENT, T0.plusSeconds(150)));
        checkNear("cap still 0.0", cap(xy), 0.0);

        check("'o1' width 1.0 observedAt T0+195s (newer) is accepted",
                xy.reportObstructionEvent("o1", 1.0, 0.5, ObstructionKind.SEMI_PERMANENT, T0.plusSeconds(195)));
        checkNear("cap 1.0", cap(xy), 1.0);

        int countBefore = xy.physicalObstructionCount();
        check("TRANSIENT 'late' observedAt T0+100s (already GONE) is refused",
                !xy.reportObstructionEvent("late", 0.3, 0.5, ObstructionKind.TRANSIENT, T0.plusSeconds(100)));
        check("physical count unchanged (got " + xy.physicalObstructionCount() + ", was " + countBefore + ")",
                xy.physicalObstructionCount() == countBefore);

        check("SEMI_PERMANENT 'fut' observedAt now+1h is accepted",
                xy.reportObstructionEvent("fut", 0.3, 0.5, ObstructionKind.SEMI_PERMANENT,
                        clock.instant().plus(Duration.ofHours(1))));
        at(200 + 30);
        checkState("'fut' at +30s: FRESH (stamp clamped to now)", xy, "fut", ExpiryPolicy.State.FRESH);
        at(200 + 61);
        checkState("'fut' at +61s: STALE (proves the stamp was clamped, not kept 1h ahead)",
                xy, "fut", ExpiryPolicy.State.STALE);
    }

    // ------------------------------------------------------------------
    // Case 6 - penalty composition (ON)
    // ------------------------------------------------------------------
    private static void case6_penaltyComposition() {
        header("CASE 6: penalty is the worst STALE expected-encounter cost (stale probing ON, clock T0+100s)");
        at(100);
        RouteGraph g = g1();
        g.setStaleProbingEnabled(true);
        GraphEdge xy = g.getEdge("X", "Y");

        Instant now = clock.instant();
        xy.reportObstructionEvent("e1", 0.3, 0.5, ObstructionKind.TRANSIENT, now.minusSeconds(7));
        xy.reportObstructionEvent("e2", 0.75, 0.5, ObstructionKind.TRANSIENT, now.minusSeconds(17));

        ObstructionView e1 = view(xy, "e1");
        ObstructionView e2 = view(xy, "e2");
        checkNear("'e1' existence probability", e1 == null ? Double.NaN : e1.existenceProbability, 0.9);
        checkNear("'e2' existence probability", e2 == null ? Double.NaN : e2.existenceProbability, 0.4);
        checkNear("cap 1.0", cap(xy), 1.0);
        checkNear("penalty = max(20*0.9*1, 20*0.4*0.5) = 18.0", penalty(xy), 18.0);
        checkNear("cost 28.0", cost(xy), 28.0);
    }

    // ------------------------------------------------------------------
    // Case 7 - A* crossover (ON)
    // ------------------------------------------------------------------
    private static void case7_aStarCrossover() {
        header("CASE 7: A* switches to the stale route once its expected cost beats the detour (stale probing ON)");
        clock.set(T0);
        RouteGraph g = g1();
        g.setStaleProbingEnabled(true);
        GraphEdge xy = g.getEdge("X", "Y");
        xy.reportObstructionEvent("x", 0.3, 0.5, ObstructionKind.TRANSIENT, null);

        checkPath("t=0: [X,Z,Y]", g, "X", "Z", "Y");
        at(8);
        checkNear("t=8s: direct cost 27.0 (p 0.85)", cost(xy), 27.0);
        checkPath("t=8s: [X,Z,Y]", g, "X", "Z", "Y");
        at(15);
        checkNear("t=15s: direct cost 20.0 (p 0.5)", cost(xy), 20.0);
        checkPath("t=15s: [X,Y]", g, "X", "Y");
        at(26);
        checkPath("t=26s (GONE): [X,Y]", g, "X", "Y");
    }

    // ------------------------------------------------------------------
    // Case 8 - passage verification (ON)
    // ------------------------------------------------------------------
    private static void case8_passageVerification() {
        header("CASE 8a: a robot passing a STALE obstruction's position clears it (stale probing ON)");
        clock.set(T0);
        RouteGraph g = g1();
        g.setStaleProbingEnabled(true);
        MissionManager mm = new MissionManager(g, new AStarRouter());
        GraphEdge xy = g.getEdge("X", "Y");

        mm.reportEdgeObstruction("X", "Y", "probe", 0.3, 0.5, ObstructionKind.TRANSIENT, null);
        at(15);
        mm.onTelemetry("r", 0, 0, "Y");
        GraphNode initial = mm.initialTargetFor("r");
        check("initialTargetFor(r) is Y - the route through the stale obstruction (20.0 < 24.14) is chosen (got "
                + describe(initial) + ")", initial != null && "Y".equals(initial.getId()));

        GraphNode at3 = mm.onTelemetry("r", 3, 0, "Y");
        check("onTelemetry(r, 3, 0) returns null", at3 == null);
        check("the edge still has 1 view (got " + states(xy) + ")", states(xy).size() == 1);
        checkState("that view is STALE", xy, "probe", ExpiryPolicy.State.STALE);

        int before = fullLog.size();
        mm.onTelemetry("r", 6, 0, "Y");
        check("onTelemetry(r, 6, 0) clears it: 0 views (got " + states(xy) + ")", states(xy).isEmpty());
        check("log contains \"Stale obstruction 'probe' cleared\"",
                newSegment(before).contains("Stale obstruction 'probe' cleared"));

        header("CASE 8b: arriving at the far node does NOT clear an obstruction recorded at 0.995");
        clock.set(T0);
        RouteGraph gb = g1();
        gb.setStaleProbingEnabled(true);
        MissionManager mmb = new MissionManager(gb, new AStarRouter());
        GraphEdge xyb = gb.getEdge("X", "Y");
        mmb.reportEdgeObstruction("X", "Y", "probe", 0.3, 0.995, ObstructionKind.TRANSIENT,
                clock.instant().minusSeconds(15));
        mmb.onTelemetry("r", 0, 0, "Y");
        before = fullLog.size();
        mmb.onTelemetry("r", 9.9, 0, "Y");
        check("the mission arrives (log contains \"confirmed at Y\")",
                newSegment(before).contains("confirmed at Y"));
        check("the event is still present: 1 view (got " + states(xyb) + ")", states(xyb).size() == 1);
        checkState("that view is STALE - arrival alone must not clear it", xyb, "probe", ExpiryPolicy.State.STALE);
        at(11);
        checkState("+11s: the view state is GONE", xyb, "probe", ExpiryPolicy.State.GONE);
        mmb.purgeExpiredObstructions();
        check("after mm.purgeExpiredObstructions(): physicalObstructionCount()==0 (got "
                + xyb.physicalObstructionCount() + ")", xyb.physicalObstructionCount() == 0);

        header("CASE 8c: progress strictly beyond 0.995 clears it");
        clock.set(T0);
        RouteGraph gc = g1();
        gc.setStaleProbingEnabled(true);
        MissionManager mmc = new MissionManager(gc, new AStarRouter());
        GraphEdge xyc = gc.getEdge("X", "Y");
        mmc.reportEdgeObstruction("X", "Y", "probe", 0.3, 0.995, ObstructionKind.TRANSIENT,
                clock.instant().minusSeconds(15));
        mmc.onTelemetry("r", 0, 0, "Y");
        before = fullLog.size();
        mmc.onTelemetry("r", 9.99, 0, "Y");
        String segment = newSegment(before);
        check("log contains \"Stale obstruction\" and \"cleared\"",
                segment.contains("Stale obstruction") && segment.contains("cleared"));
        check("the edge has 0 views (got " + states(xyc) + ")", states(xyc).isEmpty());
    }

    // ------------------------------------------------------------------
    // Case 9 - corroboration (ON)
    // ------------------------------------------------------------------
    private static void case9_corroboration() {
        header("CASE 9: a matching proximity reading re-confirms a STALE obstruction ahead (stale probing ON)");
        clock.set(T0);
        RouteGraph g = g1();
        g.setStaleProbingEnabled(true);
        MissionManager mm = new MissionManager(g, new AStarRouter());
        GraphEdge xy = g.getEdge("X", "Y");
        mm.reportEdgeObstruction("X", "Y", "probe", 0.3, 0.5, ObstructionKind.TRANSIENT,
                clock.instant().minusSeconds(15));
        mm.onTelemetry("r", 0, 0, "Y");
        GraphNode initial = mm.initialTargetFor("r");
        check("the direct mission is created (target Y, got " + describe(initial) + ")",
                initial != null && "Y".equals(initial.getId()));

        int before = fullLog.size();
        mm.observeProximity("r", 3.0, 0.0, 2.2);
        checkState("sensed 2.2m vs predicted 2.0m: the event becomes FRESH", xy, "probe", ExpiryPolicy.State.FRESH);
        check("log contains \"retreating to X\"", newSegment(before).contains("retreating to X"));
        GraphNode retreat = mm.onTelemetry("r", 3, 0, "Y");
        check("onTelemetry(r, 3, 0) returns X (got " + describe(retreat) + ")",
                retreat != null && "X".equals(retreat.getId()));

        header("CASE 9 negative: a reading that does not match leaves it STALE");
        clock.set(T0);
        RouteGraph gn = g1();
        gn.setStaleProbingEnabled(true);
        MissionManager mmn = new MissionManager(gn, new AStarRouter());
        GraphEdge xyn = gn.getEdge("X", "Y");
        mmn.reportEdgeObstruction("X", "Y", "probe", 0.3, 0.5, ObstructionKind.TRANSIENT,
                clock.instant().minusSeconds(15));
        mmn.onTelemetry("r", 0, 0, "Y");
        mmn.observeProximity("r", 3.0, 0.0, 8.0);
        checkState("sensed 8.0m: the event stays STALE", xyn, "probe", ExpiryPolicy.State.STALE);
        GraphNode noTarget = mmn.onTelemetry("r", 3, 0, "Y");
        check("onTelemetry(r, 3, 0) returns null (got " + describe(noTarget) + ")", noTarget == null);

        header("CASE 9 negative: an obstruction behind the robot is not corroborated");
        clock.set(T0);
        RouteGraph gb = g1();
        gb.setStaleProbingEnabled(true);
        MissionManager mmb = new MissionManager(gb, new AStarRouter());
        GraphEdge xyb = gb.getEdge("X", "Y");
        mmb.reportEdgeObstruction("X", "Y", "behind", 0.3, 0.2, ObstructionKind.TRANSIENT,
                clock.instant().minusSeconds(15));
        mmb.onTelemetry("r", 0, 0, "Y");
        before = fullLog.size();
        mmb.observeProximity("r", 3.0, 0.0, 2.0);
        checkState("event at 0.2 (robot at 0.3), sensed 2.0m: stays STALE", xyb, "behind", ExpiryPolicy.State.STALE);
        check("log does not contain \"corroborated\"", !newSegment(before).contains("corroborated"));
    }

    // ------------------------------------------------------------------
    // Case 10 - moving obstacle
    // ------------------------------------------------------------------
    private static void case10_movingObstacle() {
        header("CASE 10: a moving obstacle re-reported on another corridor leaves no phantom behind");
        clock.set(T0);
        RouteGraph g = g2();
        MissionManager mm = new MissionManager(g, new AStarRouter());
        GraphEdge xy = g.getEdge("X", "Y");
        GraphEdge xz = g.getEdge("X", "Z");

        int before = fullLog.size();
        mm.reportWorldObstruction(new WorldObstructionReport("mv", AX, AY, 0.8, null,
                ObstructionKind.TRANSIENT, null));
        check("log contains \"mapped to edge\"", newSegment(before).contains("mapped to edge"));
        checkNear("at A: cap(X->Y) 0.0", cap(xy), 0.0);

        before = fullLog.size();
        mm.reportWorldObstruction(new WorldObstructionReport("mv", BX, BY, 0.8, null,
                ObstructionKind.TRANSIENT, null));
        checkNear("at B: cap(X->Y) 1.0", cap(xy), 1.0);
        checkNear("at B: cap(X->Z) 0.0", cap(xz), 0.0);
        check("log contains \"moved from corridor\"", newSegment(before).contains("moved from corridor"));
        check("trackedWorldObstructionCount()==1 (got " + mm.trackedWorldObstructionCount() + ")",
                mm.trackedWorldObstructionCount() == 1);
        int views = xy.getObstructionViews().size() + xz.getObstructionViews().size();
        check("the X-Y and X-Z corridors together hold exactly 1 view (got " + views + ")", views == 1);
    }

    // ------------------------------------------------------------------
    // Case 11 - late, older and out-of-order reports
    // ------------------------------------------------------------------
    private static void case11_lateOlderOutOfOrder() {
        header("CASE 11: late, older and out-of-order world reports and clears (clock T0+100s)");
        at(100);
        RouteGraph g = g2();
        MissionManager mm = new MissionManager(g, new AStarRouter());
        GraphEdge xy = g.getEdge("X", "Y");
        GraphEdge xz = g.getEdge("X", "Z");

        mm.reportWorldObstruction(semi("m1", AX, AY, T0.plusSeconds(90)));
        checkNear("1. 'm1' at A @90s accepted: cap(X->Y) 0.0", cap(xy), 0.0);

        mm.reportWorldObstruction(semi("m1", BX, BY, T0.plusSeconds(95)));
        checkNear("2. 'm1' at B @95s moved: cap(X->Y) 1.0", cap(xy), 1.0);
        checkNear("2. cap(X->Z) 0.0", cap(xz), 0.0);

        int before = fullLog.size();
        mm.reportWorldObstruction(semi("m1", AX, AY, T0.plusSeconds(92)));
        check("3. 'm1' at A @92s (older): log contains \"ignored\"", newSegment(before).contains("ignored"));
        checkNear("3. X->Y stays 1.0", cap(xy), 1.0);
        checkNear("3. X->Z stays 0.0", cap(xz), 0.0);
        checkCount("3. live count", mm.trackedWorldObstructionCount(), 1);

        before = fullLog.size();
        mm.clearWorldObstruction("m1", T0.plusSeconds(94));
        check("4. clear @94s (older than the latest sighting): ignored", newSegment(before).contains("ignored"));
        checkNear("4. X->Z still 0.0", cap(xz), 0.0);

        mm.clearWorldObstruction("m1", T0.plusSeconds(96));
        checkNear("5. clear @96s: X->Z 1.0", cap(xz), 1.0);
        checkCount("5. live count", mm.trackedWorldObstructionCount(), 0);
        checkCount("5. trackedTombstoneCount()", mm.trackedTombstoneCount(), 1);

        before = fullLog.size();
        mm.reportWorldObstruction(semi("m1", BX, BY, T0.plusSeconds(96)));
        check("6. 'm1' at B @96s (tie with the clear): ignored", newSegment(before).contains("ignored"));
        checkNear("6. X->Z stays 1.0", cap(xz), 1.0);

        mm.reportWorldObstruction(semi("m1", BX, BY, T0.plusSeconds(97)));
        checkNear("7. 'm1' at B @97s accepted: X->Z 0.0", cap(xz), 0.0);
        checkCount("7. tombstone count", mm.trackedTombstoneCount(), 0);
        checkCount("7. live count", mm.trackedWorldObstructionCount(), 1);

        mm.clearWorldObstruction("n1", T0.plusSeconds(98));
        checkCount("8. clear of never-seen 'n1' @98s: tombstone count", mm.trackedTombstoneCount(), 1);
        before = fullLog.size();
        mm.reportWorldObstruction(semi("n1", AX, AY, T0.plusSeconds(97)));
        check("8. 'n1' at A @97s (older than the clear): ignored", newSegment(before).contains("ignored"));
        checkNear("8. cap(X->Y) 1.0", cap(xy), 1.0);
        mm.reportWorldObstruction(semi("n1", AX, AY, T0.plusSeconds(99)));
        checkNear("8. 'n1' at A @99s accepted: cap(X->Y) 0.0", cap(xy), 0.0);
    }

    // ------------------------------------------------------------------
    // Case 12 - hygiene
    // ------------------------------------------------------------------
    private static void case12_hygiene() {
        header("CASE 12: purging drops expired events and old tombstones");
        clock.set(T0);
        RouteGraph g = g2();
        MissionManager mm = new MissionManager(g, new AStarRouter());
        GraphEdge xy = g.getEdge("X", "Y");

        mm.reportWorldObstruction(new WorldObstructionReport("w1", AX, AY, 0.8, null,
                ObstructionKind.TRANSIENT, null));
        checkCount("after reporting 'w1': live count", mm.trackedWorldObstructionCount(), 1);

        at(26);
        mm.purgeExpiredObstructions();
        checkCount("+26s, after purge: live count", mm.trackedWorldObstructionCount(), 0);
        checkCount("+26s, after purge: physicalObstructionCount()", xy.physicalObstructionCount(), 0);

        mm.clearWorldObstruction("z9", null);
        checkCount("after clearing never-seen 'z9': tombstone count", mm.trackedTombstoneCount(), 1);
        clock.advance(Duration.ofMinutes(11));
        mm.purgeExpiredObstructions();
        checkCount("+11 minutes, after purge: tombstone count", mm.trackedTombstoneCount(), 0);
    }

    // ------------------------------------------------------------------
    // Case 13 - report fields (ON)
    // ------------------------------------------------------------------
    private static void case13_reportFields() {
        header("CASE 13a: observedAt in the past - STALE at birth (stale probing ON)");
        clock.set(T0);
        RouteGraph g = g2();
        g.setStaleProbingEnabled(true);
        MissionManager mm = new MissionManager(g, new AStarRouter());
        GraphEdge xy = g.getEdge("X", "Y");
        GraphEdge yx = g.getEdge("Y", "X");

        mm.reportWorldObstruction(new WorldObstructionReport("f1", 5.0, 0.0, 0.8, null,
                ObstructionKind.TRANSIENT, clock.instant().minusSeconds(10)));
        ObstructionView v = view(xy, "f1");
        check("STALE at birth (got " + (v == null ? "no view" : v.state) + ")",
                v != null && v.state == ExpiryPolicy.State.STALE);
        checkNear("remaining width 0.4", v == null ? Double.NaN : v.remainingWidthMeters, 0.4);
        checkNear("existence probability 0.75", v == null ? Double.NaN : v.existenceProbability, 0.75);
        checkNear("cap 1.0", cap(xy), 1.0);
        checkNear("penalty 15.0", penalty(xy), 15.0);
        checkNear("cost 25.0", cost(xy), 25.0);
        checkNear("Y->X cap identical", cap(yx), cap(xy));
        checkNear("Y->X penalty identical", penalty(yx), penalty(xy));
        checkNear("Y->X cost identical", cost(yx), cost(xy));

        header("CASE 13b: a measured free width overrides structural-minus-obstacle (stale probing ON)");
        clock.set(T0);
        RouteGraph gb = g2();
        gb.setStaleProbingEnabled(true);
        MissionManager mmb = new MissionManager(gb, new AStarRouter());
        GraphEdge xyb = gb.getEdge("X", "Y");
        mmb.reportWorldObstruction(new WorldObstructionReport("f2", 5.0, 0.0, 0.1, 0.5, null, null));
        ObstructionView vb = view(xyb, "f2");
        checkNear("remaining width 0.5 (not 1.2 - 0.1 = 1.1)",
                vb == null ? Double.NaN : vb.remainingWidthMeters, 0.5);
        checkNear("cap 0.0", cap(xyb), 0.0);
        check("state FRESH (got " + (vb == null ? "no view" : vb.state) + ")",
                vb != null && vb.state == ExpiryPolicy.State.FRESH);
    }

    // ------------------------------------------------------------------
    // Case 14 - unknown width
    // ------------------------------------------------------------------
    private static void case14_unknownWidth() {
        header("CASE 14: unknown corridor width uses an explicit, configurable assumption");
        clock.set(T0);

        RouteGraph ga = g1();
        MissionManager mma = new MissionManager(ga, new AStarRouter());
        int before = fullLog.size();
        mma.reportWorldObstruction(WorldObstructionReport.basic("u1", 5.0, 0.0, 1.0));
        check("(a) log contains \"assuming a 1.5m corridor\"",
                newSegment(before).contains("assuming a 1.5m corridor"));
        checkNear("(a) cap(X->Y) 0.0 (remaining 0.5)", cap(ga.getEdge("X", "Y")), 0.0);

        RouteGraph gb = g1();
        MissionManager mmb = new MissionManager(gb, new AStarRouter());
        before = fullLog.size();
        mmb.reportWorldObstruction(WorldObstructionReport.basic("u2", 5.0, 0.0, 0.2));
        check("(b) log contains \"assuming\"", newSegment(before).contains("assuming"));
        checkNear("(b) cap(X->Y) 1.0 (remaining 1.3)", cap(gb.getEdge("X", "Y")), 1.0);

        RouteGraph gc = g1();
        gc.setAssumedUnknownCorridorWidthMeters(3.0);
        MissionManager mmc = new MissionManager(gc, new AStarRouter());
        mmc.reportWorldObstruction(WorldObstructionReport.basic("u3", 5.0, 0.0, 1.0));
        checkNear("(c) assumed width 3.0, obstacle 1.0: cap(X->Y) 1.0", cap(gc.getEdge("X", "Y")), 1.0);

        RouteGraph gd = g2();
        MissionManager mmd = new MissionManager(gd, new AStarRouter());
        before = fullLog.size();
        mmd.reportWorldObstruction(WorldObstructionReport.basic("u4", 5.0, 0.0, 0.8));
        check("(d) known width 1.2: log does NOT contain \"assuming\"",
                !newSegment(before).contains("assuming"));
        checkNear("(d) cap(X->Y) 0.0", cap(gd.getEdge("X", "Y")), 0.0);

        RouteGraph ge = g1();
        boolean threw = false;
        try {
            ge.setAssumedUnknownCorridorWidthMeters(0);
        } catch (IllegalArgumentException expected) {
            threw = true;
        }
        check("(e) setAssumedUnknownCorridorWidthMeters(0) throws IllegalArgumentException", threw);
    }

    // ------------------------------------------------------------------
    // Case 15 - history (ON)
    // ------------------------------------------------------------------
    private static void case15_history() {
        header("CASE 15: bounded history records why each obstruction ended (stale probing ON)");
        clock.set(T0);
        RouteGraph g = g2();
        g.setStaleProbingEnabled(true);
        GraphEdge xy = g.getEdge("X", "Y");

        xy.reportObstructionEvent("h-exp", 0.3, 0.5, ObstructionKind.TRANSIENT, null);
        clock.advance(Duration.ofSeconds(26));
        xy.purgeExpiredObstructions();
        check("(a) history contains (h-exp, EXPIRED)",
                historyContains(g, "h-exp", ObstructionHistoryEntry.EndReason.EXPIRED));

        xy.reportObstructionEvent("h-clr", 0.3);
        xy.clearObstructionEvent("h-clr");
        check("(b) history contains (h-clr, CLEARED)",
                historyContains(g, "h-clr", ObstructionHistoryEntry.EndReason.CLEARED));

        xy.reportObstructionEvent("h-pas", 0.3, 0.5, ObstructionKind.TRANSIENT, clock.instant().minusSeconds(15));
        List<String> passed = xy.clearStaleObstructionsPassedBy(0.9);
        check("(c) clearStaleObstructionsPassedBy(0.9) returns [h-pas] (got " + passed + ")",
                passed.size() == 1 && "h-pas".equals(passed.get(0)));
        check("(c) history contains (h-pas, PASSED)",
                historyContains(g, "h-pas", ObstructionHistoryEntry.EndReason.PASSED));

        MissionManager mm = new MissionManager(g, new AStarRouter());
        mm.reportWorldObstruction(new WorldObstructionReport("mv", AX, AY, 0.8, null,
                ObstructionKind.TRANSIENT, null));
        mm.reportWorldObstruction(new WorldObstructionReport("mv", BX, BY, 0.8, null,
                ObstructionKind.TRANSIENT, null));
        check("(d) history contains (mv, MOVED)",
                historyContains(g, "mv", ObstructionHistoryEntry.EndReason.MOVED));

        for (int i = 0; i < 600; i++) {
            xy.reportObstructionEvent("b" + i, 0.3);
            xy.clearObstructionEvent("b" + i);
        }
        List<ObstructionHistoryEntry> history = g.getObstructionHistory();
        checkCount("(e) after 600 report/clear pairs: history size", history.size(), 500);
        String last = history.isEmpty() ? "none" : history.get(history.size() - 1).eventId;
        check("(e) the last entry is b599 (got " + last + ")", "b599".equals(last));
    }

    // ------------------------------------------------------------------
    // Case 16 - concurrency (REAL clock)
    // ------------------------------------------------------------------
    private static void case16_concurrency() throws InterruptedException {
        header("CASE 16: concurrent world reports vs. A* reads (real clock, ~1s)");
        RouteGraph g = triangle(1.2); // REAL clock: setClock deliberately not called
        MissionManager mm = new MissionManager(g, new AStarRouter());
        AStarRouter router = new AStarRouter();
        GraphEdge xy = g.getEdge("X", "Y");
        GraphEdge xz = g.getEdge("X", "Z");

        AtomicBoolean writerDone = new AtomicBoolean(false);
        AtomicReference<Throwable> writerError = new AtomicReference<>();
        AtomicReference<Throwable> readerError = new AtomicReference<>();
        AtomicReference<String> badPath = new AtomicReference<>();
        AtomicReference<Double> badCost = new AtomicReference<>();
        AtomicInteger writes = new AtomicInteger();
        AtomicInteger reads = new AtomicInteger();
        AtomicInteger emptyPaths = new AtomicInteger();
        AtomicInteger nonEmptyPaths = new AtomicInteger();

        Thread writer = new Thread(() -> {
            try {
                long end = System.nanoTime() + 1_000_000_000L;
                boolean onA = true;
                while (System.nanoTime() < end) {
                    mm.reportWorldObstruction(WorldObstructionReport.basic(
                            "mv", onA ? AX : BX, onA ? AY : BY, 0.8));
                    writes.incrementAndGet();
                    onA = !onA;
                }
            } catch (Throwable t) {
                writerError.set(t);
            } finally {
                writerDone.set(true);
            }
        }, "case16-writer");

        Thread reader = new Thread(() -> {
            try {
                while (!writerDone.get()) {
                    List<GraphNode> path = router.findPath(g, "X", "Y");
                    if (path.isEmpty()) {
                        emptyPaths.incrementAndGet();
                    } else {
                        nonEmptyPaths.incrementAndGet();
                        if (!"X".equals(path.get(0).getId())
                                || !"Y".equals(path.get(path.size() - 1).getId())) {
                            badPath.compareAndSet(null, pathIds(path));
                        }
                    }
                    double c = g.getEdge("X", "Y").effectiveCost();
                    if (!(c == 10.0 || c == INF)) {
                        badCost.compareAndSet(null, c);
                    }
                    reads.incrementAndGet();
                }
            } catch (Throwable t) {
                readerError.set(t);
            }
        }, "case16-reader");

        System.setOut(new PrintStream(OutputStream.nullOutputStream()));
        try {
            reader.start();
            writer.start();
            writer.join();
            reader.join();
        } finally {
            System.setOut(teeOut);
        }

        System.out.println("writer reports: " + writes.get() + ", reader iterations: " + reads.get()
                + ", empty paths seen: " + emptyPaths.get() + ", non-empty paths seen: " + nonEmptyPaths.get());

        check("no exception in the writer thread (got " + writerError.get() + ")", writerError.get() == null);
        check("no exception in the reader thread (got " + readerError.get() + ")", readerError.get() == null);
        check("every path is empty, or non-empty from X to Y (first bad path: " + badPath.get() + ")",
                badPath.get() == null);
        check("every X->Y cost is exactly 10.0 or +INFINITY (first bad cost: " + badCost.get() + ")",
                badCost.get() == null);
        check("the reader saw at least one non-empty path (got " + nonEmptyPaths.get() + ")",
                nonEmptyPaths.get() > 0);
        checkCount("at the end: trackedWorldObstructionCount()", mm.trackedWorldObstructionCount(), 1);
        int views = xy.getObstructionViews().size() + xz.getObstructionViews().size();
        checkCount("at the end: views across the X-Y and X-Z corridors", views, 1);
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------
    private static ExpiryPolicy policyP() {
        Map<ObstructionKind, ExpiryPolicy.Lifetime> m = new EnumMap<>(ObstructionKind.class);
        m.put(ObstructionKind.TRANSIENT,
                ExpiryPolicy.Lifetime.of(Duration.ofSeconds(5), Duration.ofSeconds(25)));
        m.put(ObstructionKind.SEMI_PERMANENT,
                ExpiryPolicy.Lifetime.of(Duration.ofSeconds(60), Duration.ofSeconds(600)));
        m.put(ObstructionKind.PERMANENT, ExpiryPolicy.Lifetime.permanent());
        m.put(ObstructionKind.UNKNOWN,
                ExpiryPolicy.Lifetime.of(Duration.ofSeconds(10), Duration.ofSeconds(100)));
        return new ExpiryPolicy(m, 20.0);
    }

    /** X(0,0), Y(10,0), Z(10,10); bidirectional X-Y, X-Z, Z-Y; policy P; clock NOT set. */
    private static RouteGraph triangle(Double widthMeters) {
        RouteGraph g = new RouteGraph();
        g.addNode(new GraphNode("X", 0.0, 0.0));
        g.addNode(new GraphNode("Y", 10.0, 0.0));
        g.addNode(new GraphNode("Z", 10.0, 10.0));
        double w = widthMeters == null ? GraphEdge.UNCONSTRAINED_WIDTH_METERS : widthMeters;
        g.addEdge("X", "Y", true, w);
        g.addEdge("X", "Z", true, w);
        g.addEdge("Z", "Y", true, w);
        g.setExpiryPolicy(P);
        return g;
    }

    /** G1: unconstrained width, shared MutableClock. */
    private static RouteGraph g1() {
        RouteGraph g = triangle(null);
        g.setClock(clock);
        return g;
    }

    /** G2: width 1.2 on all three corridors, shared MutableClock. */
    private static RouteGraph g2() {
        RouteGraph g = triangle(1.2);
        g.setClock(clock);
        return g;
    }

    private static WorldObstructionReport semi(String id, double x, double y, Instant observedAt) {
        return new WorldObstructionReport(id, x, y, 0.8, null, ObstructionKind.SEMI_PERMANENT, observedAt);
    }

    private static void at(long secondsAfterT0) {
        clock.set(T0.plusSeconds(secondsAfterT0));
    }

    private static double cap(GraphEdge edge) {
        return edge.getCapacityFactor();
    }

    private static double cost(GraphEdge edge) {
        return edge.effectiveCost();
    }

    private static double penalty(GraphEdge edge) {
        return edge.getStalePenaltyMeters();
    }

    private static List<ExpiryPolicy.State> states(GraphEdge edge) {
        List<ExpiryPolicy.State> result = new ArrayList<>();
        for (ObstructionView v : edge.getObstructionViews()) {
            result.add(v.state);
        }
        return result;
    }

    private static ObstructionView view(GraphEdge edge, String eventId) {
        for (ObstructionView v : edge.getObstructionViews()) {
            if (v.eventId.equals(eventId)) {
                return v;
            }
        }
        return null;
    }

    private static boolean historyContains(RouteGraph g, String eventId, ObstructionHistoryEntry.EndReason reason) {
        for (ObstructionHistoryEntry e : g.getObstructionHistory()) {
            if (e.eventId.equals(eventId) && e.reason == reason) {
                return true;
            }
        }
        return false;
    }

    /** Checks the forward view, cap, penalty and cost, and that the backward edge agrees. */
    private static void probe(String label, GraphEdge fwd, GraphEdge bwd, String eventId,
            ExpiryPolicy.State state, double p, double expectedCap, double expectedPenalty,
            double expectedCost) {
        ObstructionView v = view(fwd, eventId);
        check(label + ": view state " + state + " (got " + (v == null ? "no view" : v.state) + ")",
                v != null && v.state == state);
        checkNear(label + ": existence probability", v == null ? Double.NaN : v.existenceProbability, p);
        checkNear(label + ": cap", cap(fwd), expectedCap);
        checkNear(label + ": penalty", penalty(fwd), expectedPenalty);
        checkNear(label + ": cost", cost(fwd), expectedCost);
        checkNear(label + ": backward Y->X cap identical", cap(bwd), cap(fwd));
        checkNear(label + ": backward Y->X cost identical", cost(bwd), cost(fwd));
    }

    private static void checkState(String description, GraphEdge edge, String eventId,
            ExpiryPolicy.State expected) {
        ObstructionView v = view(edge, eventId);
        check(description + " (got " + (v == null ? "no view" : v.state) + ")",
                v != null && v.state == expected);
    }

    private static void checkPath(String description, RouteGraph g, String... expectedIds) {
        String got = pathIds(new AStarRouter().findPath(g, "X", "Y"));
        String expected = String.join(",", expectedIds);
        check(description + " (got [" + got + "])", got.equals(expected));
    }

    private static void checkNear(String description, double actual, double expected) {
        boolean ok = (Double.isInfinite(expected) || Double.isInfinite(actual))
                ? actual == expected
                : Math.abs(actual - expected) < EPS;
        check(description + " (got " + actual + ")", ok);
    }

    private static void checkCount(String description, int actual, int expected) {
        check(description + " == " + expected + " (got " + actual + ")", actual == expected);
    }

    private static String pathIds(List<GraphNode> path) {
        List<String> ids = new ArrayList<>();
        for (GraphNode n : path) {
            ids.add(n.getId());
        }
        return String.join(",", ids);
    }

    private static String describe(GraphNode node) {
        return node == null ? "null" : node.getId();
    }

    private static String newSegment(int before) {
        byte[] all = fullLog.toByteArray();
        return new String(all, before, all.length - before);
    }

    private static void check(String description, boolean condition) {
        if (condition) {
            passCount++;
            System.out.println("  PASS: " + description);
        } else {
            failCount++;
            System.out.println("  FAIL: " + description);
        }
    }

    private static void header(String title) {
        System.out.println();
        System.out.println("=== " + title + " ===");
    }

    private static class TeeOutputStream extends OutputStream {
        private final OutputStream a;
        private final OutputStream b;

        TeeOutputStream(OutputStream a, OutputStream b) {
            this.a = a;
            this.b = b;
        }

        @Override
        public void write(int b1) throws IOException {
            a.write(b1);
            b.write(b1);
        }

        @Override
        public void flush() throws IOException {
            a.flush();
            b.flush();
        }
    }
}
