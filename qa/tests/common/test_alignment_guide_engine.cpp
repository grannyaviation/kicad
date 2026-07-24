/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#define BOOST_TEST_NO_MAIN
#include <boost/test/unit_test.hpp>

#include <tool/alignment_guide_engine.h>

BOOST_AUTO_TEST_SUITE( AlignmentGuideEngine )


BOOST_AUTO_TEST_CASE( NoNeighborsNoSnap )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    BOX2I moving( VECTOR2I( 0, 0 ), VECTOR2I( 40, 20 ) );

    BOOST_CHECK( !engine.FindSnap( moving, 10 ).has_value() );
}


BOOST_AUTO_TEST_CASE( EdgeAlignLeftX )
{
    ALIGNMENT_GUIDE_ENGINE engine;

    // Neighbor occupying x:[0,100], y:[0,50]
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 100, 50 ) ) } );

    // Moving box near x=3: left edges should align at x=0.  Y is far away on
    // purpose so no Y candidate is in range.
    BOX2I moving( VECTOR2I( 3, 500 ), VECTOR2I( 40, 20 ) );

    auto result = engine.FindSnap( moving, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, -3 );
    BOOST_CHECK_EQUAL( result->Offset.y, 0 );

    // Only the X axis snapped, so exactly one guide line.  It sits on the aligned
    // ordinate x=0 and spans the cross axis merged from the snapped moving box
    // (y:[500,520]) and the neighbor (y:[0,50]) -> y:[0,520].
    BOOST_REQUIRE_EQUAL( result->Lines.size(), 1 );
    BOOST_CHECK_EQUAL( result->Lines[0].A.x, 0 );
    BOOST_CHECK_EQUAL( result->Lines[0].A.y, 0 );
    BOOST_CHECK_EQUAL( result->Lines[0].B.x, 0 );
    BOOST_CHECK_EQUAL( result->Lines[0].B.y, 520 );
}


BOOST_AUTO_TEST_CASE( CenterAlignNegativeX )
{
    ALIGNMENT_GUIDE_ENGINE engine;

    // Neighbor occupying x:[-100,-61], y:[0,40].  The span is deliberately odd-sized
    // and negative: SPAN::Center() computes Min + (Max-Min)/2 = -81, whereas a naive
    // (Min+Max)/2 would truncate towards zero and give -80.
    engine.SetNeighbors( { BOX2I( VECTOR2I( -100, 0 ), VECTOR2I( 39, 40 ) ) } );

    // Moving box x:[-82,-72] (centre -77), narrower than the neighbor so every edge
    // candidate is out of range and only the centre-to-centre one (-4) survives.
    // Y is far away on purpose so no Y candidate is in range.
    BOX2I moving( VECTOR2I( -82, 500 ), VECTOR2I( 10, 20 ) );

    auto result = engine.FindSnap( moving, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, -4 );
    BOOST_CHECK_EQUAL( result->Offset.y, 0 );

    // Snapped box is x:[-86,-76], centre -81 == the neighbor centre, so the guide
    // ordinate is -81 and the cross span merges y:[500,520] with y:[0,40].
    BOOST_REQUIRE_EQUAL( result->Lines.size(), 1 );
    BOOST_CHECK_EQUAL( result->Lines[0].A.x, -81 );
    BOOST_CHECK_EQUAL( result->Lines[0].A.y, 0 );
    BOOST_CHECK_EQUAL( result->Lines[0].B.x, -81 );
    BOOST_CHECK_EQUAL( result->Lines[0].B.y, 520 );
}


BOOST_AUTO_TEST_CASE( CenterAlignY )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 100, 50 ) ) } ); // centerY = 25

    // Moving box 20 tall, top at y=17 -> centerY = 27, should center-align to 25
    BOX2I moving( VECTOR2I( 500, 17 ), VECTOR2I( 40, 20 ) );

    auto result = engine.FindSnap( moving, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.y, -2 );
    BOOST_CHECK_EQUAL( result->Offset.x, 0 );
}


BOOST_AUTO_TEST_CASE( BothAxesIndependent )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 100, 50 ) ) } );

    // Left edge near x=0 (delta -4), top edge near y=0 (delta +3)
    BOX2I moving( VECTOR2I( 4, -3 ), VECTOR2I( 40, 20 ) );

    auto result = engine.FindSnap( moving, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, -4 );
    BOOST_CHECK_EQUAL( result->Offset.y, 3 );
    BOOST_CHECK_EQUAL( result->Lines.size(), 2 );
}


BOOST_AUTO_TEST_CASE( NearestCandidateWins )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    // Two neighbors: right edge of A at 100, left edge of B at 103
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 100, 50 ) ),
                           BOX2I( VECTOR2I( 103, 0 ), VECTOR2I( 50, 50 ) ) } );

    // Moving left edge at 102: B.left (delta +1) beats A.right (delta -2)
    BOX2I moving( VECTOR2I( 102, 500 ), VECTOR2I( 40, 20 ) );

    auto result = engine.FindSnap( moving, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, 1 );
}


BOOST_AUTO_TEST_CASE( OutOfRangeNoSnap )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 100, 50 ) ) } );

    BOX2I moving( VECTOR2I( 500, 500 ), VECTOR2I( 40, 20 ) );

    BOOST_CHECK( !engine.FindSnap( moving, 10 ).has_value() );
}


BOOST_AUTO_TEST_CASE( EqualSpacingExtendsChain )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    // A x:[0,20], B x:[50,70] -> gap 30.  All share y:[0,20].
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 20, 20 ) ),
                           BOX2I( VECTOR2I( 50, 0 ), VECTOR2I( 20, 20 ) ) } );

    // Moving box (20 wide) near x=104; equal spacing puts left edge at 70+30=100
    BOX2I moving( VECTOR2I( 104, 0 ), VECTOR2I( 20, 20 ) );

    auto result = engine.FindSnap( moving, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, -4 );

    // Two gaps -> two badges, both reporting 30
    BOOST_REQUIRE_EQUAL( result->Badges.size(), 2 );
    BOOST_CHECK_EQUAL( result->Badges[0].Gap, 30 );
    BOOST_CHECK_EQUAL( result->Badges[1].Gap, 30 );
    BOOST_CHECK( !result->Badges[0].Vertical );
}


BOOST_AUTO_TEST_CASE( EqualSpacingExtendsChainBackwards )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    // Same A x:[0,20] / B x:[50,70] pair, gap 30, all at y:[0,20].
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 20, 20 ) ),
                           BOX2I( VECTOR2I( 50, 0 ), VECTOR2I( 20, 20 ) ) } );

    // Moving box (20 wide) at x:[-54,-34]; equal spacing puts its right edge at
    // 0-30=-30, i.e. x:[-50,-30], so the offset is +4.  Nearest alignment target
    // is A.left=0 (delta +34), well out of range.
    BOX2I moving( VECTOR2I( -54, 0 ), VECTOR2I( 20, 20 ) );

    auto result = engine.FindSnap( moving, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, 4 );

    // Badges walk left-to-right: moving.right(-30)->A.left(0), then A.right(20)->
    // B.left(50).  Both 30 wide, both centred on the shared y overlap [0,20].
    BOOST_REQUIRE_EQUAL( result->Badges.size(), 2 );
    BOOST_CHECK_EQUAL( result->Badges[0].Gap, 30 );
    BOOST_CHECK_EQUAL( result->Badges[1].Gap, 30 );
    BOOST_CHECK( !result->Badges[0].Vertical );
    BOOST_CHECK_EQUAL( result->Badges[0].Pos.x, -15 );
    BOOST_CHECK_EQUAL( result->Badges[0].Pos.y, 10 );
    BOOST_CHECK_EQUAL( result->Badges[1].Pos.x, 35 );
    BOOST_CHECK_EQUAL( result->Badges[1].Pos.y, 10 );
}


BOOST_AUTO_TEST_CASE( EqualSpacingRequiresCrossOverlap )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    // Same as above but neighbors live at y:[0,20] while moving is at y:[400,420]:
    // no cross-axis overlap -> no equal-spacing candidate (alignment may still
    // fire on Y=aligned edges, so keep X far from alignment targets too).
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 20, 20 ) ),
                           BOX2I( VECTOR2I( 50, 0 ), VECTOR2I( 20, 20 ) ) } );

    BOX2I moving( VECTOR2I( 104, 400 ), VECTOR2I( 20, 20 ) );

    auto result = engine.FindSnap( moving, 10 );

    // x=104: nearest alignment target is B.right=70 (delta -34, out of range);
    // equal-spacing target x=100 must NOT fire because of the y separation.
    BOOST_CHECK( !result.has_value() );
}


BOOST_AUTO_TEST_CASE( CenterBetweenTwoNeighbors )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    // A x:[0,20], B x:[100,120]; room between edges = 80, moving is 20 wide
    // -> centered position has 30 on each side: moving x:[50,70]
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 20, 20 ) ),
                           BOX2I( VECTOR2I( 100, 0 ), VECTOR2I( 20, 20 ) ) } );

    BOX2I moving( VECTOR2I( 53, 0 ), VECTOR2I( 20, 20 ) );

    auto result = engine.FindSnap( moving, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, -3 );

    BOOST_REQUIRE_EQUAL( result->Badges.size(), 2 );
    BOOST_CHECK_EQUAL( result->Badges[0].Gap, 30 );
    BOOST_CHECK_EQUAL( result->Badges[1].Gap, 30 );
}


BOOST_AUTO_TEST_CASE( CenterBetweenOddLeftoverIsAsymmetric )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    // A x:[0,20], B x:[101,121] -> room 81 for a 20-wide box.  81-20=61 is odd, so
    // the integer halving cannot split it evenly: left gap 30, right gap 31.
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 20, 20 ) ),
                           BOX2I( VECTOR2I( 101, 0 ), VECTOR2I( 20, 20 ) ) } );

    BOX2I moving( VECTOR2I( 53, 0 ), VECTOR2I( 20, 20 ) );

    auto result = engine.FindSnap( moving, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, -3 ); // 20 + 61/2 = 50, so x:[50,70]

    BOOST_REQUIRE_EQUAL( result->Badges.size(), 2 );
    BOOST_CHECK_EQUAL( result->Badges[0].Gap, 30 );
    BOOST_CHECK_EQUAL( result->Badges[1].Gap, 31 );
    BOOST_CHECK_EQUAL( result->Badges[0].Pos.x, 35 );
    BOOST_CHECK_EQUAL( result->Badges[1].Pos.x, 85 );
}


BOOST_AUTO_TEST_CASE( CenterInContainer )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    // Container (e.g. board outline) x:[0,200], y:[0,100] -> center (100,50)
    engine.SetContainers( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 200, 100 ) ) } );

    // Moving box 20x10, near-centered: center at (104,52)
    BOX2I moving( VECTOR2I( 94, 47 ), VECTOR2I( 20, 10 ) );

    auto result = engine.FindSnap( moving, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, -4 );
    BOOST_CHECK_EQUAL( result->Offset.y, -2 );
    BOOST_REQUIRE_EQUAL( result->CenterMarks.size(), 1 );
    BOOST_CHECK_EQUAL( result->CenterMarks[0].x, 100 );
    BOOST_CHECK_EQUAL( result->CenterMarks[0].y, 50 );
}


BOOST_AUTO_TEST_CASE( CenterInContainerSingleAxis )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    // Same container as above: x:[0,200], y:[0,100] -> center (100,50)
    engine.SetContainers( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 200, 100 ) ) } );

    // Moving box 20x10 at x:[94,114] (centre 104) but y:[500,510] (centre 505).
    // X centres with delta -4; the Y candidate is 50-505 = -455, far out of range.
    BOX2I moving( VECTOR2I( 94, 500 ), VECTOR2I( 20, 10 ) );

    auto result = engine.FindSnap( moving, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, -4 );
    BOOST_CHECK_EQUAL( result->Offset.y, 0 );

    // One mark per snap, not per axis: only X won, so still exactly one.
    BOOST_REQUIRE_EQUAL( result->CenterMarks.size(), 1 );
    BOOST_CHECK_EQUAL( result->CenterMarks[0].x, 100 );
    BOOST_CHECK_EQUAL( result->CenterMarks[0].y, 50 );
    BOOST_CHECK( result->Lines.empty() );
}


BOOST_AUTO_TEST_CASE( ContainerLosesToNearerAlignment )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    // Container x:[0,200], y:[0,100] -> center (100,50)
    engine.SetContainers( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 200, 100 ) ) } );

    // A single neighbor at x:[96,116], y:[400,440].  Its Y candidates are all
    // 340+ away, so it can only compete on X.
    engine.SetNeighbors( { BOX2I( VECTOR2I( 96, 400 ), VECTOR2I( 20, 40 ) ) } );

    // Moving box 20x10 at x:[94,114] (centre 104), y:[47,57] (centre 52).
    BOX2I moving( VECTOR2I( 94, 47 ), VECTOR2I( 20, 10 ) );

    auto result = engine.FindSnap( moving, 10 );

    BOOST_REQUIRE( result.has_value() );

    // X: the neighbor's left edge (96-94 = +2) beats the container centre
    // (100-104 = -4).  Three neighbor candidates tie at +2 (min-min, max-max and
    // centre-centre); the first pushed wins, so the guide ordinate is min = 96.
    BOOST_CHECK_EQUAL( result->Offset.x, 2 );

    // Y: no neighbor candidate is in range, so the container centre wins (-2).
    BOOST_CHECK_EQUAL( result->Offset.y, -2 );

    // Snapped box is x:[96,116], y:[45,55].  The X guide spans the cross axis
    // merged from the snapped box (y:[45,55]) and the neighbor (y:[400,440]).
    BOOST_REQUIRE_EQUAL( result->Lines.size(), 1 );
    BOOST_CHECK_EQUAL( result->Lines[0].A.x, 96 );
    BOOST_CHECK_EQUAL( result->Lines[0].A.y, 45 );
    BOOST_CHECK_EQUAL( result->Lines[0].B.x, 96 );
    BOOST_CHECK_EQUAL( result->Lines[0].B.y, 440 );

    // Only the Y axis snapped to the container, so exactly one centre mark.
    BOOST_REQUIRE_EQUAL( result->CenterMarks.size(), 1 );
    BOOST_CHECK_EQUAL( result->CenterMarks[0].x, 100 );
    BOOST_CHECK_EQUAL( result->CenterMarks[0].y, 50 );
    BOOST_CHECK( result->Badges.empty() );
}


BOOST_AUTO_TEST_CASE( NestedNeighborsPairByCluster )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    // B is nested inside A along X.  Pairing sorted neighbors consecutively would
    // pair B with C and report a 180-wide gap (20 -> 200) that runs straight through
    // A's body.  The real clear space is A/B's merged right edge 100 -> C's left
    // edge 200, i.e. 100.  All three share y:[0,20].
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 100, 20 ) ),
                           BOX2I( VECTOR2I( 10, 0 ), VECTOR2I( 10, 20 ) ),
                           BOX2I( VECTOR2I( 200, 0 ), VECTOR2I( 10, 20 ) ) } );

    // The phantom B->C gap of 180 would offer 210 + 180 = 390.  It must not exist.
    // Y still snaps (delta 0), so the result is present but must not move X.
    BOX2I phantom( VECTOR2I( 394, 0 ), VECTOR2I( 20, 20 ) );

    auto phantomResult = engine.FindSnap( phantom, 10 );

    BOOST_REQUIRE( phantomResult.has_value() );
    BOOST_CHECK_EQUAL( phantomResult->Offset.x, 0 );
    BOOST_CHECK( phantomResult->Badges.empty() );

    // Extending the genuine gap puts the moving box at 210 + 100 = 310.  Nothing else
    // is within 10: the nearest alignment target is C.right = 210 (delta -104).
    BOX2I moving( VECTOR2I( 314, 0 ), VECTOR2I( 20, 20 ) );

    auto result = engine.FindSnap( moving, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, -4 );
    BOOST_CHECK_EQUAL( result->Offset.y, 0 ); // y:[0,20] already aligns with A/B/C

    // Badges measure the cluster gap 100 -> 200 and the new gap 210 -> 310, both 100.
    // Cross overlap of the near cluster (y:[0,20]) with the snapped box (y:[0,20])
    // is [0,20], so both badges sit at y = 10.
    BOOST_REQUIRE_EQUAL( result->Badges.size(), 2 );
    BOOST_CHECK_EQUAL( result->Badges[0].Gap, 100 );
    BOOST_CHECK_EQUAL( result->Badges[1].Gap, 100 );
    BOOST_CHECK_EQUAL( result->Badges[0].Pos.x, 150 );
    BOOST_CHECK_EQUAL( result->Badges[0].Pos.y, 10 );
    BOOST_CHECK_EQUAL( result->Badges[1].Pos.x, 260 );
    BOOST_CHECK_EQUAL( result->Badges[1].Pos.y, 10 );
}


BOOST_AUTO_TEST_CASE( PartialOverlapAnchorsOnClusterEdge )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    // A x:[0,50] and B x:[40,100] partially overlap and merge into one cluster
    // x:[0,100]; C x:[200,210] stands alone.  Gap between clusters = 100.
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 50, 20 ) ),
                           BOX2I( VECTOR2I( 40, 0 ), VECTOR2I( 60, 20 ) ),
                           BOX2I( VECTOR2I( 200, 0 ), VECTOR2I( 10, 20 ) ) } );

    // Pushing the gap out on the "before" side must anchor on the cluster's left edge
    // 0, giving moving.right = 0 - 100 = -100, i.e. x:[-120,-100].  Anchoring on B
    // (which owns neither cluster edge) would give -60 instead, 40 units off.
    BOX2I moving( VECTOR2I( -116, 0 ), VECTOR2I( 20, 20 ) );

    auto result = engine.FindSnap( moving, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, -4 );
    BOOST_CHECK_EQUAL( result->Offset.y, 0 );

    // Badges walk left to right: moving.right(-100) -> cluster.left(0), then
    // cluster.right(100) -> C.left(200).  Both report 100 units of genuinely empty
    // space; a badge drawn from B's edges would claim 140 and cover A.
    BOOST_REQUIRE_EQUAL( result->Badges.size(), 2 );
    BOOST_CHECK_EQUAL( result->Badges[0].Gap, 100 );
    BOOST_CHECK_EQUAL( result->Badges[1].Gap, 100 );
    BOOST_CHECK_EQUAL( result->Badges[0].Pos.x, -50 );
    BOOST_CHECK_EQUAL( result->Badges[0].Pos.y, 10 );
    BOOST_CHECK_EQUAL( result->Badges[1].Pos.x, 150 );
    BOOST_CHECK_EQUAL( result->Badges[1].Pos.y, 10 );
}


BOOST_AUTO_TEST_CASE( EqualSpacingVerticalBadges )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    // A y:[0,20], B y:[50,70] -> gap 30 measured along Y.  Both x:[0,20].
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 20, 20 ) ),
                           BOX2I( VECTOR2I( 0, 50 ), VECTOR2I( 20, 20 ) ) } );

    // Moving box (20 tall) at y:[104,124]; equal spacing puts its top at 70+30=100.
    // X aligns exactly (delta 0) so the snapped box stays at x:[0,20].
    BOX2I moving( VECTOR2I( 0, 104 ), VECTOR2I( 20, 20 ) );

    auto result = engine.FindSnap( moving, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, 0 );
    BOOST_CHECK_EQUAL( result->Offset.y, -4 );

    // Badges measure A.bottom(20)->B.top(50) and B.bottom(70)->moving.top(100).
    // Along Y the badge position is (crossMid, mid), i.e. x carries the cross
    // ordinate 10 and y carries the gap midpoint - the transposed layout of the
    // horizontal case.
    BOOST_REQUIRE_EQUAL( result->Badges.size(), 2 );
    BOOST_CHECK( result->Badges[0].Vertical );
    BOOST_CHECK( result->Badges[1].Vertical );
    BOOST_CHECK_EQUAL( result->Badges[0].Gap, 30 );
    BOOST_CHECK_EQUAL( result->Badges[1].Gap, 30 );
    BOOST_CHECK_EQUAL( result->Badges[0].Pos.x, 10 );
    BOOST_CHECK_EQUAL( result->Badges[0].Pos.y, 35 );
    BOOST_CHECK_EQUAL( result->Badges[1].Pos.x, 10 );
    BOOST_CHECK_EQUAL( result->Badges[1].Pos.y, 85 );
}


BOOST_AUTO_TEST_CASE( CrossEdgeAlignOrdinates )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    // Neighbor x:[0,100], y:[0,50].  Y is kept far from the moving box throughout so
    // only X can snap.
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 100, 50 ) ) } );

    // moving.left near neighbor.right: 100 - 97 = +3 is the only candidate in range
    // (min-min -97, min-max -137, max-max -37, centre -67).  The guide must land on
    // the neighbor's *right* edge, x=100 - not on its left edge.
    BOX2I after( VECTOR2I( 97, 500 ), VECTOR2I( 40, 20 ) );

    auto result = engine.FindSnap( after, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, 3 );
    BOOST_CHECK_EQUAL( result->Offset.y, 0 );
    BOOST_REQUIRE_EQUAL( result->Lines.size(), 1 );
    BOOST_CHECK_EQUAL( result->Lines[0].A.x, 100 );
    BOOST_CHECK_EQUAL( result->Lines[0].A.y, 0 );
    BOOST_CHECK_EQUAL( result->Lines[0].B.x, 100 );
    BOOST_CHECK_EQUAL( result->Lines[0].B.y, 520 );

    // The mirror pairing: moving.right near neighbor.left, 0 - (-3) = +3, the only
    // candidate in range (min-min 43, max-min 143, max-max 103, centre 73).  Guide
    // lands on the neighbor's left edge, x=0.
    BOX2I before( VECTOR2I( -43, 500 ), VECTOR2I( 40, 20 ) );

    result = engine.FindSnap( before, 10 );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, 3 );
    BOOST_CHECK_EQUAL( result->Offset.y, 0 );
    BOOST_REQUIRE_EQUAL( result->Lines.size(), 1 );
    BOOST_CHECK_EQUAL( result->Lines[0].A.x, 0 );
    BOOST_CHECK_EQUAL( result->Lines[0].A.y, 0 );
    BOOST_CHECK_EQUAL( result->Lines[0].B.x, 0 );
    BOOST_CHECK_EQUAL( result->Lines[0].B.y, 520 );
}


BOOST_AUTO_TEST_CASE( GridLegalOffsetSurvives )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 100, 50 ) ) } );

    // Left edges align at x=0 with delta -10, which is exactly 1 step of a grid of 10.
    BOX2I moving( VECTOR2I( 10, 500 ), VECTOR2I( 40, 20 ) );

    auto result = engine.FindSnap( moving, 15, VECTOR2I( 10, 10 ) );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, -10 );
    BOOST_CHECK_EQUAL( result->Offset.y, 0 );
}


BOOST_AUTO_TEST_CASE( GridIllegalOffsetIsRejectedNotRounded )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 100, 50 ) ) } );

    // Left edges would align with delta -3, which is not a multiple of 10.
    // The engine must NOT round it to 0 and must NOT report a snap on X.
    BOX2I moving( VECTOR2I( 3, 500 ), VECTOR2I( 40, 20 ) );

    auto result = engine.FindSnap( moving, 15, VECTOR2I( 10, 10 ) );

    // No X candidate is grid-legal and Y is far away, so there is no snap at all.
    BOOST_CHECK( !result.has_value() );
}


BOOST_AUTO_TEST_CASE( GridLegalCandidateBeatsNearerIllegalOne )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    // A's right edge at 100, B's left edge at 104.
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 100, 50 ) ),
                           BOX2I( VECTOR2I( 104, 0 ), VECTOR2I( 50, 50 ) ) } );

    // Moving box x:[110,150].  Every in-range candidate is illegal on a grid of 10
    // except A.right (delta -10): the nearest is centre-to-centre (B centre 129 vs
    // moving centre 130, delta -1), then max-max (B.right 154 vs moving.right 150,
    // delta +4), then B.left (delta -6).  The legal, farther candidate beats all three.
    BOX2I moving( VECTOR2I( 110, 500 ), VECTOR2I( 40, 20 ) );

    auto result = engine.FindSnap( moving, 15, VECTOR2I( 10, 10 ) );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, -10 );
}


BOOST_AUTO_TEST_CASE( GridStepIsPerAxis )
{
    ALIGNMENT_GUIDE_ENGINE engine;
    engine.SetNeighbors( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 100, 50 ) ) } );

    // Moving box x:[20,60] (centre 40), y:[-25,-5] (centre -15), grid 10 x 25.
    //
    // X, step 10: min-min -20 and centre-centre (50-40) +10 are both in range and
    //   both whole multiples -> +10 wins on distance.  Neither is a multiple of 25.
    // Y, step 25: min-min (0 - -25) +25 is in range and a whole multiple of 25 but
    //   not of 10.  The other in-range Y candidate, min-max (0 - -5) +5, is a
    //   multiple of neither; max-min +75, max-max +55 and centre (25 - -15) +40 are
    //   all beyond the range of 30.
    //
    // So each axis is legal only under *its own* step: reading aGridStep->x for both
    // axes rejects the +25 and leaves Offset.y at 0, and reading ->y for both rejects
    // -20 and +10 and leaves Offset.x at 0.
    BOX2I moving( VECTOR2I( 20, -25 ), VECTOR2I( 40, 20 ) );

    auto result = engine.FindSnap( moving, 30, VECTOR2I( 10, 25 ) );

    BOOST_REQUIRE( result.has_value() );
    BOOST_CHECK_EQUAL( result->Offset.x, 10 );
    BOOST_CHECK_EQUAL( result->Offset.y, 25 );
}


BOOST_AUTO_TEST_SUITE_END()
