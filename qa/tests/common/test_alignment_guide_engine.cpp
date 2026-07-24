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


BOOST_AUTO_TEST_SUITE_END()
