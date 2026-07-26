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

#include <tool/align_geom.h>
#include <geometry/seg.h>

BOOST_AUTO_TEST_SUITE( AlignGeom )

// The target is whichever box the cursor is inside; that is what lets a user pick which item the
// others line up against instead of always getting the first one.
BOOST_AUTO_TEST_CASE( TargetIsTheBoxUnderTheCursor )
{
    const std::vector<BOX2I> boxes = { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 10, 10 ) ),
                                       BOX2I( VECTOR2I( 100, 0 ), VECTOR2I( 10, 10 ) ) };

    // Dereferenced, not compared as optionals: BOOST_CHECK_EQUAL needs operator<< on its
    // operands and std::optional has none.
    BOOST_REQUIRE( ALIGN_GEOM::SelectTargetIndex( boxes, VECTOR2I( 105, 5 ) ).has_value() );
    BOOST_CHECK_EQUAL( *ALIGN_GEOM::SelectTargetIndex( boxes, VECTOR2I( 105, 5 ) ), 1u );
    BOOST_CHECK_EQUAL( *ALIGN_GEOM::SelectTargetIndex( boxes, VECTOR2I( 5, 5 ) ), 0u );
}


// Cursor outside every box falls back to the first, which is the caller's sort order -- the tool
// sorts by the edge being aligned, so "first" means "the extreme one".
BOOST_AUTO_TEST_CASE( TargetFallsBackToFirstWhenCursorIsOutside )
{
    const std::vector<BOX2I> boxes = { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 10, 10 ) ),
                                       BOX2I( VECTOR2I( 100, 0 ), VECTOR2I( 10, 10 ) ) };

    BOOST_REQUIRE( ALIGN_GEOM::SelectTargetIndex( boxes, VECTOR2I( 500, 500 ) ).has_value() );
    BOOST_CHECK_EQUAL( *ALIGN_GEOM::SelectTargetIndex( boxes, VECTOR2I( 500, 500 ) ), 0u );
}


BOOST_AUTO_TEST_CASE( TargetOnEmptyInputIsAbsent )
{
    BOOST_CHECK( !ALIGN_GEOM::SelectTargetIndex( {}, VECTOR2I( 0, 0 ) ).has_value() );
}


// One delta per box, on one axis only.  A box already on the target ordinate gets zero, so
// passing the target itself in the list is harmless.
BOOST_AUTO_TEST_CASE( CenterXDeltas )
{
    const BOX2I target( VECTOR2I( 100, 0 ), VECTOR2I( 20, 10 ) ); // centre 110

    const std::vector<BOX2I> boxes = { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 10, 10 ) ), // centre 5
                                       target };

    const std::vector<VECTOR2I> deltas =
            ALIGN_GEOM::Deltas( boxes, ALIGN_GEOM::MODE::CENTER_X, target );

    BOOST_REQUIRE_EQUAL( deltas.size(), 2u );
    BOOST_CHECK_EQUAL( deltas[0], VECTOR2I( 105, 0 ) ); // 110 - 5
    BOOST_CHECK_EQUAL( deltas[1], VECTOR2I( 0, 0 ) );   // the target does not move
}


BOOST_AUTO_TEST_CASE( TopDeltas )
{
    const BOX2I target( VECTOR2I( 0, 0 ), VECTOR2I( 10, 10 ) );

    const std::vector<BOX2I> boxes = { BOX2I( VECTOR2I( 0, 40 ), VECTOR2I( 10, 10 ) ), target };

    const std::vector<VECTOR2I> deltas = ALIGN_GEOM::Deltas( boxes, ALIGN_GEOM::MODE::TOP, target );

    BOOST_REQUIRE_EQUAL( deltas.size(), 2u );
    BOOST_CHECK_EQUAL( deltas[0], VECTOR2I( 0, -40 ) );
    BOOST_CHECK_EQUAL( deltas[1], VECTOR2I( 0, 0 ) );
}


// A target outside the moved list is the locked-item case: everything moves onto it and it is
// not itself moved, because it is not in the list at all.
BOOST_AUTO_TEST_CASE( TargetNeedNotBeInTheMovedList )
{
    const BOX2I locked( VECTOR2I( 500, 0 ), VECTOR2I( 10, 10 ) ); // left edge 500

    const std::vector<BOX2I> boxes = { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( 10, 10 ) ),
                                       BOX2I( VECTOR2I( 20, 0 ), VECTOR2I( 10, 10 ) ) };

    const std::vector<VECTOR2I> deltas =
            ALIGN_GEOM::Deltas( boxes, ALIGN_GEOM::MODE::LEFT, locked );

    BOOST_REQUIRE_EQUAL( deltas.size(), 2u );
    BOOST_CHECK_EQUAL( deltas[0], VECTOR2I( 500, 0 ) );
    BOOST_CHECK_EQUAL( deltas[1], VECTOR2I( 480, 0 ) );
}


// Every mode moves on exactly one axis.  A mode leaking movement onto the other axis would drag
// items sideways while the user asked for a vertical align.
BOOST_AUTO_TEST_CASE( EveryModeMovesOneAxisOnly )
{
    const std::vector<BOX2I> boxes = { BOX2I( VECTOR2I( 3, 7 ), VECTOR2I( 11, 13 ) ),
                                       BOX2I( VECTOR2I( 50, 60 ), VECTOR2I( 17, 19 ) ) };

    for( ALIGN_GEOM::MODE mode : { ALIGN_GEOM::MODE::TOP, ALIGN_GEOM::MODE::BOTTOM,
                                   ALIGN_GEOM::MODE::LEFT, ALIGN_GEOM::MODE::RIGHT,
                                   ALIGN_GEOM::MODE::CENTER_X, ALIGN_GEOM::MODE::CENTER_Y } )
    {
        for( const VECTOR2I& d : ALIGN_GEOM::Deltas( boxes, mode, boxes.front() ) )
            BOOST_CHECK( d.x == 0 || d.y == 0 );
    }
}

// A title block is a rectangle plus a handful of dividers; the "box in the corner" a user wants
// to centre a logo in is never an object, only the region those lines happen to enclose.
BOOST_AUTO_TEST_CASE( CellAtFindsTheEnclosingRegion )
{
    // Verticals at x = 0, 10, 20, 30; horizontals at y = 0, 10, 20.  Six cells.
    std::vector<SEG> segs;

    for( int x : { 0, 10, 20, 30 } )
        segs.emplace_back( VECTOR2I( x, 0 ), VECTOR2I( x, 20 ) );

    for( int y : { 0, 10, 20 } )
        segs.emplace_back( VECTOR2I( 0, y ), VECTOR2I( 30, y ) );

    const std::optional<BOX2I> cell = ALIGN_GEOM::CellAt( segs, VECTOR2I( 15, 5 ) );

    BOOST_REQUIRE( cell.has_value() );
    BOOST_CHECK_EQUAL( cell->GetOrigin(), VECTOR2I( 10, 0 ) );
    BOOST_CHECK_EQUAL( cell->GetEnd(), VECTOR2I( 20, 10 ) );
}


// Unbounded on any side means the point is not inside a closed cell.  Returning a box built from
// three sides would invent a fourth edge and drag the item towards a boundary that is not there.
BOOST_AUTO_TEST_CASE( CellAtRejectsUnboundedPoints )
{
    const std::vector<SEG> box = { SEG( VECTOR2I( 0, 0 ), VECTOR2I( 10, 0 ) ),
                                   SEG( VECTOR2I( 10, 0 ), VECTOR2I( 10, 10 ) ),
                                   SEG( VECTOR2I( 10, 10 ), VECTOR2I( 0, 10 ) ),
                                   SEG( VECTOR2I( 0, 10 ), VECTOR2I( 0, 0 ) ) };

    // Inside: fine.
    BOOST_CHECK( ALIGN_GEOM::CellAt( box, VECTOR2I( 5, 5 ) ).has_value() );

    // Outside on the right: nothing bounds it to the right.
    BOOST_CHECK( !ALIGN_GEOM::CellAt( box, VECTOR2I( 15, 5 ) ).has_value() );

    // Three walls only.
    const std::vector<SEG> open = { SEG( VECTOR2I( 0, 0 ), VECTOR2I( 10, 0 ) ),
                                    SEG( VECTOR2I( 10, 0 ), VECTOR2I( 10, 10 ) ),
                                    SEG( VECTOR2I( 0, 10 ), VECTOR2I( 0, 0 ) ) };

    BOOST_CHECK( !ALIGN_GEOM::CellAt( open, VECTOR2I( 5, 5 ) ).has_value() );

    BOOST_CHECK( !ALIGN_GEOM::CellAt( {}, VECTOR2I( 5, 5 ) ).has_value() );
}


// A segment only bounds a point if it actually spans it on the other axis.  Title-block dividers
// are short -- the vertical between two fields runs a few mm, not the height of the block -- so
// ignoring the span would report a cell whose walls are nowhere near the point.
BOOST_AUTO_TEST_CASE( CellAtIgnoresSegmentsThatDoNotSpanThePoint )
{
    std::vector<SEG> segs = { SEG( VECTOR2I( 0, 0 ), VECTOR2I( 0, 100 ) ),
                              SEG( VECTOR2I( 100, 0 ), VECTOR2I( 100, 100 ) ),
                              SEG( VECTOR2I( 0, 0 ), VECTOR2I( 100, 0 ) ),
                              SEG( VECTOR2I( 0, 100 ), VECTOR2I( 100, 100 ) ) };

    // A stub vertical near the top must not become the right wall of a point near the bottom.
    segs.emplace_back( VECTOR2I( 50, 0 ), VECTOR2I( 50, 10 ) );

    const std::optional<BOX2I> cell = ALIGN_GEOM::CellAt( segs, VECTOR2I( 20, 90 ) );

    BOOST_REQUIRE( cell.has_value() );
    BOOST_CHECK_EQUAL( cell->GetEnd().x, 100 );
}


// A point resting exactly on a divider belongs to the cell on one side, not to a zero-width one.
// The comparison is strict for this reason; a logo dragged along a rule would otherwise flicker
// between a real cell and a degenerate one.
BOOST_AUTO_TEST_CASE( CellAtTreatsAPointOnADividerAsOutsideIt )
{
    std::vector<SEG> segs;

    for( int x : { 0, 10, 20 } )
        segs.emplace_back( VECTOR2I( x, 0 ), VECTOR2I( x, 10 ) );

    segs.emplace_back( VECTOR2I( 0, 0 ), VECTOR2I( 20, 0 ) );
    segs.emplace_back( VECTOR2I( 0, 10 ), VECTOR2I( 20, 10 ) );

    const std::optional<BOX2I> cell = ALIGN_GEOM::CellAt( segs, VECTOR2I( 10, 5 ) );

    BOOST_REQUIRE( cell.has_value() );
    BOOST_CHECK_EQUAL( cell->GetOrigin().x, 0 );
    BOOST_CHECK_EQUAL( cell->GetEnd().x, 20 );
}


// Drawing sheets may carry diagonals and polygons.  They bound nothing rectilinear, and treating
// an endpoint as a wall would put a cell edge at an arbitrary place.
BOOST_AUTO_TEST_CASE( CellAtIgnoresNonAxisAlignedSegments )
{
    const std::vector<SEG> segs = { SEG( VECTOR2I( 0, 0 ), VECTOR2I( 0, 10 ) ),
                                    SEG( VECTOR2I( 10, 0 ), VECTOR2I( 10, 10 ) ),
                                    SEG( VECTOR2I( 0, 0 ), VECTOR2I( 10, 0 ) ),
                                    SEG( VECTOR2I( 0, 10 ), VECTOR2I( 10, 10 ) ),
                                    SEG( VECTOR2I( 2, 2 ), VECTOR2I( 8, 8 ) ),   // diagonal
                                    SEG( VECTOR2I( 4, 4 ), VECTOR2I( 4, 4 ) ) }; // degenerate

    const std::optional<BOX2I> cell = ALIGN_GEOM::CellAt( segs, VECTOR2I( 5, 5 ) );

    BOOST_REQUIRE( cell.has_value() );
    BOOST_CHECK_EQUAL( cell->GetOrigin(), VECTOR2I( 0, 0 ) );
    BOOST_CHECK_EQUAL( cell->GetEnd(), VECTOR2I( 10, 10 ) );
}

// A T-junction: a partial divider whose endpoint lands on a full-width one.  The partial divider
// separates nothing at the junction coordinate, so it must not become a wall there -- KiCad's own
// default title block has this shape, a short column divider meeting a full-width row rule.
BOOST_AUTO_TEST_CASE( CellAtIgnoresADividerThatOnlyTouchesThePoint )
{
    std::vector<SEG> segs = { SEG( VECTOR2I( 0, 0 ), VECTOR2I( 100, 0 ) ),
                              SEG( VECTOR2I( 100, 0 ), VECTOR2I( 100, 100 ) ),
                              SEG( VECTOR2I( 100, 100 ), VECTOR2I( 0, 100 ) ),
                              SEG( VECTOR2I( 0, 100 ), VECTOR2I( 0, 0 ) ),
                              SEG( VECTOR2I( 0, 50 ), VECTOR2I( 100, 50 ) ) };

    // Spans only the upper half; its lower endpoint touches the full-width rule at y = 50.
    segs.emplace_back( VECTOR2I( 50, 0 ), VECTOR2I( 50, 50 ) );

    // On the full-width rule, which is excluded by the strict rule, so the cell is the union of
    // the two rows.  The partial vertical must not narrow it: below y = 50 it does not exist.
    const std::optional<BOX2I> cell = ALIGN_GEOM::CellAt( segs, VECTOR2I( 75, 50 ) );

    BOOST_REQUIRE( cell.has_value() );
    BOOST_CHECK_EQUAL( cell->GetOrigin(), VECTOR2I( 0, 0 ) );
    BOOST_CHECK_EQUAL( cell->GetEnd(), VECTOR2I( 100, 100 ) );
}

BOOST_AUTO_TEST_SUITE_END()
