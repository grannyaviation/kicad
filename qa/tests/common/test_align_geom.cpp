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

BOOST_AUTO_TEST_SUITE_END()
