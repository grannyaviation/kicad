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


BOOST_AUTO_TEST_SUITE_END()
