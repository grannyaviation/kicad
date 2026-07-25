/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
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

#include <tools/ee_grid_helper.h>
#include <sch_text.h>
#include <sch_line.h>
#include <sch_shape.h>
#include <sch_junction.h>
#include <sch_sheet.h>
#include <layer_ids.h>

BOOST_AUTO_TEST_SUITE( EEGridHelperTest )

BOOST_AUTO_TEST_CASE( ItemGridClassification )
{
    EE_GRID_HELPER helper;

    SCH_TEXT text;
    BOOST_CHECK_EQUAL( helper.GetItemGrid( &text ), GRID_TEXT );

    SCH_LINE wire( VECTOR2I( 0, 0 ), LAYER_WIRE );
    BOOST_CHECK_EQUAL( helper.GetItemGrid( &wire ), GRID_WIRES );

    SCH_LINE graphic( VECTOR2I( 0, 0 ), LAYER_NOTES );
    BOOST_CHECK_EQUAL( helper.GetItemGrid( &graphic ), GRID_GRAPHICS );

    SCH_JUNCTION junc;
    BOOST_CHECK_EQUAL( helper.GetItemGrid( &junc ), GRID_WIRES );
}

// A hierarchical sheet is what the user calls a "chip": the big labelled box.  It must be an
// alignment target, and it must be measured by the rectangle that is actually drawn -- no border
// stroke halo, no sheet-name/file-name text -- or the whole-grid-step offsets the guide engine
// insists on can never be hit.
BOOST_AUTO_TEST_CASE( AlignmentBoxForSheet )
{
    SCH_SHEET sheet;
    sheet.SetPosition( VECTOR2I( 1000, 2000 ) );
    sheet.SetSize( VECTOR2I( 5000, 7000 ) );

    const std::optional<BOX2I> box = EE_GRID_HELPER::GetAlignmentBox( &sheet );

    BOOST_REQUIRE( box.has_value() );
    BOOST_CHECK_EQUAL( box->GetOrigin(), VECTOR2I( 1000, 2000 ) );
    BOOST_CHECK_EQUAL( box->GetEnd(), VECTOR2I( 6000, 9000 ) );

    // Not the stroked/labelled boxes -- those would break grid-legal snapping.
    BOOST_CHECK( *box != sheet.GetBodyBoundingBox() );
    BOOST_CHECK( *box != sheet.GetBoundingBox() );
}


BOOST_AUTO_TEST_CASE( AlignmentBoxRejectsNonBodies )
{
    SCH_JUNCTION junc;
    BOOST_CHECK( !EE_GRID_HELPER::GetAlignmentBox( &junc ).has_value() );

    SCH_LINE wire( VECTOR2I( 0, 0 ), LAYER_WIRE );
    BOOST_CHECK( !EE_GRID_HELPER::GetAlignmentBox( &wire ).has_value() );
}

BOOST_AUTO_TEST_SUITE_END()
