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
#include <sch_pin.h>
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

// A pin is a point, not a rectangle: what anyone aligns is the place a wire attaches.  A
// zero-size box collapses min, max and centre onto that point, which is what makes both
// pin-to-pin alignment and equal pin pitch fall out of the existing engine.
BOOST_AUTO_TEST_CASE( SymbolAlignmentBoxForPin )
{
    SCH_PIN pin( nullptr );
    pin.SetPosition( VECTOR2I( 2540, -1270 ) );

    const std::optional<BOX2I> box = EE_GRID_HELPER::GetSymbolAlignmentBox( &pin );

    BOOST_REQUIRE( box.has_value() );
    BOOST_CHECK_EQUAL( box->GetOrigin(), VECTOR2I( 2540, -1270 ) );
    BOOST_CHECK_EQUAL( box->GetEnd(), VECTOR2I( 2540, -1270 ) );
    BOOST_CHECK_EQUAL( box->GetWidth(), 0 );
    BOOST_CHECK_EQUAL( box->GetHeight(), 0 );
}


// EDA_SHAPE::getBoundingBox() inflates by half the stroke width.  Half a stroke is never a whole
// grid step, so a body outline measured that way can never align to anything on grid.  The rule
// has to hand back the rectangle that was actually drawn.
BOOST_AUTO_TEST_CASE( SymbolAlignmentBoxDeflatesShapeStroke )
{
    SCH_SHAPE rect( SHAPE_T::RECTANGLE );
    rect.SetStart( VECTOR2I( 0, 0 ) );
    rect.SetEnd( VECTOR2I( 5080, 2540 ) );
    rect.SetWidth( 254 );

    // Precondition: the inflated box really is bigger, or this test proves nothing.
    BOOST_REQUIRE_EQUAL( rect.GetBoundingBox().GetOrigin(), VECTOR2I( -127, -127 ) );

    const std::optional<BOX2I> box = EE_GRID_HELPER::GetSymbolAlignmentBox( &rect );

    BOOST_REQUIRE( box.has_value() );
    BOOST_CHECK_EQUAL( box->GetOrigin(), VECTOR2I( 0, 0 ) );
    BOOST_CHECK_EQUAL( box->GetEnd(), VECTOR2I( 5080, 2540 ) );
}


// A horizontal segment -- a diode bar, a ground bar, a connector divider line -- has zero
// extent on one axis by design.  That must not be treated as degenerate: only an uninitialised
// box (empty POLY, unrebuilt BEZIER) is rejected, not a shape that is legitimately thin.
BOOST_AUTO_TEST_CASE( SymbolAlignmentBoxAcceptsZeroHeightSegment )
{
    SCH_SHAPE seg( SHAPE_T::SEGMENT );
    seg.SetStart( VECTOR2I( 0, 0 ) );
    seg.SetEnd( VECTOR2I( 5080, 0 ) );
    seg.SetWidth( 254 );

    const std::optional<BOX2I> box = EE_GRID_HELPER::GetSymbolAlignmentBox( &seg );

    BOOST_REQUIRE( box.has_value() );
    BOOST_CHECK_EQUAL( box->GetOrigin(), VECTOR2I( 0, 0 ) );
    BOOST_CHECK_EQUAL( box->GetEnd(), VECTOR2I( 5080, 0 ) );
    BOOST_CHECK_EQUAL( box->GetHeight(), 0 );
}


// The deflate must recover the nominal outline regardless of how the stroke compares to the
// shape's own size -- there is no size threshold past which a stroke makes a rectangle stop
// being an alignment target.
BOOST_AUTO_TEST_CASE( SymbolAlignmentBoxAcceptsThinRectangleAtAnyStroke )
{
    SCH_SHAPE rect( SHAPE_T::RECTANGLE );
    rect.SetStart( VECTOR2I( 0, 0 ) );
    rect.SetEnd( VECTOR2I( 5080, 100 ) );
    rect.SetWidth( 254 );

    const std::optional<BOX2I> box = EE_GRID_HELPER::GetSymbolAlignmentBox( &rect );

    BOOST_REQUIRE( box.has_value() );
    BOOST_CHECK_EQUAL( box->GetOrigin(), VECTOR2I( 0, 0 ) );
    BOOST_CHECK_EQUAL( box->GetEnd(), VECTOR2I( 5080, 100 ) );
}


// An odd stroke width truncates deflate = GetWidth() / 2 down by one.  The result must still be
// exact: the same truncated value is what getBoundingBox() added, so it cancels either way.
BOOST_AUTO_TEST_CASE( SymbolAlignmentBoxOddStrokeWidthStillExact )
{
    SCH_SHAPE rect( SHAPE_T::RECTANGLE );
    rect.SetStart( VECTOR2I( 0, 0 ) );
    rect.SetEnd( VECTOR2I( 5080, 2540 ) );
    rect.SetWidth( 255 );

    const std::optional<BOX2I> box = EE_GRID_HELPER::GetSymbolAlignmentBox( &rect );

    BOOST_REQUIRE( box.has_value() );
    BOOST_CHECK_EQUAL( box->GetOrigin(), VECTOR2I( 0, 0 ) );
    BOOST_CHECK_EQUAL( box->GetEnd(), VECTOR2I( 5080, 2540 ) );
}


// A circle's bounding box is already built from a prior Inflate( GetRadius() ) before the
// stroke-width inflate is applied.  The deflate must undo only the latter.
BOOST_AUTO_TEST_CASE( SymbolAlignmentBoxCircleRoundTrips )
{
    SCH_SHAPE circle( SHAPE_T::CIRCLE );
    circle.SetStart( VECTOR2I( 0, 0 ) );
    circle.SetRadius( 1270 );
    circle.SetWidth( 255 );

    const std::optional<BOX2I> box = EE_GRID_HELPER::GetSymbolAlignmentBox( &circle );

    BOOST_REQUIRE( box.has_value() );
    BOOST_CHECK_EQUAL( box->GetOrigin(), VECTOR2I( -1270, -1270 ) );
    BOOST_CHECK_EQUAL( box->GetEnd(), VECTOR2I( 1270, 1270 ) );
}


// The one shape that must be rejected.  getBoundingBox() bails before touching bbox for an
// empty POLY, so it comes back default-constructed -- and the engine reads an invalid box as a
// real point box at the origin, which would drag the selection towards (0, 0).  Inflate() and
// Normalize() never set m_init, so deflating cannot revive it either.
BOOST_AUTO_TEST_CASE( SymbolAlignmentBoxRejectsUnmeasurableShape )
{
    SCH_SHAPE poly( SHAPE_T::POLY ); // no points, so the bbox is never initialised

    BOOST_CHECK( !EE_GRID_HELPER::GetSymbolAlignmentBox( &poly ).has_value() );
}


// Text extents depend on font metrics and on field visibility.  Nobody aligns to the width of a
// pin name, and guides drawn on one would look arbitrary.
BOOST_AUTO_TEST_CASE( SymbolAlignmentBoxRejectsText )
{
    SCH_TEXT text;
    BOOST_CHECK( !EE_GRID_HELPER::GetSymbolAlignmentBox( &text ).has_value() );
}


// The two rules must stay separate.  A schematic symbol is a body; in the symbol editor there is
// no such thing, and pins must not become schematic alignment targets.
BOOST_AUTO_TEST_CASE( TheTwoAlignmentRulesDoNotOverlap )
{
    SCH_PIN pin( nullptr );
    pin.SetPosition( VECTOR2I( 0, 0 ) );

    BOOST_CHECK( !EE_GRID_HELPER::GetAlignmentBox( &pin ).has_value() );

    SCH_SHEET sheet;
    BOOST_CHECK( !EE_GRID_HELPER::GetSymbolAlignmentBox( &sheet ).has_value() );
}

BOOST_AUTO_TEST_SUITE_END()
