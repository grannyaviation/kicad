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
#include <sch_bitmap.h>
#include <sch_junction.h>
#include <sch_symbol.h>
#include <lib_symbol.h>
#include <sch_label.h>
#include <sch_sheet.h>
#include <sch_sheet_pin.h>
#include <sch_pin.h>
#include <layer_ids.h>
#include <sch_field.h>
#include <sch_textbox.h>
#include <lib_symbol.h>

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


// The "!" warning shown while moving an item fires off this predicate.  A wrong answer is worse
// than no answer: a false positive marks a perfectly good library as broken, a false negative is
// exactly the silence that let a half-grid pin pitch go unnoticed in the first place.
BOOST_AUTO_TEST_CASE( OffGridDetection )
{
    const VECTOR2I grid( 100, 100 );

    SCH_LINE onGrid( VECTOR2I( 0, 0 ), LAYER_WIRE );
    onGrid.SetEndPoint( VECTOR2I( 200, 300 ) );
    BOOST_CHECK( !EE_GRID_HELPER::IsOffGrid( &onGrid, grid ) );

    SCH_LINE offGrid( VECTOR2I( 0, 0 ), LAYER_WIRE );
    offGrid.SetEndPoint( VECTOR2I( 250, 300 ) );
    BOOST_CHECK( EE_GRID_HELPER::IsOffGrid( &offGrid, grid ) );

    // C++ truncates the modulo toward zero, so -250 % 100 is -50 rather than 50.  Still non-zero,
    // but the sign flip is the kind of thing that silently disables a check on the left half of
    // a sheet.
    SCH_LINE negative( VECTOR2I( -200, -300 ), LAYER_WIRE );
    negative.SetEndPoint( VECTOR2I( -250, -300 ) );
    BOOST_CHECK( EE_GRID_HELPER::IsOffGrid( &negative, grid ) );

    // A notes line is not connectable, so its endpoints answer to nothing.
    SCH_LINE notes( VECTOR2I( 0, 0 ), LAYER_NOTES );
    notes.SetEndPoint( VECTOR2I( 250, 300 ) );
    BOOST_CHECK( !EE_GRID_HELPER::IsOffGrid( &notes, grid ) );

    // A library pin has no GetConnectionPoints(), so only the SCH_PIN_T branch can catch it --
    // and the symbol editor is where an off-grid pin gets created in the first place.
    SCH_PIN pin( nullptr );
    pin.SetPosition( VECTOR2I( 150, 0 ) );
    BOOST_CHECK( EE_GRID_HELPER::IsOffGrid( &pin, grid ) );

    pin.SetPosition( VECTOR2I( 100, 0 ) );
    BOOST_CHECK( !EE_GRID_HELPER::IsOffGrid( &pin, grid ) );

    // A grid origin shifts what counts as legal; ignoring it would warn about everything on a
    // sheet whose origin is not (0, 0).
    pin.SetPosition( VECTOR2I( 150, 0 ) );
    BOOST_CHECK( !EE_GRID_HELPER::IsOffGrid( &pin, grid, VECTOR2I( 50, 0 ) ) );

    // Text carries no connection, and a zero grid must not divide by zero.
    SCH_TEXT text;
    text.SetPosition( VECTOR2I( 150, 150 ) );
    BOOST_CHECK( !EE_GRID_HELPER::IsOffGrid( &text, grid ) );
    BOOST_CHECK( !EE_GRID_HELPER::IsOffGrid( &offGrid, VECTOR2I( 0, 0 ) ) );
}

// The third box rule.  Separate from the other two because which one applies is decided by what
// is being dragged -- so a symbol drag can never acquire a graphic target, and vice versa.
BOOST_AUTO_TEST_CASE( GraphicAlignmentBoxAcceptsGraphicLines )
{
    // A separator line is a graphic SCH_LINE.  Horizontal, so the box has zero height: kept on
    // purpose, exactly as the symbol rule keeps flat polylines.  Dropping zero-extent shapes
    // would drop the entire separator use case.
    SCH_LINE separator( VECTOR2I( 1000, 5000 ), LAYER_NOTES );
    separator.SetEndPoint( VECTOR2I( 9000, 5000 ) );

    const std::optional<BOX2I> lineBox = EE_GRID_HELPER::GetGraphicAlignmentBox( &separator );

    BOOST_REQUIRE( lineBox.has_value() );
    BOOST_CHECK_EQUAL( lineBox->GetOrigin(), VECTOR2I( 1000, 5000 ) );
    BOOST_CHECK_EQUAL( lineBox->GetEnd(), VECTOR2I( 9000, 5000 ) );
    BOOST_CHECK_EQUAL( lineBox->GetHeight(), 0 );

    // Drawn right-to-left: the box must still be normalised, or every guide against it is wrong.
    SCH_LINE backwards( VECTOR2I( 9000, 5000 ), LAYER_NOTES );
    backwards.SetEndPoint( VECTOR2I( 1000, 5000 ) );

    const std::optional<BOX2I> backBox = EE_GRID_HELPER::GetGraphicAlignmentBox( &backwards );

    BOOST_REQUIRE( backBox.has_value() );
    BOOST_CHECK_EQUAL( backBox->GetOrigin(), VECTOR2I( 1000, 5000 ) );
    BOOST_CHECK_EQUAL( backBox->GetEnd(), VECTOR2I( 9000, 5000 ) );
}


// A wire is not a graphic.  It has to keep the body rule and grid-legal snapping, or dragging one
// would silently gain the off-grid exemption that the graphics path carries.
BOOST_AUTO_TEST_CASE( GraphicAlignmentBoxRejectsConnectableLines )
{
    SCH_LINE wire( VECTOR2I( 0, 0 ), LAYER_WIRE );
    wire.SetEndPoint( VECTOR2I( 1000, 0 ) );

    BOOST_CHECK( !EE_GRID_HELPER::GetGraphicAlignmentBox( &wire ).has_value() );
}


// Same stroke deflation as the symbol rule: EDA_SHAPE::getBoundingBox() inflates by half the
// stroke, and half a stroke is not a whole grid step.
BOOST_AUTO_TEST_CASE( GraphicAlignmentBoxDeflatesShapeStroke )
{
    SCH_SHAPE rect( SHAPE_T::RECTANGLE );
    rect.SetStart( VECTOR2I( 0, 0 ) );
    rect.SetEnd( VECTOR2I( 2540, 2540 ) );
    rect.SetWidth( 254 );

    // Precondition, as the symbol-rule test does: the inflated box really is bigger, or this
    // test would pass against an implementation that deflates nothing.
    BOOST_REQUIRE_EQUAL( rect.GetBoundingBox().GetOrigin(), VECTOR2I( -127, -127 ) );

    const std::optional<BOX2I> box = EE_GRID_HELPER::GetGraphicAlignmentBox( &rect );

    BOOST_REQUIRE( box.has_value() );
    BOOST_CHECK_EQUAL( box->GetOrigin(), VECTOR2I( 0, 0 ) );
    BOOST_CHECK_EQUAL( box->GetEnd(), VECTOR2I( 2540, 2540 ) );
}


// Text is excluded by all three rules: font metrics make it a poor alignment reference.
BOOST_AUTO_TEST_CASE( GraphicAlignmentBoxRejectsTextAndBodies )
{
    SCH_TEXT text;
    BOOST_CHECK( !EE_GRID_HELPER::GetGraphicAlignmentBox( &text ).has_value() );

    SCH_SHEET sheet;
    BOOST_CHECK( !EE_GRID_HELPER::GetGraphicAlignmentBox( &sheet ).has_value() );

    SCH_PIN pin( nullptr );
    BOOST_CHECK( !EE_GRID_HELPER::GetGraphicAlignmentBox( &pin ).has_value() );
}


// The invariant that matters is NOT "no type is accepted by more than one rule".  SCH_SHAPE_T is
// deliberately accepted by both the symbol rule and the graphic rule, and that is harmless --
// they never apply in the same editor.  What must stay disjoint is the pair that competes: both
// schematic rules, chosen per drag.  Overlap there makes a single drag ambiguous.
BOOST_AUTO_TEST_CASE( TheTwoSchematicRulesDoNotOverlap )
{
    SCH_SHEET sheet;
    BOOST_CHECK( EE_GRID_HELPER::GetAlignmentBox( &sheet ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetGraphicAlignmentBox( &sheet ).has_value() );

    SCH_LINE separator( VECTOR2I( 0, 0 ), LAYER_NOTES );
    separator.SetEndPoint( VECTOR2I( 1000, 0 ) );
    BOOST_CHECK( !EE_GRID_HELPER::GetAlignmentBox( &separator ).has_value() );
    BOOST_CHECK( EE_GRID_HELPER::GetGraphicAlignmentBox( &separator ).has_value() );
}

// A bitmap with no image loaded does not produce an *invalid* box -- ByCenter() marks it
// initialised whatever the size -- so it arrives as a valid zero-size box and IsValid() cannot
// catch it.  An item with nothing drawn must not become an alignment target: these guides have
// already shipped one bug where they snapped to symbol geometry that was not on screen.
BOOST_AUTO_TEST_CASE( GraphicAlignmentBoxRejectsAnImagelessBitmap )
{
    SCH_BITMAP bitmap( VECTOR2I( 2540, 2540 ) );

    // Precondition: the box really is valid and really is empty, or this test proves nothing.
    BOOST_REQUIRE( bitmap.GetBoundingBox().IsValid() );
    BOOST_REQUIRE_EQUAL( bitmap.GetBoundingBox().GetWidth(), 0 );

    BOOST_CHECK( !EE_GRID_HELPER::GetGraphicAlignmentBox( &bitmap ).has_value() );
}

// A hierarchical sheet's pins are what the user lines up when a sub-sheet's inputs on one border
// have to sit level with its outputs on the other.  Measured as a point, for the reason symbol
// pins are: the connection point is the thing being aligned, and a zero-size box makes both
// pin-to-pin alignment and equal pin pitch fall out of the engine unchanged.
BOOST_AUTO_TEST_CASE( SheetPinAlignmentBoxIsThePinPoint )
{
    SCH_SHEET      sheet;
    SCH_SHEET_PIN  pin( &sheet, VECTOR2I( 2540, -1270 ), wxT( "MISO" ) );

    const std::optional<BOX2I> box = EE_GRID_HELPER::GetSheetPinAlignmentBox( &pin );

    BOOST_REQUIRE( box.has_value() );
    BOOST_CHECK_EQUAL( box->GetOrigin(), pin.GetPosition() );
    BOOST_CHECK_EQUAL( box->GetWidth(), 0 );
    BOOST_CHECK_EQUAL( box->GetHeight(), 0 );

    // The rule is exclusive, like the other three: a sheet-pin drag must not acquire sheet or
    // graphic targets, and a sheet drag must not acquire pin targets.
    BOOST_CHECK( !EE_GRID_HELPER::GetSheetPinAlignmentBox( &sheet ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetAlignmentBox( &pin ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetGraphicAlignmentBox( &pin ).has_value() );
}


// Which box rule applies is decided by the gesture, and "moving a sheet pin" has to survive the
// wires a drag hauls in alongside the pin -- an all-of test over the selection would not.
BOOST_AUTO_TEST_CASE( SheetPinSelectionSurvivesDragAdditions )
{
    SCH_SHEET     sheet;
    SCH_SHEET_PIN pin( &sheet, VECTOR2I( 0, 0 ), wxT( "MISO" ) );
    SCH_LINE      wire( VECTOR2I( 0, 0 ), LAYER_WIRE );
    SCH_SELECTION sel;

    BOOST_CHECK( !EE_GRID_HELPER::IsSheetPinSelection( sel ) );

    sel.Add( &pin );
    BOOST_CHECK( EE_GRID_HELPER::IsSheetPinSelection( sel ) );

    // A drag adds the pin's connected wire to the same selection.  Still a pin gesture.
    sel.Add( &wire );
    BOOST_CHECK( EE_GRID_HELPER::IsSheetPinSelection( sel ) );

    // A body in the selection is not: that is a sheet/symbol move that happens to carry a pin,
    // and it has to keep the body rule or the bodies would chase pin points.
    sel.Add( &sheet );
    BOOST_CHECK( !EE_GRID_HELPER::IsSheetPinSelection( sel ) );
}


// A net label had no rule at all, so dragging one produced an invalid moving box, no move
// context and no guides.  It gets the connection point, not the drawn chevron: a global label's
// outline widens with the net name, so two labels on one column would not line up by their boxes.
BOOST_AUTO_TEST_CASE( LabelAlignmentBoxIsTheConnectionPoint )
{
    SCH_LABEL       label( VECTOR2I( 2540, -1270 ), wxT( "+5V" ) );
    SCH_GLOBALLABEL global( VECTOR2I( 5080, 0 ), wxT( "+5V" ) );
    SCH_HIERLABEL   hier( VECTOR2I( 0, 2540 ), wxT( "MISO" ) );

    for( const SCH_LABEL_BASE* item : { static_cast<const SCH_LABEL_BASE*>( &label ),
                                        static_cast<const SCH_LABEL_BASE*>( &global ),
                                        static_cast<const SCH_LABEL_BASE*>( &hier ) } )
    {
        const std::optional<BOX2I> box = EE_GRID_HELPER::GetLabelAlignmentBox( item );

        BOOST_REQUIRE( box.has_value() );

        // The point ERC and the connectivity engine both read, so a label aligned by it stays a
        // whole number of grid steps from every pin it could attach to.
        BOOST_REQUIRE_EQUAL( item->GetConnectionPoints().size(), 1 );
        BOOST_CHECK_EQUAL( box->GetOrigin(), item->GetConnectionPoints().front() );
        BOOST_CHECK_EQUAL( box->GetWidth(), 0 );
        BOOST_CHECK_EQUAL( box->GetHeight(), 0 );
    }

    // Exclusive, like the other rules: a label drag must not acquire graphic or text targets,
    // and no other gesture may measure a label by this rule.  The text rule in particular --
    // a label derives from SCH_TEXT, and picking it up there would make it grid-exempt.
    SCH_TEXT text;
    text.SetText( wxT( "Notes" ) );

    BOOST_CHECK( !EE_GRID_HELPER::GetTextAlignmentBox( &label ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetGraphicAlignmentBox( &label ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetAlignmentBox( &label ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetLabelAlignmentBox( &text ).has_value() );
}


// A power port is a SCH_SYMBOL, and the body rule refuses it so a component drag never chases a
// GND flag.  That refusal used to leave a dragged power port with no box from any rule, hence no
// move context and no guides.  It belongs to the label rule instead, measured by its one pin.
BOOST_AUTO_TEST_CASE( PowerPortAlignsByItsPin )
{
    LIB_SYMBOL part( wxT( "+5V" ) );
    part.SetGlobalPower();

    // Deliberately not at the origin, or the two candidate answers below would coincide and the
    // check would prove nothing.
    SCH_PIN* libPin = new SCH_PIN( &part );
    libPin->SetPosition( VECTOR2I( 0, 2540 ) );
    part.AddDrawItem( libPin );

    SCH_SYMBOL port( part, part.GetLibId(), nullptr, 1, 0, VECTOR2I( 2540, -1270 ) );
    BOOST_REQUIRE( port.IsPower() );
    BOOST_REQUIRE_EQUAL( port.GetPins().size(), 1 );

    const std::optional<BOX2I> box = EE_GRID_HELPER::GetLabelAlignmentBox( &port );

    BOOST_REQUIRE( box.has_value() );

    // The pin, not the symbol origin: a library author is free to draw the flag anywhere
    // relative to the origin, and what the user lines up is where the wire attaches.
    BOOST_CHECK_EQUAL( box->GetOrigin(), port.GetPins().front()->GetPosition() );
    BOOST_CHECK( box->GetOrigin() != port.GetPosition() );
    BOOST_CHECK_EQUAL( box->GetWidth(), 0 );
    BOOST_CHECK_EQUAL( box->GetHeight(), 0 );

    // The body rule still refuses it, so a power port stays out of every component drag's
    // target list -- which is the whole reason it was excluded there.
    BOOST_CHECK( !EE_GRID_HELPER::GetAlignmentBox( &port ).has_value() );

    // And the gesture test routes the drag to the label rule.
    SCH_SELECTION sel;
    sel.Add( &port );
    BOOST_CHECK( EE_GRID_HELPER::IsLabelSelection( sel ) );

    // A component is not a power port: it keeps the body rule, and a drag of one is not a label
    // gesture even though both are SCH_SYMBOLs.
    LIB_SYMBOL resistorPart( wxT( "R" ) );
    SCH_PIN*   resistorPin = new SCH_PIN( &resistorPart );
    resistorPin->SetPosition( VECTOR2I( 0, 0 ) );
    resistorPart.AddDrawItem( resistorPin );

    SCH_SYMBOL resistor( resistorPart, resistorPart.GetLibId(), nullptr, 1 );
    BOOST_CHECK( !EE_GRID_HELPER::GetLabelAlignmentBox( &resistor ).has_value() );

    SCH_SELECTION bodySel;
    bodySel.Add( &resistor );
    BOOST_CHECK( !EE_GRID_HELPER::IsLabelSelection( bodySel ) );
}


// Same shape as the sheet-pin test, and for the same reason: dragging a label hauls the wire it
// sits on into the selection, so an all-of test would call it something other than a label move.
BOOST_AUTO_TEST_CASE( LabelSelectionSurvivesDragAdditions )
{
    SCH_LABEL     label( VECTOR2I( 0, 0 ), wxT( "+5V" ) );
    SCH_LINE      wire( VECTOR2I( 0, 0 ), LAYER_WIRE );
    SCH_SHEET     sheet;
    SCH_SELECTION sel;

    BOOST_CHECK( !EE_GRID_HELPER::IsLabelSelection( sel ) );

    sel.Add( &label );
    BOOST_CHECK( EE_GRID_HELPER::IsLabelSelection( sel ) );

    sel.Add( &wire );
    BOOST_CHECK( EE_GRID_HELPER::IsLabelSelection( sel ) );

    // A body is a sheet/symbol move carrying a label along, and has to keep the body rule.
    sel.Add( &sheet );
    BOOST_CHECK( !EE_GRID_HELPER::IsLabelSelection( sel ) );
}


// Text was excluded from the guides originally because font metrics make a poor reference.  The
// exclusion is lifted deliberately: what the user wants lined up is a column of reference
// designators, and that means the box the eye sees, not the anchor point the file stores.
BOOST_AUTO_TEST_CASE( TextAlignmentBoxIsTheTextBox )
{
    SCH_TEXT text;
    text.SetText( wxT( "Notes" ) );
    text.SetPosition( VECTOR2I( 2540, -1270 ) );

    const std::optional<BOX2I> textBox = EE_GRID_HELPER::GetTextAlignmentBox( &text );

    BOOST_REQUIRE( textBox.has_value() );
    BOOST_CHECK_EQUAL( textBox->GetOrigin(), text.GetBoundingBox().GetOrigin() );
    BOOST_CHECK_EQUAL( textBox->GetEnd(), text.GetBoundingBox().GetEnd() );
    BOOST_CHECK( textBox->GetWidth() > 0 );
    BOOST_CHECK( textBox->GetHeight() > 0 );

    SCH_SHEET sheet;
    SCH_FIELD field( &sheet, FIELD_T::USER, wxT( "Ref" ) );
    field.SetText( wxT( "U1" ) );
    field.SetVisible( true );

    const std::optional<BOX2I> fieldBox = EE_GRID_HELPER::GetTextAlignmentBox( &field );

    BOOST_REQUIRE( fieldBox.has_value() );
    BOOST_CHECK_EQUAL( fieldBox->GetOrigin(), field.GetBoundingBox().GetOrigin() );
    BOOST_CHECK_EQUAL( fieldBox->GetEnd(), field.GetBoundingBox().GetEnd() );

    // A guide against text nobody can see is a lie, and an empty field has no box worth drawing.
    field.SetVisible( false );
    BOOST_CHECK( !EE_GRID_HELPER::GetTextAlignmentBox( &field ).has_value() );

    field.SetVisible( true );
    field.SetText( wxEmptyString );
    BOOST_CHECK( !EE_GRID_HELPER::GetTextAlignmentBox( &field ).has_value() );

    // Exclusive against the other four rules, like each of them is against the rest.
    BOOST_CHECK( !EE_GRID_HELPER::GetTextAlignmentBox( &sheet ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetAlignmentBox( &text ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetGraphicAlignmentBox( &text ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetSymbolAlignmentBox( &text ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetSheetPinAlignmentBox( &text ).has_value() );

    // A field is exclusive against the other four rules too -- checked separately from the
    // SCH_TEXT instance above, since a field's body ownership makes it a different degenerate
    // case from a free-standing text item.
    BOOST_CHECK( !EE_GRID_HELPER::GetAlignmentBox( &field ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetGraphicAlignmentBox( &field ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetSymbolAlignmentBox( &field ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetSheetPinAlignmentBox( &field ).has_value() );
}


// GetTextBox() measures GetShownText(true), which for a field with its name shown prepends
// "Name: " even when the stored value is empty.  The emptiness guard in GetTextAlignmentBox()
// has to agree with that or a real, hit-testable box gets rejected as if there were nothing
// there -- silently, since a dropped guide leaves no message on screen.
BOOST_AUTO_TEST_CASE( TextAlignmentBoxGuardMatchesShownText )
{
    SCH_SHEET sheet;
    SCH_FIELD field( &sheet, FIELD_T::USER, wxT( "Rev" ) );
    field.SetText( wxEmptyString );
    field.SetVisible( true );
    field.SetNameShown( true );

    const std::optional<BOX2I> shownBox = EE_GRID_HELPER::GetTextAlignmentBox( &field );

    BOOST_REQUIRE( shownBox.has_value() );
    BOOST_CHECK( shownBox->GetWidth() > 0 );
    BOOST_CHECK( shownBox->GetHeight() > 0 );

    // Name hidden again: an empty value with nothing shown really does draw nothing, and must
    // stay rejected.
    field.SetNameShown( false );
    BOOST_CHECK( !EE_GRID_HELPER::GetTextAlignmentBox( &field ).has_value() );
}


// All-of, unlike the sheet-pin predicate.  A sheet pin drag hauls its connected wires into the
// selection and the predicate has to tolerate them; neither a field nor free text connects to
// anything, so there are no drag additions and the stricter test is the correct one.
BOOST_AUTO_TEST_CASE( TextSelectionRejectsBodies )
{
    SCH_SHEET sheet;
    SCH_FIELD field( &sheet, FIELD_T::USER, wxT( "Ref" ) );
    field.SetText( wxT( "U1" ) );
    field.SetVisible( true );

    SCH_TEXT text;
    text.SetText( wxT( "Notes" ) );

    SCH_SELECTION sel;
    BOOST_CHECK( !EE_GRID_HELPER::IsTextSelection( sel ) );

    sel.Add( &field );
    BOOST_CHECK( EE_GRID_HELPER::IsTextSelection( sel ) );

    sel.Add( &text );
    BOOST_CHECK( EE_GRID_HELPER::IsTextSelection( sel ) );

    // A body in the selection is a symbol or sheet move carrying its fields along, and it has to
    // keep the body rule or the body would chase its own reference designator.
    sel.Add( &sheet );
    BOOST_CHECK( !EE_GRID_HELPER::IsTextSelection( sel ) );
}


// A field owned by a LIB_SYMBOL, i.e. one being dragged in the symbol editor.  Different enough
// from the sheet-owned case above to be worth its own check: GetBoundingBox() applies a parent
// transform only for SCH_SYMBOL_T, GetShownText() takes a whole different branch when there is no
// SCHEMATIC to resolve variables against, and the symbol editor has neither.  A degenerate box
// here would leave the drag with an invalid moving box and therefore no guides at all -- silently,
// which is exactly how the missing symbol-editor support presented.
BOOST_AUTO_TEST_CASE( LibSymbolFieldAlignsAsText )
{
    LIB_SYMBOL symbol( wxT( "R" ) );

    SCH_FIELD* reference = symbol.GetField( FIELD_T::REFERENCE );
    BOOST_REQUIRE( reference );
    reference->SetText( wxT( "R" ) );
    reference->SetVisible( true );

    const std::optional<BOX2I> refBox = EE_GRID_HELPER::GetTextAlignmentBox( reference );

    BOOST_REQUIRE( refBox.has_value() );
    BOOST_CHECK( refBox->GetWidth() > 0 );
    BOOST_CHECK( refBox->GetHeight() > 0 );
    BOOST_CHECK_EQUAL( refBox->GetOrigin(), reference->GetBoundingBox().GetOrigin() );

    // The gesture test is what routes the drag to the text rule instead of the symbol editor's
    // own pin-and-shape rule, so the two must agree on a lib field.
    SCH_SELECTION sel;
    sel.Add( reference );
    BOOST_CHECK( EE_GRID_HELPER::IsTextSelection( sel ) );

    // And the symbol editor's own rule still refuses it, so the two never both claim the drag.
    BOOST_CHECK( !EE_GRID_HELPER::GetSymbolAlignmentBox( reference ).has_value() );
}


// A text box is a drawn rectangle that happens to contain text, which is why GetItemGrid() already
// puts it on the graphic grid -- its border is the thing that has to sit somewhere sensible.  It
// therefore belongs to the graphic rule, not the text one: it aligns to logos, separators and the
// drawing sheet, and never chases a reference designator.
BOOST_AUTO_TEST_CASE( TextBoxAlignsAsAGraphic )
{
    SCH_TEXTBOX box;
    box.SetStart( VECTOR2I( 1000, 2000 ) );
    box.SetEnd( VECTOR2I( 6000, 5000 ) );
    box.SetWidth( 200 );

    const std::optional<BOX2I> graphicBox = EE_GRID_HELPER::GetGraphicAlignmentBox( &box );

    BOOST_REQUIRE( graphicBox.has_value() );

    // The authored rectangle, not the half-stroke halo EDA_SHAPE::getBoundingBox() adds.  Half a
    // stroke is not a whole grid step, so a box measured that way could never align on grid to a
    // shape drawn with a different pen width.
    BOOST_CHECK_EQUAL( graphicBox->GetOrigin(), VECTOR2I( 1000, 2000 ) );
    BOOST_CHECK_EQUAL( graphicBox->GetEnd(), VECTOR2I( 6000, 5000 ) );

    // Exclusive against the other four, so a text box drag can never acquire text or body targets.
    BOOST_CHECK( !EE_GRID_HELPER::GetTextAlignmentBox( &box ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetAlignmentBox( &box ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetSheetPinAlignmentBox( &box ).has_value() );
    BOOST_CHECK( !EE_GRID_HELPER::GetSymbolAlignmentBox( &box ).has_value() );
}


// EE_GRID_HELPER is default-constructible with no tool manager, which is how every test above
// uses it, and how it is briefly constructed in some tool paths.  The drawing-sheet sweep must
// be a safe no-op there rather than dereferencing its way to a frame that does not exist.
BOOST_AUTO_TEST_CASE( CollectAlignmentNeighborsWithoutAFrameIsSafe )
{
    EE_GRID_HELPER helper;
    SCH_SELECTION  empty;

    BOOST_CHECK_NO_THROW( helper.CollectAlignmentNeighbors( empty ) );
}

BOOST_AUTO_TEST_SUITE_END()
