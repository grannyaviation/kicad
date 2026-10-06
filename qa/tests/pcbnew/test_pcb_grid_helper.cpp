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

#include <algorithm>
#include <memory>
#include <tuple>

#include <tools/pcb_grid_helper.h>
#include <geometry/seg.h>
#include <geometry/shape_arc.h>
#include <footprint.h>
#include <pad.h>
#include <pcb_shape.h>
#include <pcb_track.h>
#include <zone.h>
#include <pcb_text.h>

// Mock EDA_ITEM class for testing GetItemGrid
class MOCK_BOARD_ITEM : public BOARD_ITEM
{
public:
    MOCK_BOARD_ITEM( KICAD_T aType ) : BOARD_ITEM( nullptr, aType ) {}

    // Required virtual functions
    wxString GetClass() const override { return "MockEDAItem"; }
    void Move( const VECTOR2I& aMoveVector ) override {}
    VECTOR2I GetPosition() const override { return VECTOR2I( 0, 0 ); }
    void SetPosition( const VECTOR2I& aPos ) override {}
    BOARD_ITEM* Clone() const override { return new MOCK_BOARD_ITEM( Type() ); }

    // Implement pure virtuals from BOARD_ITEM
    double Similarity( const BOARD_ITEM& aItem ) const override { return this == &aItem ? 1.0 : 0.0; }
    bool operator==( const BOARD_ITEM& aItem ) const override { return this == &aItem; }
};

// Test fixture for accessing protected members
class PCBGridHelperTestFixture
{
public:
    PCBGridHelperTestFixture()
    {
        helper.SetGridSize( VECTOR2D( 100, 100 ) );
        helper.SetOrigin( VECTOR2I( 0, 0 ) );
        helper.SetGridSnapping( true );
        helper.SetSnap( true );
    }

    PCB_GRID_HELPER helper;
};

BOOST_AUTO_TEST_SUITE( PCBGridHelperTest )

BOOST_AUTO_TEST_CASE( DefaultConstructor )
{
    PCB_GRID_HELPER helper;

    // Test default state matches base class
    BOOST_CHECK( helper.GetSnap() );
    BOOST_CHECK( helper.GetUseGrid() );

    // Test that GetSnapped returns nullptr initially
    BOOST_CHECK( helper.GetSnapped() == nullptr );
}

BOOST_AUTO_TEST_CASE( AlignToSegmentBasic )
{
    PCB_GRID_HELPER helper;
    helper.SetGridSize( VECTOR2D( 100, 100 ) );
    helper.SetOrigin( VECTOR2I( 0, 0 ) );
    helper.SetGridSnapping( true );

    // Horizontal segment
    SEG seg( VECTOR2I( 0, 0 ), VECTOR2I( 300, 0 ) );
    VECTOR2I result = helper.AlignToSegment( VECTOR2I( 150, 50 ), seg );

    // Should snap to grid point that intersects with segment
    BOOST_CHECK_EQUAL( result.x, 200 );
    BOOST_CHECK_EQUAL( result.y, 0 );
}

BOOST_AUTO_TEST_CASE( AlignToSegmentVertical )
{
    PCB_GRID_HELPER helper;
    helper.SetGridSize( VECTOR2D( 100, 100 ) );
    helper.SetOrigin( VECTOR2I( 0, 0 ) );
    helper.SetGridSnapping( true );

    // Vertical segment
    SEG seg( VECTOR2I( 200, 0 ), VECTOR2I( 200, 400 ) );
    VECTOR2I result = helper.AlignToSegment( VECTOR2I( 150, 150 ), seg );

    // Should snap to intersection with vertical line
    BOOST_CHECK_EQUAL( result.x, 200 );
    BOOST_CHECK_EQUAL( result.y, 200 );
}

BOOST_AUTO_TEST_CASE( AlignToSegmentDiagonal )
{
    PCB_GRID_HELPER helper;
    helper.SetGridSize( VECTOR2D( 100, 100 ) );
    helper.SetOrigin( VECTOR2I( 0, 0 ) );
    helper.SetGridSnapping( true );

    // Diagonal segment
    SEG seg( VECTOR2I( 0, 0 ), VECTOR2I( 400, 400 ) );
    VECTOR2I result = helper.AlignToSegment( VECTOR2I( 150, 250 ), seg );

    // 250,250 is the closest point to the nearest grid point to the segment.
    // First, it snaps to grid point (200, 300) and then finds the closest point on the segment.
    // So the result should be (250, 250).
    BOOST_CHECK_EQUAL( result.x, 250 );
    BOOST_CHECK_EQUAL( result.y, 250 );
}

BOOST_AUTO_TEST_CASE( AlignToSegmentEndpoints )
{
    PCB_GRID_HELPER helper;
    helper.SetGridSize( VECTOR2D( 100, 100 ) );
    helper.SetOrigin( VECTOR2I( 0, 0 ) );
    helper.SetGridSnapping( true );

    SEG seg( VECTOR2I( 50, 50 ), VECTOR2I( 150, 50 ) );

    // Point very close to start endpoint
    VECTOR2I result = helper.AlignToSegment( VECTOR2I( 55, 55 ), seg );
    BOOST_CHECK_EQUAL( result.x, 50 );
    BOOST_CHECK_EQUAL( result.y, 50 );

    // Point very close to end endpoint
    result = helper.AlignToSegment( VECTOR2I( 145, 55 ), seg );
    BOOST_CHECK_EQUAL( result.x, 150 );
    BOOST_CHECK_EQUAL( result.y, 50 );
}

BOOST_AUTO_TEST_CASE( AlignToSegmentSnapDisabled )
{
    PCB_GRID_HELPER helper;
    helper.SetGridSize( VECTOR2D( 100, 100 ) );
    helper.SetOrigin( VECTOR2I( 0, 0 ) );
    helper.SetGridSnapping( true );
    helper.SetSnap( false );  // Disable snapping

    SEG seg( VECTOR2I( 0, 0 ), VECTOR2I( 300, 0 ) );
    VECTOR2I point( 150, 50 );
    VECTOR2I result = helper.AlignToSegment( point, seg );

    // Should return grid-aligned point when snap is disabled
    VECTOR2I expected = helper.Align( point );
    BOOST_CHECK_EQUAL( result.x, expected.x );
    BOOST_CHECK_EQUAL( result.y, expected.y );
}

BOOST_AUTO_TEST_CASE( AlignToArcBasic )
{
    PCB_GRID_HELPER helper;
    helper.SetGridSize( VECTOR2D( 100, 100 ) );
    helper.SetOrigin( VECTOR2I( 0, 0 ) );
    helper.SetGridSnapping( true );

    // Create a simple arc (quarter circle)
    SHAPE_ARC arc( VECTOR2I( 100, 0 ), VECTOR2I( 0, 100 ), ANGLE_90 );

    VECTOR2I result = helper.AlignToArc( VECTOR2I( 50, 50 ), arc );

    // Should snap to arc endpoints or intersections
    BOOST_CHECK( result.x >= 0 );
    BOOST_CHECK( result.y >= 0 );
}

// TODO: Fix broken AlignToArc Routine
// BOOST_AUTO_TEST_CASE( AlignToArcEndpoints )
// {
//     PCB_GRID_HELPER helper;
//     helper.SetGridSize( VECTOR2D( 100, 100 ) );
//     helper.SetOrigin( VECTOR2I( 0, 0 ) );
//     helper.SetGridSnapping( true );

//     SHAPE_ARC arc( VECTOR2I( 100, 0 ), VECTOR2I( 65, 65 ), VECTOR2I( 0, 100 ), 0 );

//     // Point close to start
//     VECTOR2I result = helper.AlignToArc( VECTOR2I( 95, 5 ), arc );
//     BOOST_CHECK_EQUAL( result.x, arc.GetP0().x );
//     BOOST_CHECK_EQUAL( result.y, arc.GetP0().y );

//     // Point close to end
//     result = helper.AlignToArc( VECTOR2I( 5, 95 ), arc );
//     BOOST_CHECK_EQUAL( result.x, arc.GetP1().x );
//     BOOST_CHECK_EQUAL( result.y, arc.GetP1().y );
// }

BOOST_AUTO_TEST_CASE( AlignToArcSnapDisabled )
{
    PCB_GRID_HELPER helper;
    helper.SetGridSize( VECTOR2D( 100, 100 ) );
    helper.SetOrigin( VECTOR2I( 0, 0 ) );
    helper.SetGridSnapping( true );
    helper.SetSnap( false );  // Disable snapping

    SHAPE_ARC arc( VECTOR2I( 100, 0 ), VECTOR2I( 0, 100 ), ANGLE_90 );
    VECTOR2I point( 50, 50 );
    VECTOR2I result = helper.AlignToArc( point, arc );

    // Should return grid-aligned point when snap is disabled
    VECTOR2I expected = helper.Align( point );
    BOOST_CHECK_EQUAL( result.x, expected.x );
    BOOST_CHECK_EQUAL( result.y, expected.y );
}

BOOST_AUTO_TEST_CASE( GetItemGridFootprint )
{
    PCB_GRID_HELPER helper;

    MOCK_BOARD_ITEM footprint( PCB_FOOTPRINT_T );
    GRID_HELPER_GRIDS grid = helper.GetItemGrid( &footprint );
    BOOST_CHECK_EQUAL( grid, GRID_CONNECTABLE );
}

BOOST_AUTO_TEST_CASE( GetItemGridPad )
{
    PCB_GRID_HELPER helper;

    MOCK_BOARD_ITEM pad( PCB_PAD_T );
    GRID_HELPER_GRIDS grid = helper.GetItemGrid( &pad );
    BOOST_CHECK_EQUAL( grid, GRID_CONNECTABLE );
}

BOOST_AUTO_TEST_CASE( GetItemGridText )
{
    PCB_GRID_HELPER helper;

    MOCK_BOARD_ITEM text( PCB_TEXT_T );
    GRID_HELPER_GRIDS grid = helper.GetItemGrid( &text );
    BOOST_CHECK_EQUAL( grid, GRID_TEXT );

    MOCK_BOARD_ITEM field( PCB_FIELD_T );
    grid = helper.GetItemGrid( &field );
    BOOST_CHECK_EQUAL( grid, GRID_TEXT );
}

BOOST_AUTO_TEST_CASE( GetItemGridGraphics )
{
    PCB_GRID_HELPER helper;

    MOCK_BOARD_ITEM shape( PCB_SHAPE_T );
    GRID_HELPER_GRIDS grid = helper.GetItemGrid( &shape );
    BOOST_CHECK_EQUAL( grid, GRID_GRAPHICS );

    MOCK_BOARD_ITEM dimension( PCB_DIMENSION_T );
    grid = helper.GetItemGrid( &dimension );
    BOOST_CHECK_EQUAL( grid, GRID_GRAPHICS );

    MOCK_BOARD_ITEM refImage( PCB_REFERENCE_IMAGE_T );
    grid = helper.GetItemGrid( &refImage );
    BOOST_CHECK_EQUAL( grid, GRID_GRAPHICS );

    MOCK_BOARD_ITEM textbox( PCB_TEXTBOX_T );
    grid = helper.GetItemGrid( &textbox );
    BOOST_CHECK_EQUAL( grid, GRID_GRAPHICS );
}

BOOST_AUTO_TEST_CASE( GetItemGridTracks )
{
    PCB_GRID_HELPER helper;

    MOCK_BOARD_ITEM trace( PCB_TRACE_T );
    GRID_HELPER_GRIDS grid = helper.GetItemGrid( &trace );
    BOOST_CHECK_EQUAL( grid, GRID_WIRES );

    MOCK_BOARD_ITEM arc( PCB_ARC_T );
    grid = helper.GetItemGrid( &arc );
    BOOST_CHECK_EQUAL( grid, GRID_WIRES );
}

BOOST_AUTO_TEST_CASE( GetItemGridVias )
{
    PCB_GRID_HELPER helper;

    MOCK_BOARD_ITEM via( PCB_VIA_T );
    GRID_HELPER_GRIDS grid = helper.GetItemGrid( &via );
    BOOST_CHECK_EQUAL( grid, GRID_VIAS );
}

BOOST_AUTO_TEST_CASE( GetItemGridDefault )
{
    PCB_GRID_HELPER helper;

    // Test with unknown item type
    MOCK_BOARD_ITEM unknown( PCB_ZONE_T );
    GRID_HELPER_GRIDS grid = helper.GetItemGrid( &unknown );
    BOOST_CHECK_EQUAL( grid, GRID_CURRENT );

    // Test with nullptr
    grid = helper.GetItemGrid( nullptr );
    BOOST_CHECK_EQUAL( grid, GRID_CURRENT );
}

BOOST_AUTO_TEST_CASE( GetSnappedInitiallyNull )
{
    PCB_GRID_HELPER helper;

    BOARD_ITEM* snapped = helper.GetSnapped();
    BOOST_CHECK( snapped == nullptr );
}

BOOST_AUTO_TEST_CASE( OnBoardItemRemovedClearsSnap )
{
    PCB_GRID_HELPER helper;

    // Create a mock board and item - this test verifies the interface exists
    // In a real scenario, the snap item would be set by other operations
    BOARD board;
    MOCK_BOARD_ITEM item( PCB_TRACE_T );

    // This should not crash even if no snap item is set
    helper.OnBoardItemRemoved( board, static_cast<BOARD_ITEM*>( &item ) );

    BOOST_CHECK( helper.GetSnapped() == nullptr );
}


BOOST_AUTO_TEST_CASE( GeometricSnapTolerance )
{
    PCB_GRID_HELPER helper;
    helper.SetGridSize( VECTOR2D( 100, 100 ) );
    helper.SetOrigin( VECTOR2I( 0, 0 ) );
    helper.SetGridSnapping( true );

    SEG seg( VECTOR2I( 0, 0 ), VECTOR2I( 200, 0 ) );

    // Test that points very far from segment don't snap to unrealistic locations
    VECTOR2I result = helper.AlignToSegment( VECTOR2I( 50, 100000 ), seg );

    // Should snap to midpoint of segment
    BOOST_CHECK( result == VECTOR2I( 100, 0 ) );
}

BOOST_AUTO_TEST_CASE( SegmentIntersectionPriority )
{
    PCB_GRID_HELPER helper;
    helper.SetGridSize( VECTOR2D( 50, 50 ) );
    helper.SetOrigin( VECTOR2I( 0, 0 ) );
    helper.SetGridSnapping( true );

    // Segment that passes through a grid point
    SEG seg( VECTOR2I( 0, 25 ), VECTOR2I( 100, 25 ) );

    // Point near a grid intersection with the segment
    VECTOR2I result = helper.AlignToSegment( VECTOR2I( 48, 27 ), seg );

    // Should snap to the intersection at (50, 25)
    BOOST_CHECK_EQUAL( result.x, 50 );
    BOOST_CHECK_EQUAL( result.y, 25 );
}

BOOST_AUTO_TEST_CASE( ArcIntersectionWithGrid )
{
    PCB_GRID_HELPER helper;
    helper.SetGridSize( VECTOR2D( 50, 50 ) );
    helper.SetOrigin( VECTOR2I( 0, 0 ) );
    helper.SetGridSnapping( true );

    // Arc that should intersect grid lines
    SHAPE_ARC arc( VECTOR2I( 50, 0 ), VECTOR2I( 50, 50 ), VECTOR2I( 0, 50 ), 0 );

    // Point that should snap to an intersection
    VECTOR2I result = helper.AlignToArc( VECTOR2I( 25, 25 ), arc );

    // Should snap to the mid point of the arc
    BOOST_CHECK_EQUAL( result.x, 50 );
    BOOST_CHECK_EQUAL( result.y, 50 );
}

BOOST_FIXTURE_TEST_CASE( LargeGridSegmentSnap, PCBGridHelperTestFixture )
{
    // Test with larger grid to verify scaling behavior
    helper.SetGridSize( VECTOR2D( 1000, 1000 ) );

    SEG seg( VECTOR2I( 500, 0 ), VECTOR2I( 500, 2000 ) );
    VECTOR2I result = helper.AlignToSegment( VECTOR2I( 400, 800 ), seg );

    // Should snap to the segment at grid intersection
    BOOST_CHECK_EQUAL( result.x, 500 );
    BOOST_CHECK_EQUAL( result.y, 1000 );
}

BOOST_FIXTURE_TEST_CASE( ZeroLengthSegment, PCBGridHelperTestFixture )
{
    // Edge case: zero-length segment (point)
    SEG seg( VECTOR2I( 100, 100 ), VECTOR2I( 100, 100 ) );
    VECTOR2I result = helper.AlignToSegment( VECTOR2I( 95, 95 ), seg );

    // Should snap to the point itself
    BOOST_CHECK_EQUAL( result.x, 100 );
    BOOST_CHECK_EQUAL( result.y, 100 );
}

/**
 * Regression test for issue 22945: pasting a grid-aligned arc landed it off grid because the
 * grid helper offered the arc's derived (off-grid) geometric center as the drag/paste origin.
 *
 * Drive PCB_GRID_HELPER::GetArcAnchors directly (the anchor-selection logic BestDragOrigin relies
 * on) so the test needs no GAL/view. The arc is built with grid-aligned start/mid/end whose
 * circumcenter is deliberately off grid, matching the arc from the report.
 */
BOOST_AUTO_TEST_CASE( ArcDragOriginStaysOnGrid )
{
    const int grid = pcbIUScale.mmToIU( 1.0 );
    VECTOR2I  arcStart( grid * 30, grid * 50 );
    VECTOR2I  arcMid( grid * 35, grid * 47 );
    VECTOR2I  arcEnd( grid * 45, grid * 50 );

    PCB_ARC arc( nullptr );
    arc.SetStart( arcStart );
    arc.SetMid( arcMid );
    arc.SetEnd( arcEnd );

    const VECTOR2I center = arc.GetCenter();

    // Guard the fixture premise: the center must be off grid for this arc, otherwise the
    // regression could not be exercised.
    BOOST_REQUIRE( center.x % grid != 0 || center.y % grid != 0 );

    auto onGrid = []( const VECTOR2I& aPt, int aGrid )
    {
        return aPt.x % aGrid == 0 && aPt.y % aGrid == 0;
    };

    // Drag-source path: no anchor may carry ORIGIN, so BestDragOrigin falls back to the
    // grid-aligned start/end/mid corners instead of the off-grid center.
    for( const PCB_GRID_HELPER::ANCHOR_SPEC& spec : PCB_GRID_HELPER::GetArcAnchors( arc, true ) )
    {
        BOOST_CHECK_MESSAGE( ( spec.flags & GRID_HELPER::ORIGIN ) == 0,
                             "Arc drag source must not offer an ORIGIN anchor at (" << spec.pos.x
                             << ", " << spec.pos.y << ")" );

        if( spec.flags & GRID_HELPER::ORIGIN )
            BOOST_CHECK_MESSAGE( onGrid( spec.pos, grid ), "ORIGIN anchor off grid" );
    }

    // The stored midpoint is exposed as a grid-aligned corner drag reference.
    bool midOffered = false;

    for( const PCB_GRID_HELPER::ANCHOR_SPEC& spec : PCB_GRID_HELPER::GetArcAnchors( arc, true ) )
    {
        if( spec.pos == arcMid && ( spec.flags & GRID_HELPER::CORNER ) )
            midOffered = true;
    }

    BOOST_CHECK( midOffered );

    // Snap-target path (aFrom=false): the center is still offered as a drag origin for other
    // items, but must not be a magnetic snap target (would pull snaps to the off-grid center,
    // unlike a straight track's midpoint).
    bool centerAsOrigin = false;

    for( const PCB_GRID_HELPER::ANCHOR_SPEC& spec : PCB_GRID_HELPER::GetArcAnchors( arc, false ) )
    {
        if( spec.pos == center )
        {
            centerAsOrigin = true;
            BOOST_CHECK( spec.flags & GRID_HELPER::ORIGIN );
            BOOST_CHECK_EQUAL( spec.flags & GRID_HELPER::SNAPPABLE, 0 );
        }
    }

    BOOST_CHECK( centerAsOrigin );
}


// With the grid in use a smart alignment guide may only move a PCB item by whole grid steps, the
// way the schematic editor's guides do; with it off (no grid, or snapping overridden) the guide
// is free.  Before this, PCB guides ignored the grid and left aligned footprints off it.
BOOST_AUTO_TEST_CASE( AlignmentGuideStepFollowsGridUse )
{
    const std::optional<VECTOR2I> onGrid =
            PCB_GRID_HELPER::AlignmentGuideStep( true, VECTOR2D( 100000, 250000 ) );

    BOOST_REQUIRE( onGrid.has_value() );
    BOOST_CHECK_EQUAL( *onGrid, VECTOR2I( 100000, 250000 ) );

    // Grid sizes are doubles; the engine takes an integer step, rounded to nearest.
    const std::optional<VECTOR2I> rounded =
            PCB_GRID_HELPER::AlignmentGuideStep( true, VECTOR2D( 127000.4, 127000.6 ) );

    BOOST_REQUIRE( rounded.has_value() );
    BOOST_CHECK_EQUAL( *rounded, VECTOR2I( 127000, 127001 ) );

    BOOST_CHECK( !PCB_GRID_HELPER::AlignmentGuideStep( false, VECTOR2D( 100000, 100000 ) ) );
}

namespace
{
// A footprint on aSide carrying one pad per entry: { number, attribute, position }.
std::unique_ptr<FOOTPRINT> makePinFootprint( PCB_LAYER_ID aSide,
        const std::vector<std::tuple<wxString, PAD_ATTRIB, VECTOR2I>>& aPads )
{
    auto fp = std::make_unique<FOOTPRINT>( nullptr );
    fp->SetLayer( aSide );

    for( const auto& [number, attrib, pos] : aPads )
    {
        PAD* pad = new PAD( fp.get() );
        pad->SetAttribute( attrib );
        pad->SetNumber( number );

        if( attrib == PAD_ATTRIB::SMD )
        {
            LSET layers = PAD::SMDMask();

            if( aSide != F_Cu )
                layers.FlipStandardLayers();

            pad->SetLayerSet( layers );
        }
        else
        {
            pad->SetLayerSet( PAD::PTHMask() );
        }

        pad->SetPosition( pos );
        fp->Add( pad, ADD_MODE::APPEND ); // INSERT would reverse the pad order
    }

    return fp;
}
} // namespace


BOOST_AUTO_TEST_CASE( PinTargetsKeepSameSideAndOppositeThroughHole )
{
    auto sameSide = makePinFootprint( B_Cu, { { "1", PAD_ATTRIB::SMD, VECTOR2I( 100, 0 ) },
                                              { "", PAD_ATTRIB::SMD, VECTOR2I( 110, 0 ) },     // paste-only aperture
                                              { "2", PAD_ATTRIB::NPTH, VECTOR2I( 120, 0 ) } } );
    auto otherSide = makePinFootprint( F_Cu, { { "1", PAD_ATTRIB::SMD, VECTOR2I( 200, 0 ) },
                                               { "2", PAD_ATTRIB::PTH, VECTOR2I( 210, 0 ) } } );

    const std::vector<VECTOR2I> got = PCB_GRID_HELPER::CollectPinTargets(
            { sameSide.get(), otherSide.get() }, B_Cu, VECTOR2I( 0, 0 ), 400 );

    BOOST_CHECK_EQUAL( got.size(), 2 );
    BOOST_CHECK( std::find( got.begin(), got.end(), VECTOR2I( 100, 0 ) ) != got.end() );
    BOOST_CHECK( std::find( got.begin(), got.end(), VECTOR2I( 210, 0 ) ) != got.end() );
}


BOOST_AUTO_TEST_CASE( PinTargetsKeepTheNearestCap )
{
    auto fp = makePinFootprint( F_Cu, { { "1", PAD_ATTRIB::SMD, VECTOR2I( 300, 0 ) },
                                        { "2", PAD_ATTRIB::SMD, VECTOR2I( 100, 0 ) },
                                        { "3", PAD_ATTRIB::SMD, VECTOR2I( 200, 0 ) } } );

    const std::vector<VECTOR2I> got =
            PCB_GRID_HELPER::CollectPinTargets( { fp.get() }, F_Cu, VECTOR2I( 0, 0 ), 2 );

    BOOST_REQUIRE_EQUAL( got.size(), 2 );
    BOOST_CHECK_EQUAL( got[0], VECTOR2I( 100, 0 ) );
    BOOST_CHECK_EQUAL( got[1], VECTOR2I( 200, 0 ) );
}


BOOST_AUTO_TEST_CASE( MovingPadPointsAreBoxRelative )
{
    auto fp = makePinFootprint( F_Cu, { { "1", PAD_ATTRIB::SMD, VECTOR2I( 1005, 2010 ) },
                                        { "2", PAD_ATTRIB::SMD, VECTOR2I( 1035, 2010 ) },
                                        { "", PAD_ATTRIB::SMD, VECTOR2I( 1020, 2010 ) } } );

    const BOX2I box( VECTOR2I( 1000, 2000 ), VECTOR2I( 40, 20 ) );
    const std::vector<VECTOR2I> got = PCB_GRID_HELPER::MovingPadPoints( { fp.get() }, box );

    BOOST_REQUIRE_EQUAL( got.size(), 2 );
    BOOST_CHECK_EQUAL( got[0], VECTOR2I( 5, 10 ) );
    BOOST_CHECK_EQUAL( got[1], VECTOR2I( 35, 10 ) );
}


BOOST_AUTO_TEST_CASE( MovingPadPointsIgnoreFootprintOrder )
{
    auto a = makePinFootprint( F_Cu, { { "1", PAD_ATTRIB::SMD, VECTOR2I( 1030, 2010 ) } } );
    auto b = makePinFootprint( F_Cu, { { "1", PAD_ATTRIB::SMD, VECTOR2I( 1005, 2010 ) } } );

    const BOX2I box( VECTOR2I( 1000, 2000 ), VECTOR2I( 40, 20 ) );

    const std::vector<VECTOR2I> ab = PCB_GRID_HELPER::MovingPadPoints( { a.get(), b.get() }, box );
    const std::vector<VECTOR2I> ba = PCB_GRID_HELPER::MovingPadPoints( { b.get(), a.get() }, box );

    BOOST_REQUIRE_EQUAL( ab.size(), 2 );
    BOOST_CHECK( ab == ba );
    BOOST_CHECK_EQUAL( ab[0], VECTOR2I( 5, 10 ) );
    BOOST_CHECK_EQUAL( ab[1], VECTOR2I( 30, 10 ) );
}


BOOST_AUTO_TEST_SUITE_END()
