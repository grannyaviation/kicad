/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2024 Jon Evans <jon@craftyjon.com>
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <boost/test/unit_test.hpp>
#include <import_export.h>
#include <qa_utils/api_test_utils.h>
#include <qa_utils/wx_utils/wx_assert.h>
#include <pcbnew_utils/board_test_utils.h>
#include <settings/settings_manager.h>

#include <api/board/board_types.pb.h>

#include <board.h>
#include <footprint.h>
#include <pcb_barcode.h>
#include <pcb_dimension.h>
#include <pcb_reference_image.h>
#include <pcb_shape.h>
#include <pcb_track.h>
#include <zone.h>


BOOST_AUTO_TEST_SUITE( ApiProto )

struct PROTO_TEST_FIXTURE
{
    PROTO_TEST_FIXTURE()
    { }

    SETTINGS_MANAGER       m_settingsManager;
    std::unique_ptr<BOARD> m_board;
};


BOOST_FIXTURE_TEST_CASE( BoardTypes, PROTO_TEST_FIXTURE )
{
    KI_TEST::LoadBoard( m_settingsManager, "api_kitchen_sink", m_board );

    int barcodeCount = 0;
    int referenceImageCount = 0;

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        switch( track->Type() )
        {
        case PCB_TRACE_T:
            testProtoFromKiCadObject<kiapi::board::types::Track>( track, m_board.get() );
            break;

        case PCB_ARC_T:
            testProtoFromKiCadObject<kiapi::board::types::Arc>( static_cast<PCB_ARC*>( track ),
                                                                m_board.get() );
            break;

        case PCB_VIA_T:
            // Vias are not strict-checked at the moment because m_zoneLayerOverrides is not
            // currently exposed to the API
            // TODO(JE) enable strict when fixed
            testProtoFromKiCadObject<kiapi::board::types::Via>( static_cast<PCB_VIA*>( track ),
                                                                m_board.get(), false );
            break;

        default:
            wxFAIL;
        }
    }

    for( FOOTPRINT* footprint : m_board->Footprints() )
        testProtoFromKiCadObject<kiapi::board::types::FootprintInstance>( footprint, m_board.get() );

    for( ZONE* zone : m_board->Zones() )
        testProtoFromKiCadObject<kiapi::board::types::Zone>( zone, m_board.get() );

    for( BOARD_ITEM* item : m_board->Drawings() )
    {
        switch( item->Type() )
        {
        case PCB_DIM_ALIGNED_T:
            testProtoFromKiCadObject<kiapi::board::types::Dimension>(
                    static_cast<PCB_DIM_ALIGNED*>( item ), m_board.get() );
            break;

        case PCB_DIM_ORTHOGONAL_T:
            testProtoFromKiCadObject<kiapi::board::types::Dimension>(
                    static_cast<PCB_DIM_ORTHOGONAL*>( item ), m_board.get() );
            break;

        case PCB_DIM_CENTER_T:
            testProtoFromKiCadObject<kiapi::board::types::Dimension>(
                    static_cast<PCB_DIM_CENTER*>( item ), m_board.get() );
            break;

        case PCB_DIM_LEADER_T:
            testProtoFromKiCadObject<kiapi::board::types::Dimension>(
                    static_cast<PCB_DIM_LEADER*>( item ), m_board.get() );
            break;

        case PCB_DIM_RADIAL_T:
            testProtoFromKiCadObject<kiapi::board::types::Dimension>(
                    static_cast<PCB_DIM_RADIAL*>( item ), m_board.get() );
            break;

        case PCB_BARCODE_T:
            testProtoFromKiCadObject<kiapi::board::types::Barcode>(
                    static_cast<PCB_BARCODE*>( item ), m_board.get() );
            ++barcodeCount;
            break;

        case PCB_REFERENCE_IMAGE_T:
            testProtoFromKiCadObject<kiapi::board::types::ReferenceImage>(
                    static_cast<PCB_REFERENCE_IMAGE*>( item ), m_board.get() );
            ++referenceImageCount;
            break;

        default: break;
        }
        // TODO(JE) Shapes

        // TODO(JE) Text
    }

    BOOST_CHECK_GT( barcodeCount, 0 );
    BOOST_CHECK_GT( referenceImageCount, 0 );
}


BOOST_FIXTURE_TEST_CASE( Padstacks, PROTO_TEST_FIXTURE )
{
    KI_TEST::LoadBoard( m_settingsManager, "padstacks", m_board );

    for( PCB_TRACK* track : m_board->Tracks() )
    {
        switch( track->Type() )
        {
        case PCB_VIA_T:
            // Vias are not strict-checked at the moment because m_zoneLayerOverrides is not
            // currently exposed to the API
            // TODO(JE) enable strict when fixed
            testProtoFromKiCadObject<kiapi::board::types::Via>( static_cast<PCB_VIA*>( track ),
                                                                m_board.get(), false );
            break;

        default:
            wxFAIL;
        }
    }

    for( FOOTPRINT* footprint : m_board->Footprints() )
        testProtoFromKiCadObject<kiapi::board::types::FootprintInstance>( footprint, m_board.get() );
}

/**
 * Round-trip a copper-thieving zone through the protobuf API.  The shared
 * testProtoFromKiCadObject helper relies on ZONE::operator==, which by
 * existing precedent does not compare fill-mode or hatch/thieving fields,
 * so we hand-check every thieving field plus the netless invariant.
 */
BOOST_FIXTURE_TEST_CASE( CopperThievingZoneRoundTrip, PROTO_TEST_FIXTURE )
{
    m_board = std::make_unique<BOARD>();

    ZONE* zone = new ZONE( m_board.get() );
    zone->SetLayer( F_Cu );
    zone->AppendCorner( VECTOR2I( 0, 0 ), -1 );
    zone->AppendCorner( VECTOR2I( pcbIUScale.mmToIU( 5 ), 0 ), -1 );
    zone->AppendCorner( VECTOR2I( pcbIUScale.mmToIU( 5 ), pcbIUScale.mmToIU( 5 ) ), -1 );
    zone->AppendCorner( VECTOR2I( 0, pcbIUScale.mmToIU( 5 ) ), -1 );
    zone->SetFillMode( ZONE_FILL_MODE::COPPER_THIEVING );

    THIEVING_SETTINGS thieving;
    thieving.pattern      = THIEVING_PATTERN::SQUARES;
    thieving.element_size = pcbIUScale.mmToIU( 0.75 );
    thieving.gap        = pcbIUScale.mmToIU( 2.0 );
    thieving.line_width   = pcbIUScale.mmToIU( 0.4 );
    thieving.stagger      = true;
    thieving.orientation     = EDA_ANGLE( 15.0, DEGREES_T );
    zone->SetThievingSettings( thieving );

    m_board->Add( zone );

    google::protobuf::Any any;
    BOOST_REQUIRE_NO_THROW( zone->Serialize( any ) );

    kiapi::board::types::Zone proto;
    BOOST_REQUIRE( any.UnpackTo( &proto ) );
    BOOST_REQUIRE( proto.has_copper_settings() );
    BOOST_REQUIRE( proto.copper_settings().has_thieving_settings() );

    std::unique_ptr<ZONE> roundTripped = std::make_unique<ZONE>( m_board.get() );
    BOOST_REQUIRE( roundTripped->Deserialize( any ) );

    BOOST_CHECK( roundTripped->GetFillMode() == ZONE_FILL_MODE::COPPER_THIEVING );
    // A board with no netinfo list returns GetNetCode() == -1 for an unbound zone.
    // The invariant we want is "no real net assigned"; netcode > 0 would mean a leak.
    BOOST_CHECK_LE( roundTripped->GetNetCode(), 0 );

    const THIEVING_SETTINGS& loaded = roundTripped->GetThievingSettings();
    BOOST_CHECK( loaded.pattern == THIEVING_PATTERN::SQUARES );
    BOOST_CHECK_EQUAL( loaded.element_size, thieving.element_size );
    BOOST_CHECK_EQUAL( loaded.gap, thieving.gap );
    BOOST_CHECK_EQUAL( loaded.line_width, thieving.line_width );
    BOOST_CHECK_EQUAL( loaded.stagger, true );
    BOOST_CHECK( loaded.orientation == EDA_ANGLE( 15.0, DEGREES_T ) );
}


/**
 * An API client moves a footprint by sending it back with a new position and every child
 * shifted by the same offset in board coordinates, because FOOTPRINT::Deserialize re-creates
 * the children from the message.  Footprint children keep their geometry in the library
 * frame, so a zone or an arc that is not packed and unpacked in board coordinates comes back
 * detached from the footprint (zone) or collapsed to the library origin (arc).
 */
BOOST_FIXTURE_TEST_CASE( FootprintMovedThroughApiKeepsZonesAndArcs, PROTO_TEST_FIXTURE )
{
    auto mm = []( double aX, double aY )
    {
        return VECTOR2I( pcbIUScale.mmToIU( aX ), pcbIUScale.mmToIU( aY ) );
    };

    m_board = std::make_unique<BOARD>();

    FOOTPRINT* footprint = new FOOTPRINT( m_board.get() );
    m_board->Add( footprint );

    // Built at the identity transform, so these are library coordinates.
    ZONE* zone = new ZONE( footprint );
    zone->SetIsRuleArea( true );
    zone->SetLayer( F_Cu );
    zone->AppendCorner( mm( -1, -1 ), -1 );
    zone->AppendCorner( mm( 2, -1 ), -1 );
    zone->AppendCorner( mm( 2, 3 ), -1 );
    zone->AppendCorner( mm( -1, 3 ), -1 );
    footprint->Add( zone );

    PCB_SHAPE* arc = new PCB_SHAPE( footprint, SHAPE_T::ARC );
    arc->SetLayer( F_SilkS );
    arc->SetArcGeometry( mm( -3, 0 ), mm( -2.12132, -2.12132 ), mm( 0, -3 ) );
    footprint->Add( arc );

    footprint->SetPosition( mm( 10, 20 ) );
    footprint->SetOrientation( ANGLE_90 );

    const SHAPE_POLY_SET libOutline = *zone->Outline();
    const VECTOR2I       libStart = arc->GetLibraryStart();
    const VECTOR2I       libMid = arc->GetLibraryArcMid();
    const VECTOR2I       libEnd = arc->GetLibraryEnd();

    google::protobuf::Any any;
    footprint->Serialize( any );

    kiapi::board::types::FootprintInstance msg;
    BOOST_REQUIRE( any.UnpackTo( &msg ) );

    const VECTOR2I offset = mm( -6, 17 );

    auto shift = [&]( kiapi::common::types::Vector2* aPoint )
    {
        aPoint->set_x_nm( aPoint->x_nm() + offset.x );
        aPoint->set_y_nm( aPoint->y_nm() + offset.y );
    };

    shift( msg.mutable_position() );

    for( google::protobuf::Any& item : *msg.mutable_definition()->mutable_items() )
    {
        kiapi::board::types::Zone              zoneMsg;
        kiapi::board::types::BoardGraphicShape shapeMsg;

        if( item.UnpackTo( &zoneMsg ) )
        {
            for( auto& polygon : *zoneMsg.mutable_outline()->mutable_polygons() )
            {
                for( auto& node : *polygon.mutable_outline()->mutable_nodes() )
                    shift( node.mutable_point() );
            }

            item.PackFrom( zoneMsg );
        }
        else if( item.UnpackTo( &shapeMsg ) && shapeMsg.shape().has_arc() )
        {
            auto* arcMsg = shapeMsg.mutable_shape()->mutable_arc();
            shift( arcMsg->mutable_start() );
            shift( arcMsg->mutable_mid() );
            shift( arcMsg->mutable_end() );
            item.PackFrom( shapeMsg );
        }
    }

    any.PackFrom( msg );

    FOOTPRINT moved( m_board.get() );
    BOOST_REQUIRE( moved.Deserialize( any ) );
    BOOST_CHECK_EQUAL( moved.GetPosition(), mm( 10, 20 ) + offset );

    BOOST_REQUIRE_EQUAL( moved.Zones().size(), 1 );
    const SHAPE_POLY_SET* movedOutline = moved.Zones().front()->Outline();
    BOOST_REQUIRE_EQUAL( movedOutline->TotalVertices(), libOutline.TotalVertices() );

    for( int i = 0; i < libOutline.TotalVertices(); ++i )
        BOOST_CHECK_EQUAL( movedOutline->CVertex( i ), libOutline.CVertex( i ) );

    const PCB_SHAPE* movedArc = nullptr;

    for( BOARD_ITEM* item : moved.GraphicalItems() )
    {
        if( item->Type() == PCB_SHAPE_T && static_cast<PCB_SHAPE*>( item )->GetShape() == SHAPE_T::ARC )
            movedArc = static_cast<PCB_SHAPE*>( item );
    }

    BOOST_REQUIRE( movedArc );
    BOOST_CHECK_EQUAL( movedArc->GetLibraryStart(), libStart );
    BOOST_CHECK_LE( ( movedArc->GetLibraryArcMid() - libMid ).EuclideanNorm(), 2 );
    BOOST_CHECK_EQUAL( movedArc->GetLibraryEnd(), libEnd );
}


BOOST_AUTO_TEST_SUITE_END()
