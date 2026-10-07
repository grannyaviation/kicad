/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
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

#include <fstream>
#include <memory>
#include <vector>

#include <boost/test/unit_test.hpp>

#include <wx/filename.h>

#include <qa_utils/wx_utils/unit_test_utils.h>
#include <pcbnew_utils/board_test_utils.h>

#include <api/api_handler_pcb.h>
#include <api/headless_pcb_context.h>
#include <api/board/board_commands.pb.h>
#include <api/common/envelope.pb.h>
#include <api/common/types/base_types.pb.h>

#include <board.h>
#include <connectivity/connectivity_data.h>
#include <footprint.h>
#include <netinfo.h>
#include <pcb_track.h>
#include <settings/settings_manager.h>
#include <zone.h>


namespace
{

/// issue5830 is a four-copper-zone board with stable, human-readable zone UUIDs.
const wxString F_CU_ZONE   = wxS( "00000000-0000-0000-0000-00005c07d704" );
const wxString B_CU_ZONE   = wxS( "00000000-0000-0000-0000-00005c07d701" );
const wxString IN1_CU_ZONE = wxS( "00000000-0000-0000-0000-00005c07d707" );
const wxString IN2_CU_ZONE = wxS( "00000000-0000-0000-0000-00005c07d70a" );


struct API_HANDLER_PCB_FIXTURE
{
    SETTINGS_MANAGER                      m_settingsManager;
    std::unique_ptr<BOARD>                m_board;
    std::shared_ptr<HEADLESS_PCB_CONTEXT> m_context;

    // The context takes ownership of the board; the returned raw pointer lets the test inspect
    // zone state after the handler runs.
    BOARD* loadBoard( const wxString& aRelPath )
    {
        KI_TEST::LoadBoard( m_settingsManager, aRelPath, m_board );

        BOARD* board = m_board.get();
        m_context = std::make_shared<HEADLESS_PCB_CONTEXT>( std::move( m_board ),
                                                            &m_settingsManager.Prj(), nullptr );
        return board;
    }

    kiapi::common::ApiRequest makeRefillRequest( BOARD* aBoard, const std::vector<wxString>& aZoneIds ) const
    {
        kiapi::board::commands::RefillZones command;
        command.mutable_board()->set_type( kiapi::common::types::DocumentType::DOCTYPE_PCB );
        command.mutable_board()->set_board_filename(
                wxFileName( aBoard->GetFileName() ).GetFullName().ToStdString() );

        for( const wxString& id : aZoneIds )
            command.add_zones()->set_value( id.ToStdString() );

        kiapi::common::ApiRequest request;
        request.mutable_header()->set_client_name( "kicad.qa" );
        BOOST_REQUIRE( request.mutable_message()->PackFrom( command ) );

        return request;
    }

    kiapi::common::ApiRequest makeFlipRequest( BOARD* aBoard, const std::vector<KIID>& aIds ) const
    {
        kiapi::board::commands::FlipItems command;
        command.mutable_board()->set_type( kiapi::common::types::DocumentType::DOCTYPE_PCB );
        command.mutable_board()->set_board_filename(
                wxFileName( aBoard->GetFileName() ).GetFullName().ToStdString() );

        for( const KIID& id : aIds )
            command.add_items()->set_value( id.AsStdString() );

        kiapi::common::ApiRequest request;
        request.mutable_header()->set_client_name( "kicad.qa" );
        BOOST_REQUIRE( request.mutable_message()->PackFrom( command ) );

        return request;
    }

    template <typename COMMAND>
    kiapi::common::ApiRequest makePathRequest( BOARD* aBoard, const wxString& aPath ) const
    {
        COMMAND command;
        command.mutable_board()->set_type( kiapi::common::types::DocumentType::DOCTYPE_PCB );
        command.mutable_board()->set_board_filename(
                wxFileName( aBoard->GetFileName() ).GetFullName().ToStdString() );
        command.set_path( aPath.ToStdString() );

        kiapi::common::ApiRequest request;
        request.mutable_header()->set_client_name( "kicad.qa" );
        BOOST_REQUIRE( request.mutable_message()->PackFrom( command ) );

        return request;
    }

    FOOTPRINT* frontFootprint( BOARD* aBoard ) const
    {
        for( FOOTPRINT* footprint : aBoard->Footprints() )
        {
            if( footprint->GetLayer() == F_Cu )
                return footprint;
        }

        return nullptr;
    }

    ZONE* zoneByUuid( BOARD* aBoard, const wxString& aUuid ) const
    {
        for( ZONE* zone : aBoard->Zones() )
        {
            if( zone->m_Uuid.AsString() == aUuid )
                return zone;
        }

        return nullptr;
    }

    void unfillAll( BOARD* aBoard ) const
    {
        // Start from a clean slate so a positive IsFilled() result can only come from this fill
        for( ZONE* zone : aBoard->Zones() )
        {
            zone->UnFill();
            BOOST_REQUIRE( !zone->IsFilled() );
        }
    }
};

} // namespace


BOOST_FIXTURE_TEST_SUITE( ApiHandlerPcb, API_HANDLER_PCB_FIXTURE )


BOOST_AUTO_TEST_CASE( RefillZonesSubset )
{
    BOARD* board = loadBoard( wxS( "issue5830" ) );

    unfillAll( board );

    API_HANDLER_PCB           handler( m_context );
    kiapi::common::ApiRequest request = makeRefillRequest( board, { F_CU_ZONE, IN1_CU_ZONE } );
    API_RESULT                result = handler.Handle( request );

    if( !result.has_value() )
    {
        BOOST_FAIL( "RefillZones returned status " << result.error().status() << ": "
                                                    << result.error().error_message() );
    }

    BOOST_CHECK_EQUAL( result->status().status(), kiapi::common::ApiStatusCode::AS_OK );

    ZONE* fCu   = zoneByUuid( board, F_CU_ZONE );
    ZONE* bCu   = zoneByUuid( board, B_CU_ZONE );
    ZONE* in1Cu = zoneByUuid( board, IN1_CU_ZONE );
    ZONE* in2Cu = zoneByUuid( board, IN2_CU_ZONE );

    BOOST_REQUIRE( fCu && bCu && in1Cu && in2Cu );

    // Exactly the requested zones must be filled; the others must be untouched.
    BOOST_CHECK( fCu->IsFilled() );
    BOOST_CHECK( in1Cu->IsFilled() );
    BOOST_CHECK( !bCu->IsFilled() );
    BOOST_CHECK( !in2Cu->IsFilled() );
}


BOOST_AUTO_TEST_CASE( RefillZonesSubsetRebuildsConnectivity )
{
    BOARD* board = loadBoard( wxS( "issue5830" ) );

    unfillAll( board );

    // Baseline ratsnest with every zone empty; the GND planes are unfilled so their pads still
    // ratsnest together.
    board->BuildConnectivity();
    const unsigned baseline = board->GetConnectivity()->GetUnconnectedCount( false );
    BOOST_REQUIRE_MESSAGE( baseline > 0, "expected an unconnected baseline with zones empty" );

    API_HANDLER_PCB           handler( m_context );
    kiapi::common::ApiRequest request = makeRefillRequest( board, { F_CU_ZONE, IN1_CU_ZONE } );
    API_RESULT                result = handler.Handle( request );

    if( !result.has_value() )
    {
        BOOST_FAIL( "RefillZones returned status " << result.error().status() << ": "
                                                    << result.error().error_message() );
    }

    const unsigned afterFill = board->GetConnectivity()->GetUnconnectedCount( false );

    // Filling the GND planes bridges GND pads that previously ratsnested, so the unconnected
    // count drops.  Push cleared the ratsnest, so if the handler skipped the connectivity
    // rebuild this would read zero instead of the reduced-but-nonzero count.
    BOOST_CHECK_MESSAGE( afterFill > 0, "connectivity was cleared, not rebuilt, after the fill" );
    BOOST_CHECK_MESSAGE( afterFill < baseline,
                         "filling the GND planes should reduce the unconnected count ("
                                 << afterFill << " vs baseline " << baseline << ")" );
}


BOOST_AUTO_TEST_CASE( RefillZonesAllHeadless )
{
    BOARD* board = loadBoard( wxS( "issue5830" ) );

    unfillAll( board );

    API_HANDLER_PCB           handler( m_context );
    kiapi::common::ApiRequest request = makeRefillRequest( board, {} );
    API_RESULT                result = handler.Handle( request );

    if( !result.has_value() )
    {
        BOOST_FAIL( "RefillZones returned status " << result.error().status() << ": "
                                                    << result.error().error_message() );
    }

    BOOST_CHECK_EQUAL( result->status().status(), kiapi::common::ApiStatusCode::AS_OK );

    // With no frame the empty-zones request must fill everything synchronously
    for( ZONE* zone : board->Zones() )
        BOOST_CHECK_MESSAGE( zone->IsFilled(), "zone " << zone->m_Uuid.AsStdString() << " not filled" );
}


BOOST_AUTO_TEST_CASE( RefillZonesUnknownIdRejected )
{
    BOARD* board = loadBoard( wxS( "issue5830" ) );

    API_HANDLER_PCB           handler( m_context );
    kiapi::common::ApiRequest request =
            makeRefillRequest( board, { wxS( "deadbeef-0000-0000-0000-000000000000" ) } );
    API_RESULT                result = handler.Handle( request );

    BOOST_REQUIRE( !result.has_value() );
    BOOST_CHECK_EQUAL( result.error().status(), kiapi::common::ApiStatusCode::AS_BAD_REQUEST );
}


BOOST_AUTO_TEST_CASE( FlipItemsFlipsFootprintInPlace )
{
    BOARD*     board = loadBoard( wxS( "issue5830" ) );
    FOOTPRINT* footprint = frontFootprint( board );
    BOOST_REQUIRE( footprint );
    const VECTOR2I position = footprint->GetPosition();

    API_HANDLER_PCB handler( m_context );
    kiapi::common::ApiRequest request = makeFlipRequest( board, { footprint->m_Uuid } );
    API_RESULT      result = handler.Handle( request );

    BOOST_REQUIRE_MESSAGE( result.has_value(),
                           ( result.has_value() ? std::string() : result.error().error_message() ) );
    BOOST_CHECK_EQUAL( footprint->GetLayer(), B_Cu );
    BOOST_CHECK( footprint->IsFlipped() );
    BOOST_CHECK_EQUAL( footprint->GetPosition(), position );
}


BOOST_AUTO_TEST_CASE( FlipItemsTwiceReturnsToFront )
{
    BOARD*     board = loadBoard( wxS( "issue5830" ) );
    FOOTPRINT* footprint = frontFootprint( board );
    BOOST_REQUIRE( footprint );
    const VECTOR2I position = footprint->GetPosition();

    API_HANDLER_PCB handler( m_context );
    kiapi::common::ApiRequest request = makeFlipRequest( board, { footprint->m_Uuid } );
    BOOST_REQUIRE( handler.Handle( request ).has_value() );
    BOOST_REQUIRE( handler.Handle( request ).has_value() );

    BOOST_CHECK_EQUAL( footprint->GetLayer(), F_Cu );
    BOOST_CHECK( !footprint->IsFlipped() );
    BOOST_CHECK_EQUAL( footprint->GetPosition(), position );
}


BOOST_AUTO_TEST_CASE( FlipItemsUnknownIdRejected )
{
    BOARD*     board = loadBoard( wxS( "issue5830" ) );
    FOOTPRINT* footprint = frontFootprint( board );
    BOOST_REQUIRE( footprint );

    API_HANDLER_PCB handler( m_context );
    kiapi::common::ApiRequest request = makeFlipRequest(
            board, { footprint->m_Uuid, KIID( wxS( "deadbeef-0000-0000-0000-000000000000" ) ) } );
    API_RESULT      result = handler.Handle( request );

    BOOST_REQUIRE( !result.has_value() );
    BOOST_CHECK_EQUAL( result.error().status(), kiapi::common::ApiStatusCode::AS_BAD_REQUEST );
    // All or nothing: the valid footprint in the same request stays where it was
    BOOST_CHECK_EQUAL( footprint->GetLayer(), F_Cu );
}


BOOST_AUTO_TEST_CASE( FlipItemsNonFootprintRejected )
{
    BOARD*     board = loadBoard( wxS( "issue5830" ) );
    FOOTPRINT* footprint = frontFootprint( board );
    BOOST_REQUIRE( footprint );
    BOOST_REQUIRE( !board->Tracks().empty() );
    PCB_TRACK*         track = board->Tracks().front();
    const PCB_LAYER_ID trackLayer = track->GetLayer();

    API_HANDLER_PCB handler( m_context );
    kiapi::common::ApiRequest request = makeFlipRequest( board, { footprint->m_Uuid, track->m_Uuid } );
    API_RESULT result = handler.Handle( request );

    BOOST_REQUIRE( !result.has_value() );
    BOOST_CHECK_EQUAL( result.error().status(), kiapi::common::ApiStatusCode::AS_BAD_REQUEST );
    BOOST_CHECK_EQUAL( footprint->GetLayer(), F_Cu );
    BOOST_CHECK_EQUAL( track->GetLayer(), trackLayer );
}


BOOST_AUTO_TEST_CASE( ExportSpecctraDsnWritesTheBoard )
{
    BOARD*   board = loadBoard( wxS( "issue5830" ) );
    wxString path = wxFileName::CreateTempFileName( wxS( "kicad_qa_dsn" ) );

    API_HANDLER_PCB handler( m_context );
    kiapi::common::ApiRequest request = makePathRequest<kiapi::board::commands::ExportSpecctraDsn>( board, path );
    API_RESULT      result = handler.Handle( request );

    BOOST_REQUIRE_MESSAGE( result.has_value(),
                           ( result.has_value() ? std::string() : result.error().error_message() ) );
    std::ifstream in( path.ToStdString() );
    std::string   text( ( std::istreambuf_iterator<char>( in ) ), std::istreambuf_iterator<char>() );
    wxRemoveFile( path );
    BOOST_CHECK( text.find( "(pcb " ) != std::string::npos );
    BOOST_CHECK( text.find( "(network" ) != std::string::npos );
}


BOOST_AUTO_TEST_CASE( ExportSpecctraDsnRelativePathRejected )
{
    BOARD* board = loadBoard( wxS( "issue5830" ) );

    API_HANDLER_PCB handler( m_context );
    kiapi::common::ApiRequest request = makePathRequest<kiapi::board::commands::ExportSpecctraDsn>( board, wxS( "relative.dsn" ) );
    API_RESULT      result = handler.Handle( request );

    BOOST_REQUIRE( !result.has_value() );
    BOOST_CHECK_EQUAL( result.error().status(), kiapi::common::ApiStatusCode::AS_BAD_REQUEST );
}


BOOST_AUTO_TEST_CASE( ImportSpecctraSessionReplacesUnlockedTracks )
{
    BOARD* board = loadBoard( wxS( "issue5830" ) );
    BOOST_REQUIRE( !board->Tracks().empty() );

    NETINFO_ITEM* net = nullptr;

    for( NETINFO_ITEM* candidate : board->GetNetInfo() )
    {
        if( candidate->GetNetCode() > 0 && !candidate->GetNetname().IsEmpty() )
        {
            net = candidate;
            break;
        }
    }

    BOOST_REQUIRE( net );

    // One 0.25 mm wide, 1 mm long wire at the origin (resolution um 10: one unit is 0.1 µm)
    wxString path = wxFileName::CreateTempFileName( wxS( "kicad_qa_ses" ) );
    {
        std::ofstream out( path.ToStdString() );
        out << "(session qa.ses\n"
               "  (base_design qa.dsn)\n"
               "  (routes\n"
               "    (resolution um 10)\n"
               "    (parser (host_cad \"KiCad's Pcbnew\") (host_version qa))\n"
               "    (library_out)\n"
               "    (network_out\n"
               "      (net \"" << net->GetNetname().ToStdString() << "\"\n"
               "        (wire (path F.Cu 2500 0 0 10000 0))\n"
               "      )\n"
               "    )\n"
               "  )\n"
               ")\n";
    }

    API_HANDLER_PCB handler( m_context );
    kiapi::common::ApiRequest request = makePathRequest<kiapi::board::commands::ImportSpecctraSession>( board, path );
    API_RESULT      result = handler.Handle( request );
    wxRemoveFile( path );

    BOOST_REQUIRE_MESSAGE( result.has_value(),
                           ( result.has_value() ? std::string() : result.error().error_message() ) );

    int ours = 0;

    for( PCB_TRACK* track : board->Tracks() )
    {
        if( track->GetNetname() == net->GetNetname() && track->GetWidth() == pcbIUScale.mmToIU( 0.25 )
            && std::abs( track->GetEnd().x - track->GetStart().x ) == pcbIUScale.mmToIU( 1.0 ) )
        {
            ++ours;
        }
        else
        {
            // Everything else the session did not create must be a locked survivor
            BOOST_CHECK( track->IsLocked() );
        }
    }

    BOOST_CHECK_EQUAL( ours, 1 );
}


BOOST_AUTO_TEST_CASE( ImportSpecctraSessionWithoutRoutesRejected )
{
    BOARD*       board = loadBoard( wxS( "issue5830" ) );
    const size_t tracks = board->Tracks().size();
    wxString     path = wxFileName::CreateTempFileName( wxS( "kicad_qa_ses" ) );
    {
        std::ofstream out( path.ToStdString() );
        out << "(session qa.ses (base_design qa.dsn))\n";
    }

    API_HANDLER_PCB handler( m_context );
    kiapi::common::ApiRequest request = makePathRequest<kiapi::board::commands::ImportSpecctraSession>( board, path );
    API_RESULT      result = handler.Handle( request );
    wxRemoveFile( path );

    BOOST_REQUIRE( !result.has_value() );
    BOOST_CHECK_EQUAL( result.error().status(), kiapi::common::ApiStatusCode::AS_BAD_REQUEST );
    BOOST_CHECK_EQUAL( board->Tracks().size(), tracks );
}


BOOST_AUTO_TEST_SUITE_END()
