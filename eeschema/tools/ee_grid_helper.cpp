/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2014 CERN
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 * @author Tomasz Wlostowski <tomasz.wlostowski@cern.ch>
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

#include <algorithm>
#include <functional>
#include <tuple>
#include <macros.h>
#include <drawing_sheet/ds_draw_item.h>
#include <drawing_sheet/ds_proxy_view_item.h>
#include <gal/graphics_abstraction_layer.h>
#include <sch_draw_panel.h>
#include <sch_group.h>
#include <sch_item.h>
#include <sch_line.h>
#include <sch_pin.h>
#include <sch_shape.h>
#include <sch_sheet.h>
#include <sch_sheet_pin.h>
#include <sch_symbol.h>
#include <sch_table.h>
#include <sch_tablecell.h>
#include <sch_painter.h>
#include <sch_view.h>
#include <trace_helpers.h>
#include <wx/log.h>
#include <tool/align_geom.h>
#include <tool/tool_manager.h>
#include <sch_tool_base.h>
#include <settings/app_settings.h>
#include <symbol_editor/symbol_edit_frame.h>
#include <trigo.h>
#include <view/view.h>
#include "ee_grid_helper.h"

EE_GRID_HELPER::EE_GRID_HELPER() :
        GRID_HELPER( schIUScale )
{
}


EE_GRID_HELPER::EE_GRID_HELPER( TOOL_MANAGER* aToolMgr ) :
        GRID_HELPER( aToolMgr, LAYER_SCHEMATIC_ANCHOR, schIUScale )
{
    if( !m_toolMgr )
        return;

    KIGFX::VIEW* view = m_toolMgr->GetView();

    m_viewAxis.SetSize( 20000 );
    m_viewAxis.SetStyle( KIGFX::ORIGIN_VIEWITEM::CROSS );
    m_viewAxis.SetColor( COLOR4D( 0.0, 0.1, 0.4, 0.8 ) );
    m_viewAxis.SetDrawAtZero( true );
    view->Add( &m_viewAxis );
    view->SetVisible( &m_viewAxis, false );

    m_viewSnapPoint.SetStyle( KIGFX::ORIGIN_VIEWITEM::CIRCLE_CROSS );
    m_viewSnapPoint.SetColor( COLOR4D( 0.0, 0.1, 0.4, 1.0 ) );
    m_viewSnapPoint.SetDrawAtZero( true );
    view->Add( &m_viewSnapPoint );
    view->SetVisible( &m_viewSnapPoint, false );
}


EE_GRID_HELPER::~EE_GRID_HELPER()
{
    if( !m_toolMgr )
        return;

    KIGFX::VIEW* view = m_toolMgr->GetView();

    view->Remove( &m_viewAxis );
    view->Remove( &m_viewSnapPoint );
}


VECTOR2I EE_GRID_HELPER::BestDragOrigin( const VECTOR2I& aMousePos, GRID_HELPER_GRIDS aGrid,
                                         const SCH_SELECTION& aItems )
{
    clearAnchors();

    // If we're working with any connectable objects, skip non-connectable objects
    // since they are often off-grid, e.g. text anchors
    bool hasConnectables = false;

    for( EDA_ITEM* item : aItems )
    {
        GRID_HELPER_GRIDS grid = GetItemGrid( static_cast<SCH_ITEM*>( item ) );
        if( grid == GRID_CONNECTABLE || grid == GRID_WIRES )
        {
            hasConnectables = true;
            break;
        }
    }

    for( EDA_ITEM* item : aItems )
        computeAnchors( static_cast<SCH_ITEM*>( item ), aMousePos, true, !hasConnectables );

    double worldScale = m_toolMgr->GetView()->GetGAL()->GetWorldScale();
    double lineSnapMinCornerDistance = 50.0 / worldScale;

    ANCHOR* nearestOutline = nearestAnchor( aMousePos, OUTLINE, aGrid );
    ANCHOR* nearestCorner = nearestAnchor( aMousePos, CORNER, aGrid );
    ANCHOR* nearestOrigin = nearestAnchor( aMousePos, ORIGIN, aGrid );
    ANCHOR* best = nullptr;
    double minDist = std::numeric_limits<double>::max();

    if( nearestOrigin )
    {
        minDist = nearestOrigin->Distance( aMousePos );
        best = nearestOrigin;
    }

    if( nearestCorner )
    {
        double dist = nearestCorner->Distance( aMousePos );

        if( dist < minDist )
        {
            minDist = dist;
            best = nearestCorner;
        }
    }

    if( nearestOutline )
    {
        double dist = nearestOutline->Distance( aMousePos );

        if( minDist > lineSnapMinCornerDistance && dist < minDist )
            best = nearestOutline;
    }

    return best ? best->pos : aMousePos;
}


VECTOR2I EE_GRID_HELPER::BestSnapAnchor( const VECTOR2I& aOrigin, GRID_HELPER_GRIDS aGrid,
                                         SCH_ITEM* aSkip )
{
    SCH_SELECTION skipItems;
    skipItems.Add( aSkip );

    return BestSnapAnchor( aOrigin, aGrid, skipItems );
}


VECTOR2I EE_GRID_HELPER::BestSnapAnchor( const VECTOR2I& aOrigin, GRID_HELPER_GRIDS aGrid,
                                         const SCH_SELECTION& aSkip )
{
    constexpr int snapRange = SNAP_RANGE * schIUScale.IU_PER_MILS;

    VECTOR2I pt = aOrigin;
    VECTOR2I snapDist( snapRange, snapRange );
    bool     gridChecked = false;
    bool     snappedToAnchor = false;

    BOX2I    bb( VECTOR2I( aOrigin.x - snapRange / 2, aOrigin.y - snapRange / 2 ),
                 VECTOR2I( snapRange, snapRange ) );

    clearAnchors();
    m_snapItem = std::nullopt;

    // Any exit other than the guide path below means "no guides"; clearing here covers
    // the early returns too.
    clearAlignmentGuides();

    for( SCH_ITEM* item : queryVisible( bb, aSkip ) )
        computeAnchors( item, aOrigin );

    ANCHOR*  nearest = nearestAnchor( aOrigin, SNAPPABLE, aGrid );
    VECTOR2I nearestGrid = Align( aOrigin, aGrid );

    if( KIGFX::ANCHOR_DEBUG* ad = enableAndGetAnchorDebug(); ad )
    {
        ad->ClearAnchors();
        for( const ANCHOR& a : m_anchors )
            ad->AddAnchor( a.pos );

        ad->SetNearest( nearest ? OPT_VECTOR2I( nearest->pos ) : std::nullopt );
        m_toolMgr->GetView()->Update( ad, KIGFX::GEOMETRY );
    }

    showConstructionGeometry( m_enableSnap );

    SNAP_LINE_MANAGER& snapLineManager = getSnapManager().GetSnapLineManager();
    const VECTOR2D     gridSize = GetGridSize( aGrid );

    // The grid constraint only means something when the cursor is grid-snapped; with grid
    // snapping off the base below is the raw cursor and the moving box is already off-grid, so
    // requiring grid-multiple offsets would protect nothing -- and would violate FindSnap's
    // precondition that aMoving is grid-aligned -- while simply stopping guides from ever firing.
    //
    // Graphics are exempt from the whole-grid-step rule.  That rule exists so pins land on the
    // wire grid; a bitmap has no pins and a notes line connects to nothing, and at 100 mil the
    // nearest legal position is up to 1.27 mm from a title-block cell centre -- in a 3 mm row,
    // hard against an edge.  Keyed on the same graphics-only test that picked the box rule, so a
    // selection containing anything connectable stays strict.
    std::optional<VECTOR2I> gridStep;

    if( canUseGrid() && !m_graphicsMode )
        gridStep = KiROUND( gridSize );

    // At least +/-2 grid steps, whatever the grid.  Reusing snapRange alone would make the
    // feature unreachable on a 100 mil grid, since the engine only accepts whole-step offsets
    // and 55 mil is less than one step.
    const int guideRange = std::max( snapRange,
                                     2 * KiROUND( std::max( gridSize.x, gridSize.y ) ) );

    // The base is what this function returns with neither a guide nor an anchor snap, i.e. what
    // the tail below leaves in pt: nearestGrid when the grid is usable, the raw cursor otherwise.
    // It must be grid-aligned whenever gridStep is passed (FindSnap's precondition), and it must
    // be the counterpart of the move tool's OriginalCursor, which is its *snapped* cursor
    // (prevPos).  Extrapolating from the raw cursor while the grid is on would pair a snapped
    // origin with an unsnapped current point and land the selection up to half a grid step off
    // grid, which the whole-multiple offset cannot undo.
    //
    // Computed here rather than at the return below because a symbol drag ranks the guide above
    // anchor snapping, and that decision has to be made before the anchor block runs.  Queried
    // exactly once per call either way -- this is the mouse-motion path.
    const std::optional<GUIDE_SNAP> alignSnap =
            computeAlignmentGuideSnap( canUseGrid() ? nearestGrid : aOrigin, guideRange, gridStep );

    // Dragging whole symbols: the guide wins over pins and wire ends.  Anything else in the
    // selection (a wire end, a label) keeps anchor > guide, or dropping a wire on a pin breaks.
    // Set by SCH_MOVE_TOOL, which is where the selection is known.
    const bool preferGuides = alignSnap && m_moveContext && m_moveContext->PreferGuides;

    std::optional<VECTOR2I> guideSnap;

    if( m_enableSnapLine )
        guideSnap = SnapToConstructionLines( aOrigin, nearestGrid, gridSize, snapRange );

    if( m_enableSnap && !preferGuides && nearest
        && nearest->Distance( aOrigin ) < snapDist.EuclideanNorm() )
    {

        if( canUseGrid() && ( nearestGrid - aOrigin ).EuclideanNorm() < snapDist.EuclideanNorm() )
        {
            pt = nearestGrid;
            snapDist.x = std::abs( nearestGrid.x - aOrigin.x );
            snapDist.y = std::abs( nearestGrid.y - aOrigin.y );
            gridChecked = true;
        }
        else
        {
            pt = nearest->pos;
            snapDist.x = std::abs( nearest->pos.x - aOrigin.x );
            snapDist.y = std::abs( nearest->pos.y - aOrigin.y );
            snappedToAnchor = true;
            gridChecked = true;
        }
    }

    if( guideSnap && m_skipPoint != *guideSnap )
    {
        snapLineManager.SetSnapLineEnd( *guideSnap );
        m_toolMgr->GetView()->SetVisible( &m_viewSnapPoint, false );
        m_snapItem = std::nullopt;
        return *guideSnap;
    }

    if( snappedToAnchor )
    {
        m_snapItem = *nearest;
        m_viewSnapPoint.SetPosition( pt );

        snapLineManager.SetSnapLineOrigin( pt );
        snapLineManager.SetSnapLineEnd( std::nullopt );

        if( m_toolMgr->GetView()->IsVisible( &m_viewSnapPoint ) )
            m_toolMgr->GetView()->Update( &m_viewSnapPoint, KIGFX::GEOMETRY );
        else
            m_toolMgr->GetView()->SetVisible( &m_viewSnapPoint, true );

        return pt;
    }

    m_snapItem = std::nullopt;

    if( canUseGrid() && !gridChecked )
        pt = nearestGrid;

    snapLineManager.SetSnapLineEnd( std::nullopt );
    m_toolMgr->GetView()->SetVisible( &m_viewSnapPoint, false );

    // Smart alignment guides sit above the plain grid return, and -- unless preferGuides sent
    // the anchor block packing above -- below every anchor snap.  Painted only here, after the
    // teardown above, so a guide return leaves the canvas in the same clean state the plain grid
    // return does: no stale snap marker, no stale snap line, no stale m_snapItem.
    if( alignSnap )
    {
        showAlignmentGuides( *alignSnap );
        return alignSnap->Position;
    }

    return pt;
}


VECTOR2D EE_GRID_HELPER::GetGridSize( GRID_HELPER_GRIDS aGrid ) const
{
    const GRID_SETTINGS& grid = m_toolMgr->GetSettings()->m_Window.grid;
    int                  idx = -1;

    VECTOR2D g = m_toolMgr->GetView()->GetGAL()->GetGridSize();

    if( !grid.overrides_enabled )
        return g;

    switch( aGrid )
    {
    case GRID_CONNECTABLE:
        if( grid.override_connected )
            idx = grid.override_connected_idx;

        break;

    case GRID_WIRES:
        if( grid.override_wires )
            idx = grid.override_wires_idx;

        break;

    case GRID_TEXT:
        if( grid.override_text )
            idx = grid.override_text_idx;

        break;

    case GRID_GRAPHICS:
        if( grid.override_graphics )
            idx = grid.override_graphics_idx;

        break;

    default:
        break;
    }

    if( idx >= 0 && idx < (int) grid.grids.size() )
        g = grid.grids[idx].ToDouble( schIUScale );

    return g;
}


SCH_ITEM* EE_GRID_HELPER::GetSnapped() const
{
    if( !m_snapItem )
        return nullptr;

    if( m_snapItem->items.empty() )
        return nullptr;

    return static_cast<SCH_ITEM*>( m_snapItem->items[0] );
}


bool EE_GRID_HELPER::IsOffGrid( const EDA_ITEM* aItem, const VECTOR2I& aGrid,
                                const VECTOR2I& aOrigin )
{
    if( aGrid.x <= 0 || aGrid.y <= 0 )
        return false;

    std::vector<VECTOR2I> points;

    // A library pin has no GetConnectionPoints() of its own -- SCH_PIN only becomes connectable
    // through a parent symbol -- yet in the symbol editor it is the one thing that has to be on
    // grid, because every schematic that ever uses the part inherits the position.
    if( aItem->Type() == SCH_PIN_T )
    {
        points.push_back( static_cast<const SCH_PIN*>( aItem )->GetPosition() );
    }
    else if( const SCH_ITEM* item = dynamic_cast<const SCH_ITEM*>( aItem );
             item && item->IsConnectable() )
    {
        // Gated on IsConnectable() because SCH_LINE hands back its endpoints whatever layer it
        // is on, and a notes line landing between grid points connects to nothing and breaks
        // nothing.
        points = item->GetConnectionPoints();
    }

    return std::any_of( points.begin(), points.end(),
                        [&]( const VECTOR2I& aPt )
                        {
                            return ( aPt.x - aOrigin.x ) % aGrid.x != 0
                                   || ( aPt.y - aOrigin.y ) % aGrid.y != 0;
                        } );
}


void EE_GRID_HELPER::ShowOffGridWarnings( const SELECTION& aSelection, GRID_HELPER_GRIDS aGrid )
{
    const VECTOR2D gridSize = GetGridSize( aGrid );
    const VECTOR2I grid( KiROUND( gridSize.x ), KiROUND( gridSize.y ) );
    const VECTOR2I origin = GetOrigin();

    std::vector<VECTOR2I> markers;

    for( const EDA_ITEM* item : aSelection )
    {
        if( IsOffGrid( item, grid, origin ) )
        {
            // Top-right of the whole item, fields and all: outside the body, where it cannot be
            // mistaken for part of the symbol, and clear of the guide lines that run along the
            // body edges.
            const BOX2I box = item->GetBoundingBox();
            markers.emplace_back( box.GetRight(), box.GetTop() );
        }
    }

    SetOffGridWarnings( std::move( markers ) );
}


VECTOR2I EE_GRID_HELPER::AlignPointToGuides( const VECTOR2I&      aPoint,
                                             const SCH_SELECTION* aCollectSkip )
{
    // A zero-size box at the handle: its min, max and center all collapse onto the point, so
    // every alignment candidate the engine builds reduces to "line this corner up with a
    // neighbour edge" -- which is the whole of what a resize wants.  Re-set every motion
    // rather than once, since the handle is the cursor and the two never drift apart.
    SetMoveContext( BOX2I( aPoint, VECTOR2I( 0, 0 ) ), aPoint, true );

    // Must follow SetMoveContext(): the sweep bails without a move context, and sorts
    // neighbours by distance from it.
    if( aCollectSkip )
        CollectAlignmentNeighbors( *aCollectSkip );

    const VECTOR2D gridSize = GetGridSize( GRID_HELPER_GRIDS::GRID_GRAPHICS );

    // Same rules as a symbol drag: offsets must be whole grid steps or a resized sheet drags
    // its pins off grid, and the reach has to be at least a couple of steps to be usable on a
    // 100 mil grid.  aPoint is already grid-aligned by the caller, which FindSnap requires.
    // A graphic has no pins to drag off grid, so it is exempt; see BestSnapAnchor().
    std::optional<VECTOR2I> gridStep;

    if( canUseGrid() && !m_graphicsMode )
        gridStep = KiROUND( gridSize );

    const int range = 2 * KiROUND( std::max( gridSize.x, gridSize.y ) );

    if( std::optional<GUIDE_SNAP> snap = computeAlignmentGuideSnap( aPoint, range, gridStep ) )
    {
        showAlignmentGuides( *snap );
        return snap->Position;
    }

    clearAlignmentGuides();
    return aPoint;
}


std::optional<BOX2I> EE_GRID_HELPER::GetAlignmentBox( const EDA_ITEM* aItem )
{
    switch( aItem->Type() )
    {
    case SCH_SYMBOL_T:
    {
        const SCH_SYMBOL* symbol = static_cast<const SCH_SYMBOL*>( aItem );

        // Power ports are SCH_SYMBOLs too, and a sheet usually has many more of them than
        // components.  Aligning a chip to a GND flag is never what the user meant.
        if( symbol->IsPower() )
            return std::nullopt;

        // Body box only: field text is not what anyone aligns to, and pins stick out by
        // different amounts on either side of the same part.
        const BOX2I box = symbol->GetBodyBoundingBox();

        // GetBodyBoundingBox() swallows a boost::bad_pointer and returns a default-constructed
        // box.  The engine would take that as a real point box at (0, 0) and pull symbols to it.
        if( !box.IsValid() )
            return std::nullopt;

        return box;
    }

    case SCH_SHEET_T:
    {
        const SCH_SHEET* sheet = static_cast<const SCH_SHEET*>( aItem );

        // The drawn rectangle, deliberately not GetBodyBoundingBox(): that inflates by half the
        // border pen width, and the guide engine only accepts offsets that are a whole number of
        // grid steps.  Two sheets with different border widths would differ by half that
        // difference -- never a grid multiple -- so every sheet-to-sheet guide would be rejected
        // and the feature would look dead.  GetBoundingBox() is worse still: it adds the sheet
        // name above and the file name below, so guides would sit on invisible text.
        return BOX2I( sheet->GetPosition(), sheet->GetSize() );
    }

    default:
        return std::nullopt;
    }
}


/**
 * The nominal outline of a shape, as authored.
 *
 * EDA_SHAPE::getBoundingBox() ends with Inflate( GetWidth() / 2 ), so the box is half a stroke
 * wider than the shape on every side.  Half a stroke is not a whole grid step, so a shape
 * measured that way can never align on grid to anything drawn with a different width.
 *
 * Shared by the symbol and graphic rules deliberately: two copies of this reasoning would drift.
 */
static std::optional<BOX2I> shapeAlignmentBox( const SCH_SHAPE* aShape )
{
    BOX2I box = aShape->GetBoundingBox();

    // An empty POLY, or a BEZIER whose curve points have not been rebuilt, leaves
    // getBoundingBox() with a default-constructed box.  The engine would read that as a real
    // point box at the origin and pull the selection towards (0, 0).
    //
    // Deliberately not a size test: a straight polyline or segment has zero extent on one axis,
    // and dropping those would lose every diode bar and ground symbol in the library.
    if( !box.IsValid() )
        return std::nullopt;

    box.Inflate( -( std::max( 0, aShape->GetWidth() ) / 2 ) );

    return box;
}


std::optional<BOX2I> EE_GRID_HELPER::GetSymbolAlignmentBox( const EDA_ITEM* aItem )
{
    switch( aItem->Type() )
    {
    case SCH_PIN_T:
    {
        // A point, not a rectangle.  Zero size collapses min, max and centre onto the
        // connection point, so every alignment candidate the engine builds reduces to "line
        // this pin up with that one" -- and a column of pins yields clusters whose gaps are the
        // pin pitch, which is what makes equal-pitch snapping work with no extra machinery.
        const VECTOR2I pos = static_cast<const SCH_PIN*>( aItem )->GetPosition();

        return BOX2I( pos, VECTOR2I( 0, 0 ) );
    }

    case SCH_SHAPE_T:
        return shapeAlignmentBox( static_cast<const SCH_SHAPE*>( aItem ) );

    default:
        // Text, text boxes and fields: extents depend on font metrics and on whether a field is
        // visible, and the width of a pin name is not something anyone aligns to.
        return std::nullopt;
    }
}


std::optional<BOX2I> EE_GRID_HELPER::GetGraphicAlignmentBox( const EDA_ITEM* aItem )
{
    switch( aItem->Type() )
    {
    case SCH_BITMAP_T:
    {
        const BOX2I box = aItem->GetBoundingBox();

        // Not IsValid(): REFERENCE_IMAGE::GetBoundingBox() builds the box with BOX2I::ByCenter(),
        // which marks it initialised whatever the size, so a bitmap with no image loaded comes
        // back as a *valid* zero-size box at its position rather than an invalid one.  A loaded
        // image always has both dimensions positive, so the size is the honest test -- and an
        // item with nothing drawn must never become an alignment target.
        if( box.GetWidth() <= 0 || box.GetHeight() <= 0 )
            return std::nullopt;

        return box;
    }

    case SCH_LINE_T:
    {
        const SCH_LINE* line = static_cast<const SCH_LINE*>( aItem );

        // A wire or bus keeps the body rule and its grid-legal snapping.  Only a notes line is
        // a separator.
        if( line->IsConnectable() )
            return std::nullopt;

        // Merge rather than construct from the pair: a line drawn right-to-left would otherwise
        // produce a box with negative size, and every guide against it would be wrong.  A
        // horizontal separator is legitimately zero-height, exactly as flat polylines are in the
        // symbol rule.
        BOX2I box( line->GetStartPoint(), VECTOR2I( 0, 0 ) );
        box.Merge( line->GetEndPoint() );

        return box;
    }

    case SCH_SHAPE_T:
        return shapeAlignmentBox( static_cast<const SCH_SHAPE*>( aItem ) );

    default:
        return std::nullopt;
    }
}


std::optional<BOX2I> EE_GRID_HELPER::GetSheetPinAlignmentBox( const EDA_ITEM* aItem )
{
    if( aItem->Type() != SCH_SHEET_PIN_T )
        return std::nullopt;

    // A point, for the reason a symbol pin is one: what the user lines up is where the wire
    // attaches, and zero size collapses min, max and centre onto it so pin-to-pin alignment and
    // equal pin pitch both fall out of the engine with no extra machinery.
    return BOX2I( static_cast<const SCH_SHEET_PIN*>( aItem )->GetPosition(), VECTOR2I( 0, 0 ) );
}


bool EE_GRID_HELPER::IsSheetPinSelection( const SELECTION& aSelection )
{
    bool anyPin = false;

    for( const EDA_ITEM* item : aSelection )
    {
        if( item->Type() == SCH_SHEET_PIN_T )
            anyPin = true;
        else if( GetAlignmentBox( item ) )   // a sheet or symbol body: not a pin gesture
            return false;
    }

    return anyPin;
}


void EE_GRID_HELPER::CollectAlignmentNeighbors( const SCH_SELECTION& aSkip )
{
    ALIGNMENT_GUIDE_ENGINE& engine = getSnapManager().GetAlignmentEngine();
    engine.Clear();

    if( !m_moveContext || !m_toolMgr )
        return;

    // The viewport, generously inflated.  Neighbours are swept once at drag start, and limiting
    // them to what happens to be on screen makes the whole feature zoom-dependent: a fourth
    // sheet just past the bottom edge drops silently out of an equally-spaced run, and the user
    // sees two badges where there should be three.  Inflated in double so a zoomed-right-out
    // viewport cannot overflow on the way back to integers; MAX_GUIDE_NEIGHBORS still bounds
    // the cost.
    BOX2D viewbox = m_toolMgr->GetView()->GetViewport();
    viewbox.Inflate( viewbox.GetWidth(), viewbox.GetHeight() );

    const BOX2I viewport = BOX2ISafe( viewbox );

    std::vector<BOX2I> boxes;
    const VECTOR2D     ref( m_moveContext->OriginalBBox.Centre() );

    // One sweep, two rules.  A symbol is made of pins and graphics; a sheet is made of symbols
    // and subsheets.  Neither set of targets means anything in the other editor.  The frame
    // itself, not a bool: the container below needs its unit and body style.
    SYMBOL_EDIT_FRAME* symbolEditor = inSymbolEditor();

    // Which of the three box rules applies is decided once, here, by what is being dragged --
    // not by filtering targets later.  A mixed selection is a symbol move that happens to
    // include a graphic, and must keep the body rule; an empty one keeps it too.
    m_graphicsMode = !symbolEditor && !aSkip.Empty()
                     && std::all_of( aSkip.begin(), aSkip.end(),
                                     []( const EDA_ITEM* aItem )
                                     {
                                         return GetGraphicAlignmentBox( aItem ).has_value();
                                     } );

    // Disjoint from the other two by construction: a sheet pin has no graphic box, so the all_of
    // above already fails whenever one is in the selection.
    const bool sheetPinMode = !symbolEditor && IsSheetPinSelection( aSkip );

    // Named, not just measured.  A badge whose other end is off screen is impossible to account
    // for from coordinates alone, and the commonest surprise is an item nobody thought of as an
    // alignment target being one.
    auto pushTarget = [&]( const SCH_ITEM* aItem, const BOX2I& aBox )
    {
        if( wxLog::IsAllowedTraceMask( traceSnap ) )
        {
            wxLogTrace( traceSnap, "  alignment guides: target %s (%d, %d)-(%d, %d)",
                        aItem->GetClass(), aBox.GetLeft(), aBox.GetTop(), aBox.GetRight(),
                        aBox.GetBottom() );
        }

        boxes.push_back( aBox );
    };

    for( SCH_ITEM* item : queryVisible( viewport, aSkip ) )
    {
        if( sheetPinMode )
        {
            // Sheet pins are not view items -- SCH_SCREEN::Append() keeps them out of the R-tree --
            // so the query hands back the parent sheet and the pins have to be expanded from it.
            if( item->Type() != SCH_SHEET_T )
                continue;

            for( SCH_SHEET_PIN* pin : static_cast<SCH_SHEET*>( item )->GetPins() )
            {
                // The dragged pins are children of a sheet that is not itself selected, so
                // queryVisible()'s by-pointer erase never reaches them.  A target sitting at the
                // dragged pin's own position is an offset of zero, which wins its axis with an
                // unbeatable distance and would pin the drag in place with a permanent guide.
                if( !aSkip.Contains( pin ) )
                    pushTarget( pin, *GetSheetPinAlignmentBox( pin ) );
            }

            continue;
        }

        const std::optional<BOX2I> box = symbolEditor  ? GetSymbolAlignmentBox( item )
                                         : m_graphicsMode ? GetGraphicAlignmentBox( item )
                                                          : GetAlignmentBox( item );

        if( !box )
            continue;

        pushTarget( item, *box );
    }

    // The engine keeps the first candidate on a tie and walks neighbours in input order, so the
    // order decides which neighbour's edge the guide is drawn against.  queryVisible() hands back
    // a std::set, i.e. pointer-address order, which moves between runs -- hence sort always, not
    // only when the cap trims.  The bounds break distance ties so the order is total.
    auto sortKey = [&]( const BOX2I& aBox )
    {
        // Distance in double: the two centres can be far enough apart to overflow int.  Squared
        // is all an ordering needs, and skips the sqrt.
        return std::make_tuple( ( VECTOR2D( aBox.Centre() ) - ref ).SquaredEuclideanNorm(),
                                aBox.GetLeft(), aBox.GetTop(), aBox.GetRight(), aBox.GetBottom() );
    };

    // Guides are hints; dropping distant neighbours bounds the per-motion cost.
    constexpr size_t MAX_GUIDE_NEIGHBORS = 100;

    const size_t keep = std::min( boxes.size(), MAX_GUIDE_NEIGHBORS );

    std::partial_sort( boxes.begin(), boxes.begin() + keep, boxes.end(),
                       [&]( const BOX2I& a, const BOX2I& b )
                       { return sortKey( a ) < sortKey( b ); } );
    boxes.resize( keep );

    if( wxLog::IsAllowedTraceMask( traceSnap ) )
    {
        wxLogTrace( traceSnap, "  alignment guides: collected %zu neighbours", boxes.size() );

        for( const BOX2I& box : boxes )
        {
            wxLogTrace( traceSnap, "  alignment guides: neighbour (%d,%d)-(%d,%d)", box.GetLeft(),
                        box.GetTop(), box.GetRight(), box.GetBottom() );
        }
    }

    // Copied before the move: updateDynamicContainers() rebuilds the list every motion.
    if( m_graphicsMode )
        m_graphicsNeighbors = boxes;

    engine.SetNeighbors( std::move( boxes ) );

    // Containers: the area the moving item can be centred inside.  Mirrors the board outline
    // pcbnew feeds.  Computed once per drag, like the sweep above.
    if( symbolEditor )
    {
        // The body outline the user drew around the pins.  Pins excluded: a container is the
        // drawn area, and pins stick out of it by their length on every side, so including them
        // would centre a graphic against an edge nobody sees.  Private items likewise.
        if( LIB_SYMBOL* symbol = symbolEditor->GetCurSymbol() )
        {
            const BOX2I body = symbol->GetBodyBoundingBox( symbolEditor->GetUnit(),
                                                           symbolEditor->GetBodyStyle(), false,
                                                           false );

            // A symbol with no graphics yet leaves a default-constructed box, which the engine
            // would read as a real container at the origin.
            if( body.IsValid() )
                engine.SetContainers( { body } );
        }
    }
    else if( m_graphicsMode )
    {
        // No static container in graphics mode.  The container is the drawing-sheet cell the
        // item is currently over, which changes as the user carries it across the page, so it is
        // set per motion by updateDynamicContainers().
        //
        // The page container the branch below sets is deliberately not used here: it is the
        // *paper* rectangle, while the drawing frame is inset from it by the sheet margins.
        // Offering both would put two centring candidates millimetres apart, one of them on an
        // edge that is never drawn.
        collectDrawingSheetSegments();
    }
    else if( sheetPinMode )
    {
        // No container.  A sheet pin slides along its sheet's border, so "centred in the page" is
        // a position it cannot take and a candidate it must not be offered.
    }
    else if( SCH_BASE_FRAME* frame = dynamic_cast<SCH_BASE_FRAME*>( m_toolMgr->GetToolHolder() ) )
    {
        // The drawing sheet page.  Reached through the frame rather than GetModel(): no eeschema
        // tool casts GetModel(), so its type here is not something to bet a cast on.
        if( SCH_SCREEN* screen = frame->GetScreen() )
        {
            // Page origin is (0, 0) by KiCad convention and the sheet grows right and down.
            const VECTOR2D size = screen->GetPageSettings().GetSizeIU( schIUScale.IU_PER_MILS );

            if( size.x > 0 && size.y > 0 )
                engine.SetContainers( { BOX2I( VECTOR2I( 0, 0 ), VECTOR2I( KiROUND( size ) ) ) } );
        }
    }
}


SYMBOL_EDIT_FRAME* EE_GRID_HELPER::inSymbolEditor() const
{
    if( !m_toolMgr )
        return nullptr;

    return dynamic_cast<SYMBOL_EDIT_FRAME*>( m_toolMgr->GetToolHolder() );
}


void EE_GRID_HELPER::clearMoveState()
{
    m_sheetSegments.clear();
    m_graphicsNeighbors.clear();
    m_graphicsMode = false;
}


void EE_GRID_HELPER::collectDrawingSheetSegments()
{
    m_sheetSegments.clear();

    if( !m_toolMgr )
        return;

    SCH_BASE_FRAME* frame = dynamic_cast<SCH_BASE_FRAME*>( m_toolMgr->GetToolHolder() );

    if( !frame || !frame->GetCanvas() )
        return;

    KIGFX::SCH_VIEW* view = frame->GetCanvas()->GetView();

    if( !view || !view->GetDrawingSheet() )
        return;

    DS_DRAW_ITEM_LIST drawList( schIUScale );
    view->GetDrawingSheet()->BuildDrawList( view, &drawList );

    for( DS_DRAW_ITEM_BASE* item = drawList.GetFirst(); item; item = drawList.GetNext() )
    {
        switch( item->Type() )
        {
        case WSG_LINE_T:
        {
            const DS_DRAW_ITEM_LINE* line = static_cast<const DS_DRAW_ITEM_LINE*>( item );
            m_sheetSegments.emplace_back( line->GetStart(), line->GetEnd() );
            break;
        }

        case WSG_RECT_T:
        {
            // The page frame arrives this way: KiCad's default sheet draws the border as a rect,
            // so the four edges below are what a separator line snaps to.
            const DS_DRAW_ITEM_RECT* rect = static_cast<const DS_DRAW_ITEM_RECT*>( item );
            const VECTOR2I           a = rect->GetStart();
            const VECTOR2I           b = rect->GetEnd();

            m_sheetSegments.emplace_back( VECTOR2I( a.x, a.y ), VECTOR2I( b.x, a.y ) );
            m_sheetSegments.emplace_back( VECTOR2I( b.x, a.y ), VECTOR2I( b.x, b.y ) );
            m_sheetSegments.emplace_back( VECTOR2I( b.x, b.y ), VECTOR2I( a.x, b.y ) );
            m_sheetSegments.emplace_back( VECTOR2I( a.x, b.y ), VECTOR2I( a.x, a.y ) );
            break;
        }

        // Texts, bitmaps and polygons are content, not structure: they bound no cell.
        default:
            break;
        }
    }

    wxLogTrace( traceSnap, "  alignment guides: %zu drawing-sheet segments",
                m_sheetSegments.size() );
}


void EE_GRID_HELPER::updateDynamicContainers( const BOX2I& aMovingBox )
{
    if( !m_graphicsMode )
        return;

    ALIGNMENT_GUIDE_ENGINE& engine = getSnapManager().GetAlignmentEngine();

    // Measured from the moving box's centre, which is the point that ends up on the cell centre.
    const std::optional<BOX2I> cell = ALIGN_GEOM::CellAt( m_sheetSegments, aMovingBox.Centre() );

    // The cell goes in as a *neighbour*, not a container.  A neighbour already offers
    // centre-to-centre alongside its four edges, and collectAxisCandidates() pushes that before
    // any container candidate -- which FindSnap's keep-the-first-on-a-tie rule then makes
    // unreachable.  Registering the cell both ways is dead weight plus a second path that can
    // never run.
    //
    // Rebuilt every motion rather than once, because the user picks a logo up elsewhere on the
    // page and carries it to the corner box, so the cell changes mid-drag.  Dropped entirely when
    // the item is over no cell, or a logo dragged off the title block keeps being pulled back
    // into the cell it just left.
    std::vector<BOX2I> neighbors = m_graphicsNeighbors;

    if( cell )
        neighbors.push_back( *cell );

    engine.SetNeighbors( std::move( neighbors ) );
}


std::set<SCH_ITEM*> EE_GRID_HELPER::queryVisible( const BOX2I& aArea,
                                                  const SCH_SELECTION& aSkipList ) const
{
    std::set<SCH_ITEM*>                       items;
    std::vector<KIGFX::VIEW::LAYER_ITEM_PAIR> selectedItems;

    SYMBOL_EDIT_FRAME* symbolEditor = inSymbolEditor();
    KIGFX::VIEW*       view = m_toolMgr->GetView();

    view->Query( aArea, selectedItems );

    for( const KIGFX::VIEW::LAYER_ITEM_PAIR& it : selectedItems )
    {
        if( !it.first->IsSCH_ITEM() )
            continue;

        SCH_ITEM* item = static_cast<SCH_ITEM*>( it.first );

        // aSkipList is erased by pointer below, but when a group is dragged the selection holds
        // the group and not its members, so the members would survive as "neighbours" of the
        // thing they are being moved by.  An entered group is not itself SELECTED, so its
        // children stay visible here.
        if( item->HasSelectedAncestorGroup() )
            continue;

        if( symbolEditor )
        {
            // If we are in the symbol editor, don't use the symbol itself
            if( item->Type() == LIB_SYMBOL_T )
                continue;

            // Unit and body-style filtering is the painter's job (SCH_PAINTER::
            // isUnitAndConversionShown), and the view holds every unit and every body style at
            // once, so an item being in the view says nothing about it being on screen.  Without
            // this a guide can align a pin to the edge of a De Morgan body that is not drawn.
            // A zero unit or body style means "common to all", so those always qualify.
            if( item->GetUnit() && item->GetUnit() != symbolEditor->GetUnit() )
                continue;

            if( item->GetBodyStyle() && item->GetBodyStyle() != symbolEditor->GetBodyStyle() )
                continue;
        }
        else
        {
            // If we are not in the symbol editor, don't use symbol-editor-private items
            if( item->IsPrivate() )
                continue;
        }

        // The item must be visible and on an active layer
        if( view->IsVisible( item ) && item->ViewGetLOD( it.second, view ) < view->GetScale() )
            items.insert ( item );
    }

    for( EDA_ITEM* skipItem : aSkipList )
        items.erase( static_cast<SCH_ITEM*>( skipItem ) );

    return items;
}


GRID_HELPER_GRIDS EE_GRID_HELPER::GetSelectionGrid( const SELECTION& aSelection ) const
{
    GRID_HELPER_GRIDS grid = GetItemGrid( aSelection.Front() );

    // Find the largest grid of all the items and use that
    for( EDA_ITEM* item : aSelection )
    {
        GRID_HELPER_GRIDS itemGrid = GetItemGrid( item );

        if( GetGridSize( itemGrid ) > GetGridSize( grid ) )
            grid = itemGrid;
    }

    return grid;
}


GRID_HELPER_GRIDS EE_GRID_HELPER::GetItemGrid( const EDA_ITEM* aItem ) const
{
    if( !aItem )
        return GRID_CURRENT;

    switch( aItem->Type() )
    {
    case LIB_SYMBOL_T:
    case SCH_SYMBOL_T:
    case SCH_PIN_T:
    case SCH_SHEET_PIN_T:
    case SCH_SHEET_T:
    case SCH_NO_CONNECT_T:
    case SCH_GLOBAL_LABEL_T:
    case SCH_HIER_LABEL_T:
    case SCH_LABEL_T:
    case SCH_DIRECTIVE_LABEL_T:
    case SCH_RULE_AREA_T:
        return GRID_CONNECTABLE;

    case SCH_FIELD_T:
    case SCH_TEXT_T:
        return GRID_TEXT;

    case SCH_SHAPE_T:
    // The text box's border lines are what need to be on the graphic grid
    case SCH_TEXTBOX_T:
    case SCH_BITMAP_T:
        return GRID_GRAPHICS;

    case SCH_JUNCTION_T:
        return GRID_WIRES;

    case SCH_LINE_T:
        if( static_cast<const SCH_LINE*>( aItem )->IsConnectable() )
            return GRID_WIRES;
        else
            return GRID_GRAPHICS;

    case SCH_BUS_BUS_ENTRY_T:
    case SCH_BUS_WIRE_ENTRY_T:
        return GRID_WIRES;

    // Groups need to get the grid of their children
    case SCH_GROUP_T:
    {
        const SCH_GROUP* group = static_cast<const SCH_GROUP*>( aItem );

        // Shouldn't happen
        if( group->GetItems().empty() )
            return GRID_CURRENT;

        GRID_HELPER_GRIDS grid = GetItemGrid( *group->GetItems().begin() );

        for( EDA_ITEM* item : static_cast<const SCH_GROUP*>( aItem )->GetItems() )
        {
            GRID_HELPER_GRIDS itemGrid = GetItemGrid( item );

            if( GetGridSize( itemGrid ) > GetGridSize( grid ) )
                grid = itemGrid;
        }

        return grid;
    }

    default:
        return GRID_CURRENT;
    }
}


void EE_GRID_HELPER::computeAnchors( SCH_ITEM *aItem, const VECTOR2I &aRefPos, bool aFrom,
                                     bool aIncludeText )
{
    bool isGraphicLine =
            aItem->Type() == SCH_LINE_T && static_cast<SCH_LINE*>( aItem )->IsGraphicLine();

    switch( aItem->Type() )
    {
    case SCH_TEXT_T:
    case SCH_FIELD_T:
    {
        if( aIncludeText )
            addAnchor( aItem->GetPosition(), ORIGIN, aItem );

        break;
    }

    case SCH_TABLE_T:
    {
        if( aIncludeText )
        {
            addAnchor( aItem->GetPosition(), SNAPPABLE | CORNER, aItem );
            addAnchor( static_cast<SCH_TABLE*>( aItem )->GetEnd(), SNAPPABLE | CORNER, aItem );
        }

        break;
    }

    case SCH_TEXTBOX_T:
    case SCH_TABLECELL_T:
    {
        if( aIncludeText )
        {
            addAnchor( aItem->GetPosition(), SNAPPABLE | CORNER, aItem );
            addAnchor( static_cast<SCH_SHAPE*>( aItem )->GetEnd(), SNAPPABLE | CORNER, aItem );
        }

        break;
    }

    case SCH_SYMBOL_T:
    case SCH_SHEET_T:
        addAnchor( aItem->GetPosition(), ORIGIN, aItem );
        KI_FALLTHROUGH;

    case SCH_JUNCTION_T:
    case SCH_NO_CONNECT_T:
    case SCH_LINE_T:
        // Don't add anchors for graphic lines unless we're including text,
        // they may be on a non-connectable grid
        if( isGraphicLine && !aIncludeText )
            break;

        KI_FALLTHROUGH;
    case SCH_GLOBAL_LABEL_T:
    case SCH_HIER_LABEL_T:
    case SCH_LABEL_T:
    case SCH_DIRECTIVE_LABEL_T:
    case SCH_BUS_WIRE_ENTRY_T:
    case SCH_SHEET_PIN_T:
    {
        std::vector<VECTOR2I> pts = aItem->GetConnectionPoints();

        for( const VECTOR2I& pt : pts )
            addAnchor( VECTOR2I( pt ), SNAPPABLE | CORNER, aItem );

        break;
    }
    case SCH_PIN_T:
    {
        SCH_PIN* pin = static_cast<SCH_PIN*>( aItem );
        addAnchor( pin->GetPosition(), SNAPPABLE | ORIGIN, aItem );
        break;
    }

    case SCH_GROUP_T:
        for( EDA_ITEM* item : static_cast<SCH_GROUP*>( aItem )->GetItems() )
        {
            computeAnchors( static_cast<SCH_ITEM*>( item ), aRefPos, aFrom, aIncludeText );
        }

        break;

    default:
        break;
    }

    // Don't add anchors for graphic lines unless we're including text,
    // they may be on a non-connectable grid
    if( aItem->Type() == SCH_LINE_T && ( aIncludeText || !isGraphicLine ) )
    {
        SCH_LINE* line = static_cast<SCH_LINE*>( aItem );
        VECTOR2I  pt = Align( aRefPos );

        if( line->GetStartPoint().x == line->GetEndPoint().x )
        {
            VECTOR2I possible( line->GetStartPoint().x, pt.y );

            if( TestSegmentHit( possible, line->GetStartPoint(), line->GetEndPoint(), 0 ) )
                addAnchor( possible, SNAPPABLE | VERTICAL, aItem );
        }
        else if( line->GetStartPoint().y == line->GetEndPoint().y )
        {
            VECTOR2I possible( pt.x, line->GetStartPoint().y );

            if( TestSegmentHit( possible, line->GetStartPoint(), line->GetEndPoint(), 0 ) )
                addAnchor( possible, SNAPPABLE | HORIZONTAL, aItem );
        }
    }
}


EE_GRID_HELPER::ANCHOR* EE_GRID_HELPER::nearestAnchor( const VECTOR2I& aPos, int aFlags,
                                                       GRID_HELPER_GRIDS aGrid )
{
    double  minDist = std::numeric_limits<double>::max();
    ANCHOR* best = nullptr;

    for( ANCHOR& a : m_anchors )
    {
        if( ( aFlags & a.flags ) != aFlags )
            continue;

        // A "virtual" anchor with no real items associated shouldn't be filtered out
        if( !a.items.empty() )
        {
            // Filter using the first item
            SCH_ITEM* item = static_cast<SCH_ITEM*>( a.items[0] );

            if( aGrid == GRID_CONNECTABLE && !item->IsConnectable() )
                continue;
            else if( aGrid == GRID_GRAPHICS && item->IsConnectable() )
                continue;
        }

        double dist = a.Distance( aPos );

        if( dist < minDist )
        {
            minDist = dist;
            best = &a;
        }
    }

    return best;
}
