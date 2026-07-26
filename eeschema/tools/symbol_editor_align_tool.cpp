/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3
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

#include <bitmaps.h>
#include <sch_actions.h>
#include <sch_commit.h>
#include <sch_item.h>
#include <sch_selection.h>
#include <sch_selection_tool.h>
#include <symbol_edit_frame.h>
#include <tool/conditional_menu.h>
#include <tool/selection_conditions.h>
#include <tool/tool_event.h>
#include <tools/ee_grid_helper.h>
#include <tools/symbol_editor_align_tool.h>
#include <view/view_controls.h>

SYMBOL_EDITOR_ALIGN_TOOL::SYMBOL_EDITOR_ALIGN_TOOL() :
        SCH_TOOL_BASE<SYMBOL_EDIT_FRAME>( "eeschema.SymbolAlign" )
{
}


SYMBOL_EDITOR_ALIGN_TOOL::~SYMBOL_EDITOR_ALIGN_TOOL()
{
    delete m_alignMenu;
}


bool SYMBOL_EDITOR_ALIGN_TOOL::Init()
{
    SCH_TOOL_BASE::Init();

    if( !m_alignMenu )
    {
        m_alignMenu = new CONDITIONAL_MENU( this );
        m_alignMenu->SetIcon( BITMAPS::align_items );
        m_alignMenu->SetUntranslatedTitle( _HKI( "Align" ) );

        const auto canAlign = SELECTION_CONDITIONS::MoreThan( 1 );

        m_alignMenu->AddItem( SCH_ACTIONS::alignLeft, canAlign );
        m_alignMenu->AddItem( SCH_ACTIONS::alignCenterX, canAlign );
        m_alignMenu->AddItem( SCH_ACTIONS::alignRight, canAlign );

        m_alignMenu->AddSeparator( canAlign );
        m_alignMenu->AddItem( SCH_ACTIONS::alignTop, canAlign );
        m_alignMenu->AddItem( SCH_ACTIONS::alignCenterY, canAlign );
        m_alignMenu->AddItem( SCH_ACTIONS::alignBottom, canAlign );
    }

    CONDITIONAL_MENU& selToolMenu = m_selectionTool->GetToolMenu().GetMenu();
    selToolMenu.AddMenu( m_alignMenu, SELECTION_CONDITIONS::MoreThan( 1 ), 100 );

    setTransitions();

    return true;
}


// Twin of SCH_ALIGN_TOOL::adjustDeltaForGrid (eeschema/tools/sch_align_tool.cpp).  Not shared
// because that would mean touching the schematic tool a second time, and that tool has no
// automated test coverage.  If the grid rule here changes, change it there too.
VECTOR2I SYMBOL_EDITOR_ALIGN_TOOL::adjustDeltaForGrid( SCH_ITEM* aItem, const VECTOR2I& aDelta )
{
    if( aDelta == VECTOR2I( 0, 0 ) )
        return aDelta;

    EE_GRID_HELPER     grid( m_toolMgr );
    GRID_HELPER_GRIDS  gridType = grid.GetItemGrid( aItem );

    if( gridType != GRID_CONNECTABLE )
        return aDelta;

    VECTOR2I desiredPos = aItem->GetPosition() + aDelta;
    VECTOR2I snappedPos = grid.AlignGrid( desiredPos, gridType );

    return snappedPos - aItem->GetPosition();
}


int SYMBOL_EDITOR_ALIGN_TOOL::doAlign( ALIGN_GEOM::MODE aMode, const wxString& aUndoLabel )
{
    SCH_SELECTION& selection = m_selectionTool->RequestSelection();

    // Aligning one item to itself is a no-op that must not push an undo entry.
    if( selection.Size() < 2 )
        return 0;

    // A derived symbol's graphics belong to its parent, mirroring
    // SYMBOL_EDITOR_MOVE_TOOL::doMoveSelection's editability guard.
    if( !m_frame->IsSymbolEditable() || m_frame->IsSymbolAlias() )
        return 0;

    using ITEM_BOX = std::pair<SCH_ITEM*, BOX2I>;
    std::vector<ITEM_BOX> itemBoxes;

    for( EDA_ITEM* item : selection )
    {
        if( !item->IsSCH_ITEM() )
            continue;

        SCH_ITEM* schItem = static_cast<SCH_ITEM*>( item );

        // GetBoundingBox(), not EE_GRID_HELPER::GetSymbolAlignmentBox().  That is the guides'
        // notion of an alignable body and it rejects text and fields, which a user may well want
        // to align with an explicit command.
        itemBoxes.emplace_back( schItem, schItem->GetBoundingBox() );
    }

    if( itemBoxes.size() < 2 )
        return 0;

    // Sort by the edge being aligned, so the fallback target (when the cursor lands over no box)
    // is the extreme item on that axis -- mirrors the comparison lambdas SCH_ALIGN_TOOL passes to
    // GetSelections() for each mode.
    std::sort( itemBoxes.begin(), itemBoxes.end(),
            [aMode]( const ITEM_BOX& lhs, const ITEM_BOX& rhs )
            {
                switch( aMode )
                {
                case ALIGN_GEOM::MODE::TOP:      return lhs.second.GetTop() < rhs.second.GetTop();
                case ALIGN_GEOM::MODE::BOTTOM:   return lhs.second.GetBottom() > rhs.second.GetBottom();
                case ALIGN_GEOM::MODE::LEFT:     return lhs.second.GetLeft() < rhs.second.GetLeft();
                case ALIGN_GEOM::MODE::RIGHT:    return lhs.second.GetRight() > rhs.second.GetRight();
                case ALIGN_GEOM::MODE::CENTER_X: return lhs.second.Centre().x < rhs.second.Centre().x;
                case ALIGN_GEOM::MODE::CENTER_Y: return lhs.second.Centre().y < rhs.second.Centre().y;
                }

                return false;
            } );

    std::vector<SCH_ITEM*> items;
    std::vector<BOX2I>     boxes;

    for( const ITEM_BOX& itemBox : itemBoxes )
    {
        items.push_back( itemBox.first );
        boxes.push_back( itemBox.second );
    }

    // A symbol editor tool commits the whole LIB_SYMBOL once, rather than per item as
    // SCH_ALIGN_TOOL does.
    SCH_COMMIT commit( m_toolMgr );
    commit.Modify( m_frame->GetCurSymbol(), m_frame->GetScreen() );

    std::optional<size_t> target =
            ALIGN_GEOM::SelectTargetIndex( boxes, getViewControls()->GetCursorPosition() );

    if( !target )
        return 0;

    const std::vector<VECTOR2I> deltas = ALIGN_GEOM::Deltas( boxes, aMode, boxes[*target] );

    for( size_t i = 0; i < items.size(); ++i )
    {
        // Pins are GRID_CONNECTABLE and must land back on the grid after an align -- a pin left
        // off-grid in a library part is a defect that propagates into every schematic using the
        // symbol.  Shapes (GRID_GRAPHICS) and text (GRID_TEXT) are unaffected.
        VECTOR2I delta = adjustDeltaForGrid( items[i], deltas[i] );

        if( delta == VECTOR2I( 0, 0 ) )
            continue;

        items[i]->Move( delta );
        updateItem( items[i], true );
    }

    commit.Push( aUndoLabel );
    return 0;
}


void SYMBOL_EDITOR_ALIGN_TOOL::setTransitions()
{
    Go( &SYMBOL_EDITOR_ALIGN_TOOL::AlignTop,      SCH_ACTIONS::alignTop.MakeEvent() );
    Go( &SYMBOL_EDITOR_ALIGN_TOOL::AlignBottom,   SCH_ACTIONS::alignBottom.MakeEvent() );
    Go( &SYMBOL_EDITOR_ALIGN_TOOL::AlignLeft,     SCH_ACTIONS::alignLeft.MakeEvent() );
    Go( &SYMBOL_EDITOR_ALIGN_TOOL::AlignRight,    SCH_ACTIONS::alignRight.MakeEvent() );
    Go( &SYMBOL_EDITOR_ALIGN_TOOL::AlignCenterX,  SCH_ACTIONS::alignCenterX.MakeEvent() );
    Go( &SYMBOL_EDITOR_ALIGN_TOOL::AlignCenterY,  SCH_ACTIONS::alignCenterY.MakeEvent() );
}


int SYMBOL_EDITOR_ALIGN_TOOL::AlignTop( const TOOL_EVENT& aEvent )
{
    return doAlign( ALIGN_GEOM::MODE::TOP, _( "Align to Top" ) );
}


int SYMBOL_EDITOR_ALIGN_TOOL::AlignBottom( const TOOL_EVENT& aEvent )
{
    return doAlign( ALIGN_GEOM::MODE::BOTTOM, _( "Align to Bottom" ) );
}


int SYMBOL_EDITOR_ALIGN_TOOL::AlignLeft( const TOOL_EVENT& aEvent )
{
    return doAlign( ALIGN_GEOM::MODE::LEFT, _( "Align to Left" ) );
}


int SYMBOL_EDITOR_ALIGN_TOOL::AlignRight( const TOOL_EVENT& aEvent )
{
    return doAlign( ALIGN_GEOM::MODE::RIGHT, _( "Align to Right" ) );
}


int SYMBOL_EDITOR_ALIGN_TOOL::AlignCenterX( const TOOL_EVENT& aEvent )
{
    return doAlign( ALIGN_GEOM::MODE::CENTER_X, _( "Align to Middle" ) );
}


int SYMBOL_EDITOR_ALIGN_TOOL::AlignCenterY( const TOOL_EVENT& aEvent )
{
    return doAlign( ALIGN_GEOM::MODE::CENTER_Y, _( "Align to Center" ) );
}
