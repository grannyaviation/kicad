/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2019 CERN
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

#include <tool/tool_manager.h>
#include <tools/sch_selection_tool.h>
#include <sch_actions.h>
#include <ee_grid_helper.h>
#include <eda_item.h>
#include <gal/graphics_abstraction_layer.h>
#include <sch_shape.h>
#include <sch_group.h>
#include <sch_commit.h>
#include <wx/debug.h>
#include <view/view_controls.h>
#include "symbol_editor_move_tool.h"
#include "symbol_editor_pin_tool.h"


SYMBOL_EDITOR_MOVE_TOOL::SYMBOL_EDITOR_MOVE_TOOL() :
        SCH_TOOL_BASE( "eeschema.SymbolMoveTool" ),
        m_moveInProgress( false )
{
}


bool SYMBOL_EDITOR_MOVE_TOOL::Init()
{
    SCH_TOOL_BASE::Init();

    //
    // Add move actions to the selection tool menu
    //
    CONDITIONAL_MENU& selToolMenu = m_selectionTool->GetToolMenu().GetMenu();

    auto canMove =
            [&]( const SELECTION& sel )
            {
                SYMBOL_EDIT_FRAME* editor = static_cast<SYMBOL_EDIT_FRAME*>( m_frame );
                wxCHECK( editor, false );

                if( !editor->IsSymbolEditable() )
                    return false;

                if( editor->IsSymbolAlias() )
                {
                    for( EDA_ITEM* item : sel )
                    {
                        if( item->Type() != SCH_FIELD_T )
                            return false;
                    }
                }

                return true;
            };

    selToolMenu.AddItem( SCH_ACTIONS::move,         canMove && SCH_CONDITIONS::IdleSelection, 150 );
    selToolMenu.AddItem( SCH_ACTIONS::alignToGrid,  canMove && SCH_CONDITIONS::IdleSelection, 150 );

    return true;
}


void SYMBOL_EDITOR_MOVE_TOOL::Reset( RESET_REASON aReason )
{
    SCH_TOOL_BASE::Reset( aReason );

    if( aReason == MODEL_RELOAD )
        m_moveInProgress = false;
}


int SYMBOL_EDITOR_MOVE_TOOL::Main( const TOOL_EVENT& aEvent )
{
    if( SCH_COMMIT* commit = dynamic_cast<SCH_COMMIT*>( aEvent.Commit() ) )
    {
        wxCHECK( aEvent.SynchronousState(), 0 );
        aEvent.SynchronousState()->store( STS_RUNNING );

        if( doMoveSelection( aEvent, commit ) )
            aEvent.SynchronousState()->store( STS_FINISHED );
        else
            aEvent.SynchronousState()->store( STS_CANCELLED );
    }
    else
    {
        SCH_COMMIT localCommit( m_toolMgr );

        if( doMoveSelection( aEvent, &localCommit ) )
            localCommit.Push( _( "Move" ) );
        else
            localCommit.Revert();
    }

    return 0;
}


bool SYMBOL_EDITOR_MOVE_TOOL::doMoveSelection( const TOOL_EVENT& aEvent, SCH_COMMIT* aCommit )
{
    KIGFX::VIEW_CONTROLS* controls = getViewControls();
    EE_GRID_HELPER        grid( m_toolMgr );

    m_anchorPos = { 0, 0 };

    // Be sure that there is at least one item that we can move. If there's no selection try
    // looking for the stuff under mouse cursor (i.e. Kicad old-style hover selection).
    SCH_SELECTION& selection = m_frame->IsSymbolAlias() ? m_selectionTool->RequestSelection( { SCH_FIELD_T } )
                                                        : m_selectionTool->RequestSelection();
    bool           unselect = selection.IsHover();

    if( !m_frame->IsSymbolEditable() || selection.Empty() )
        return false;

    if( m_moveInProgress )
    {
        // The tool hotkey is interpreted as a click when already moving
        m_toolMgr->RunAction( ACTIONS::cursorClick );
        return true;
    }

    m_frame->PushTool( aEvent );

    Activate();
    // Must be done after Activate() so that it gets set into the correct context
    controls->ShowCursor( true );
    controls->SetAutoPan( true );

    bool        restore_state = false;
    TOOL_EVENT  copy = aEvent;
    TOOL_EVENT* evt = &copy;
    VECTOR2I    prevPos;
    VECTOR2I    moveOffset;

    // Axis locking for arrow key movement
    enum class AXIS_LOCK { NONE, HORIZONTAL, VERTICAL };
    AXIS_LOCK axisLock = AXIS_LOCK::NONE;
    long      lastArrowKeyAction = 0;

    // The moving box is re-measured whenever an event is passed on to another tool (the
    // catch-all at the bottom of the loop): rotate, mirror, swap and properties all reshape the
    // selection in place and then post refreshPreview, which re-enters the follow-the-mouse
    // branch below.  The neighbour sweep is a true one-shot -- it stores boxes by value and
    // excludes the moving selection, so nothing reshaping that selection invalidates it.
    bool updateBBox = true;
    bool collectGuideNeighbors = true;

    aCommit->Modify( m_frame->GetCurSymbol(), m_frame->GetScreen() );

    m_cursor = controls->GetCursorPosition( !aEvent.DisableGridSnapping() );

    // Main loop: keep receiving events
    do
    {
        m_frame->GetCanvas()->SetCurrentCursor( KICURSOR::MOVING );
        grid.SetSnap( !evt->Modifier( MD_SHIFT ) );
        grid.SetUseGrid( getView()->GetGAL()->GetGridSnapping() && !evt->DisableGridSnapping() );

        if( evt->IsAction( &SCH_ACTIONS::move )
                || evt->IsMotion()
                || evt->IsDrag( BUT_LEFT )
                || evt->IsAction( &ACTIONS::refreshPreview ) )
        {
            GRID_HELPER_GRIDS snapLayer = grid.GetSelectionGrid( selection );

            if( !m_moveInProgress )    // Prepare to start moving/dragging
            {
                SCH_ITEM* lib_item = static_cast<SCH_ITEM*>( selection.Front() );

                // Pick up any synchronized pins
                //
                // Careful when pasting.  The pasted pin will be at the same location as it
                // was copied from, leading us to believe it's a synchronized pin.  It's not.
                if( m_frame->SynchronizePins() && !( lib_item->GetEditFlags() & IS_PASTED ) )
                {
                    std::set<SCH_PIN*> sync_pins;

                    for( EDA_ITEM* sel_item : selection )
                    {
                        lib_item = static_cast<SCH_ITEM*>( sel_item );

                        if(  lib_item->Type() == SCH_PIN_T )
                        {
                            SCH_PIN*          cur_pin = static_cast<SCH_PIN*>( lib_item );
                            LIB_SYMBOL*       symbol = m_frame->GetCurSymbol();
                            std::vector<bool> got_unit( symbol->GetUnitCount() + 1 );

                            got_unit[cur_pin->GetUnit()] = true;

                            for( SCH_PIN* pin : symbol->GetPins() )
                            {
                                if( !got_unit[pin->GetUnit()]
                                        && pin->GetPosition() == cur_pin->GetPosition()
                                        && pin->GetOrientation() == cur_pin->GetOrientation()
                                        && pin->GetBodyStyle() == cur_pin->GetBodyStyle()
                                        && pin->GetType() == cur_pin->GetType()
                                        && pin->GetName() == cur_pin->GetName()  )
                                {
                                    if( sync_pins.insert( pin ).second )
                                        got_unit[pin->GetUnit()] = true;
                                }
                            }
                        }
                    }

                    for( SCH_PIN* pin : sync_pins )
                        m_selectionTool->AddItemToSel( pin, true /*quiet mode*/ );
                }

                // Apply any initial offset in case we're coming from a previous command.
                //
                for( EDA_ITEM* item : selection )
                    moveItem( item, moveOffset );

                // Set up the starting position and move/drag offset
                //
                m_cursor = controls->GetCursorPosition( !evt->DisableGridSnapping() );

                if( lib_item->IsNew() )
                {
                    m_anchorPos = selection.GetReferencePoint();
                    VECTOR2I delta = m_cursor - m_anchorPos;

                    // Drag items to the current cursor position
                    for( EDA_ITEM* item : selection )
                    {
                        SCH_ITEM* schItem = static_cast<SCH_ITEM*>( item );

                        moveItem( schItem, delta );
                        updateItem( schItem, false );

                        // While SCH_COMMIT::Push() will add any new items to the entered group,
                        // we need to do it earlier so that the previews while moving are correct.
                        if( SCH_GROUP* enteredGroup = m_selectionTool->GetEnteredGroup() )
                        {
                            if( schItem->IsGroupableType() && !item->GetParentGroup() )
                            {
                                aCommit->Modify( enteredGroup, m_frame->GetScreen(), RECURSE_MODE::NO_RECURSE );
                                enteredGroup->AddItem( schItem );
                            }
                        }
                    }

                    m_anchorPos = m_cursor;
                }
                else if( m_frame->GetMoveWarpsCursor() )
                {
                    // User wants to warp the mouse
                    m_cursor = grid.BestDragOrigin( m_cursor, snapLayer, selection );
                    selection.SetReferencePoint( m_cursor );
                    m_anchorPos = m_cursor;
                }
                else
                {
                    m_cursor = controls->GetCursorPosition( !evt->DisableGridSnapping() );
                    m_anchorPos = m_cursor;
                }

                controls->SetCursorPosition( m_cursor, false );

                prevPos = m_cursor;
                controls->SetAutoPan( true );
                m_moveInProgress = true;
            }

            if( updateBBox )
            {
                // Measured exactly as CollectAlignmentNeighbors() measures neighbours -- hence
                // the shared GetSymbolAlignmentBox() -- or the guides align edges that are not
                // where the user sees them.
                BOX2I guideBBox;
                bool  allBodies = !selection.Empty();

                for( EDA_ITEM* item : selection )
                {
                    const std::optional<BOX2I> box = EE_GRID_HELPER::GetSymbolAlignmentBox( item );

                    if( box )
                        guideBBox.Merge( *box );

                    // Guides outrank anchor snapping only for shapes.  A pin's guide box IS its
                    // snap anchor -- both are GetPosition() -- but the guide path only accepts
                    // whole-grid-step offsets where the anchor lands exactly, so preferring the
                    // guide for a pin can only lose targets.  Stacking pins exactly is the reason
                    // symbol authors drag pins at all.  Pins still contribute to the box and
                    // still get pitch guides; they just do not win the ranking.
                    if( !box || item->Type() == SCH_PIN_T )
                        allBodies = false;
                }

                // prevPos, not m_cursor: the items sit where prevPos put them and this event's
                // movement is applied further down.  The engine extrapolates the moving box from
                // that pair, so the two must agree.
                //
                // An invalid box would reach the engine as a real point box at the origin and
                // drag the selection towards it, so a selection with nothing measurable in it
                // gets no context at all rather than an empty one.
                if( guideBBox.IsValid() )
                {
                    grid.SetMoveContext( guideBBox, prevPos, allBodies );

                    // Must follow SetMoveContext(): the sweep sorts neighbours by distance from
                    // the moving box's centre.
                    if( collectGuideNeighbors )
                    {
                        grid.CollectAlignmentNeighbors( selection );
                        collectGuideNeighbors = false;
                    }
                }
                else
                {
                    grid.ClearMoveContext();
                }

                updateBBox = false;
            }

            //------------------------------------------------------------------------
            // Follow the mouse
            //
            // We need to bypass refreshPreview action here because it is triggered by the move,
            // so we were getting double-key events that toggled the axis locking if you
            // pressed them in a certain order.
            if( controls->GetSettings().m_lastKeyboardCursorPositionValid && !evt->IsAction( &ACTIONS::refreshPreview ) )
            {
                // This branch repositions without BestSnapAnchor(), which is where stale guides
                // normally get dropped.  m_lastKeyboardCursorPositionValid stays true until the
                // mouse really moves, so guides painted by the preceding drag would linger on
                // screen while the selection walks off under the arrow keys.
                grid.clearAlignmentGuides();

                VECTOR2I keyboardPos( controls->GetSettings().m_lastKeyboardCursorPosition );
                long action = controls->GetSettings().m_lastKeyboardCursorCommand;

                grid.SetSnap( false );
                m_cursor = grid.Align( keyboardPos, snapLayer );

                // Update axis lock based on arrow key press
                if( action == ACTIONS::CURSOR_LEFT || action == ACTIONS::CURSOR_RIGHT )
                {
                    if( axisLock == AXIS_LOCK::HORIZONTAL )
                    {
                        // Check if opposite horizontal key pressed to unlock
                        if( ( lastArrowKeyAction == ACTIONS::CURSOR_LEFT && action == ACTIONS::CURSOR_RIGHT ) ||
                            ( lastArrowKeyAction == ACTIONS::CURSOR_RIGHT && action == ACTIONS::CURSOR_LEFT ) )
                        {
                            axisLock = AXIS_LOCK::NONE;
                        }
                        // Same direction axis, keep locked
                    }
                    else
                    {
                        axisLock = AXIS_LOCK::HORIZONTAL;
                    }
                }
                else if( action == ACTIONS::CURSOR_UP || action == ACTIONS::CURSOR_DOWN )
                {
                    if( axisLock == AXIS_LOCK::VERTICAL )
                    {
                        // Check if opposite vertical key pressed to unlock
                        if( ( lastArrowKeyAction == ACTIONS::CURSOR_UP && action == ACTIONS::CURSOR_DOWN ) ||
                            ( lastArrowKeyAction == ACTIONS::CURSOR_DOWN && action == ACTIONS::CURSOR_UP ) )
                        {
                            axisLock = AXIS_LOCK::NONE;
                        }
                        // Same direction axis, keep locked
                    }
                    else
                    {
                        axisLock = AXIS_LOCK::VERTICAL;
                    }
                }

                lastArrowKeyAction = action;
            }
            else
            {
                m_cursor = grid.BestSnapAnchor( controls->GetCursorPosition( false ), snapLayer,
                                                selection );
            }

            if( axisLock != AXIS_LOCK::NONE )
            {
                const VECTOR2I unclamped = m_cursor;

                if( axisLock == AXIS_LOCK::HORIZONTAL )
                    m_cursor.y = prevPos.y;
                else
                    m_cursor.x = prevPos.x;

                // Only when the clamp actually overrode the snapped cursor.  axisLock is sticky
                // and survives into ordinary mouse motion, so clearing unconditionally would
                // suppress every guide for the rest of the drag after a single arrow-key nudge.
                //
                // Note this drops the guide *line*; the free-axis component of the snap has
                // already been folded into m_cursor by BestSnapAnchor and still applies.
                if( m_cursor != unclamped )
                    grid.clearAlignmentGuides();
            }

            VECTOR2I delta( m_cursor - prevPos );
            m_anchorPos = m_cursor;

            moveOffset += delta;
            prevPos = m_cursor;

            for( EDA_ITEM* item : selection )
            {
                moveItem( item, delta );
                updateItem( item, false );
            }

            m_toolMgr->PostEvent( EVENTS::SelectedItemsMoved );
        }
        //------------------------------------------------------------------------
        // Handle cancel
        //
        else if( evt->IsCancelInteractive() || evt->IsActivate() )
        {
            if( m_moveInProgress )
            {
                evt->SetPassEvent( false );
                restore_state = true;
            }

            break;
        }
        //------------------------------------------------------------------------
        // Handle TOOL_ACTION special cases
        //
        else if( evt->Action() == TA_UNDO_REDO_PRE )
        {
            unselect = true;
            break;
        }
        else if( evt->IsAction( &ACTIONS::doDelete ) )
        {
            // Exit on a remove operation; there is no further processing for removed items.
            break;
        }
        else if( evt->IsAction( &ACTIONS::duplicate ) )
        {
            wxBell();
        }
        //------------------------------------------------------------------------
        // Handle context menu
        //
        else if( evt->IsClick( BUT_RIGHT ) )
        {
            m_menu->ShowContextMenu( m_selectionTool->GetSelection() );
        }
        //------------------------------------------------------------------------
        // Handle drop
        //
        else if( evt->IsMouseUp( BUT_LEFT )
                || evt->IsClick( BUT_LEFT )
                || evt->IsDblClick( BUT_LEFT ) )
        {
            if( selection.GetSize() == 1 && selection.Front()->Type() == SCH_PIN_T )
            {
                SYMBOL_EDITOR_PIN_TOOL* pinTool = m_toolMgr->GetTool<SYMBOL_EDITOR_PIN_TOOL>();

                try
                {
                    SCH_PIN* curr_pin = static_cast<SCH_PIN*>( selection.Front() );

                    if( pinTool->PlacePin( aCommit, curr_pin ) )
                    {
                        // PlacePin() clears the current selection, which we don't want.  Not only
                        // is it a poor user experience, but it also prevents us from doing the
                        // proper cleanup at the end of this routine (ie: clearing the edit flags).
                        m_selectionTool->AddItemToSel( curr_pin, true /*quiet mode*/ );
                    }
                    else
                    {
                        restore_state = true;
                    }
                }
                catch( const boost::bad_pointer& e )
                {
                    restore_state = true;
                    wxFAIL_MSG( wxString::Format( wxT( "Boost pointer exception occurred: %s" ),
                                                  e.what() ) );
                }
            }

            break; // Finish
        }
        else
        {
            // Everything this tool does not handle itself goes to another tool, and that is
            // where the remaining geometry mutations live -- rotate, mirror, swap, properties --
            // each of which posts refreshPreview, so the next motion frame re-measures before
            // BestSnapAnchor() reads the box.
            updateBBox = true;
            evt->SetPassEvent();
        }

    } while( ( evt = Wait() ) );  // Assignment intentional; not equality test

    controls->ForceCursorPosition( false );
    controls->ShowCursor( false );
    controls->SetAutoPan( false );

    m_anchorPos = { 0, 0 };

    for( EDA_ITEM* item : selection )
        item->ClearEditFlags();

    if( unselect )
        m_toolMgr->RunAction( ACTIONS::selectionClear );

    m_moveInProgress = false;

    // Not load-bearing today -- grid is function-local, so its destructor would unlink the guide
    // overlay anyway -- but this makes teardown explicit rather than depending on that.
    grid.ClearMoveContext();

    m_frame->PopTool( aEvent );

    return !restore_state;
}


int SYMBOL_EDITOR_MOVE_TOOL::AlignElements( const TOOL_EVENT& aEvent )
{
    EE_GRID_HELPER grid( m_toolMgr);
    SCH_SELECTION& selection = m_selectionTool->RequestSelection();
    SCH_COMMIT     commit( m_toolMgr );

    for( EDA_ITEM* item : selection )
    {
        VECTOR2I newPos = grid.AlignGrid( item->GetPosition(), grid.GetItemGrid( item ) );
        VECTOR2I delta = newPos - item->GetPosition();

        if( delta != VECTOR2I( 0, 0 ) )
        {
            commit.Modify( item, m_frame->GetScreen(), RECURSE_MODE::RECURSE );
            static_cast<SCH_ITEM*>( item )->Move( delta );
            updateItem( item, true );
        };

        if( SCH_PIN* pin = dynamic_cast<SCH_PIN*>( item ) )
        {
            int length = pin->GetLength();
            int pinGrid;

            if( pin->GetOrientation() == PIN_ORIENTATION::PIN_LEFT
                    || pin->GetOrientation() == PIN_ORIENTATION::PIN_RIGHT )
            {
                pinGrid = KiROUND( grid.GetGridSize( grid.GetItemGrid( item ) ).x );
            }
            else
            {
                pinGrid = KiROUND( grid.GetGridSize( grid.GetItemGrid( item ) ).y );
            }

            int newLength = KiROUND( (double) length / pinGrid ) * pinGrid;

            if( newLength > 0 )
                pin->SetLength( newLength );
        }
    }

    m_toolMgr->PostEvent( EVENTS::SelectedItemsMoved );

    commit.Push( _( "Align Items to Grid" ) );
    return 0;
}


void SYMBOL_EDITOR_MOVE_TOOL::moveItem( EDA_ITEM* aItem, const VECTOR2I& aDelta )
{
    static_cast<SCH_ITEM*>( aItem )->Move( aDelta );
    aItem->SetFlags( IS_MOVING );
}


void SYMBOL_EDITOR_MOVE_TOOL::setTransitions()
{
    Go( &SYMBOL_EDITOR_MOVE_TOOL::Main,               SCH_ACTIONS::move.MakeEvent() );
    Go( &SYMBOL_EDITOR_MOVE_TOOL::AlignElements,      SCH_ACTIONS::alignToGrid.MakeEvent() );
}
