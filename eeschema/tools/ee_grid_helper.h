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

#ifndef EE_GRID_HELPER_H
#define EE_GRID_HELPER_H

#include <optional>

#include <math/vector2d.h>
#include <origin_viewitem.h>
#include <tool/grid_helper.h>
#include "sch_selection.h"

class SCH_ITEM;
class SYMBOL_EDIT_FRAME;


class EE_GRID_HELPER : public GRID_HELPER
{
public:

    EE_GRID_HELPER();
    EE_GRID_HELPER( TOOL_MANAGER* aToolMgr );
    ~EE_GRID_HELPER() override;

    /**
     * Function GetSnapped
     * If the EE_GRID_HELPER has highlighted a snap point (target shown), this function
     * will return a pointer to the item to which it snapped.
     *
     * @return NULL if not snapped.  Pointer to snapped item otherwise
     */
    SCH_ITEM* GetSnapped() const;

    VECTOR2D GetGridSize( GRID_HELPER_GRIDS aGrid ) const override;
    using GRID_HELPER::GetGrid;

    GRID_HELPER_GRIDS GetSelectionGrid( const SELECTION& aItem ) const override;
    GRID_HELPER_GRIDS GetItemGrid( const EDA_ITEM* aItem ) const override;

    VECTOR2I BestDragOrigin( const VECTOR2I& aMousePos, GRID_HELPER_GRIDS aGrid,
                             const SCH_SELECTION& aItems );

    VECTOR2I BestSnapAnchor( const VECTOR2I& aOrigin, GRID_HELPER_GRIDS aGrid, SCH_ITEM* aSkip );
    VECTOR2I BestSnapAnchor( const VECTOR2I& aOrigin, GRID_HELPER_GRIDS aGrid,
                             const SCH_SELECTION& aSkip = {} );

    /**
     * The box smart alignment guides measure an item by, or nullopt if the item is not
     * something anyone aligns to.
     *
     * Single source of truth on purpose: the moving selection and the neighbour set must be
     * measured by the same rule, or every guide sits offset by whatever the two rules disagree
     * about (field text, a stroke halo).
     */
    static std::optional<BOX2I> GetAlignmentBox( const EDA_ITEM* aItem );

    /**
     * The box alignment guides measure a *symbol editor* item by, or nullopt if it is not
     * something anyone aligns to.
     *
     * Separate from GetAlignmentBox() on purpose rather than merged with it.  SCH_PIN_T and
     * SCH_SHAPE_T both occur on a schematic sheet as well, and teaching the schematic rule about
     * them would put a guide on every pin of every symbol -- which would make chips snap to GND
     * flags rather than to each other.
     */
    static std::optional<BOX2I> GetSymbolAlignmentBox( const EDA_ITEM* aItem );

    /**
     * True when a point of aItem that has to land on the grid does not.
     *
     * Reads SCH_ITEM::GetConnectionPoints() -- the same points ERC's off-grid endpoint test
     * looks at, i.e. pins for a symbol, sheet pins for a sheet, both ends of a wire.  Items with
     * nothing connectable (graphics, text) can never be off grid in the sense that matters, so
     * they never warn.
     *
     * @param aGrid grid step; a non-positive component disables the check on that axis
     */
    static bool IsOffGrid( const EDA_ITEM* aItem, const VECTOR2I& aGrid,
                           const VECTOR2I& aOrigin = VECTOR2I( 0, 0 ) );

    /**
     * Paint a warning glyph on every item of aSelection that IsOffGrid() on aGrid.
     *
     * Call it once per motion during a move, after the items have been repositioned: a symbol
     * that was merely *placed* off grid loses its warning as soon as the move snaps it back,
     * and only one whose pin pitch does not divide the grid keeps it for the whole drag.
     */
    void ShowOffGridWarnings( const SELECTION& aSelection, GRID_HELPER_GRIDS aGrid );

    /**
     * Collect neighbour bounding boxes for smart alignment guides.
     * Call once at drag start, after SetMoveContext().
     *
     * @param aSkip the items being dragged (excluded from the neighbour set)
     */
    void CollectAlignmentNeighbors( const SCH_SELECTION& aSkip );

    /**
     * Snap a single dragged point to the alignment guides, painting them, and return where it
     * should go.  Returns aPoint unchanged when nothing is in range.
     *
     * For resize handles, where the moving geometry is one corner rather than a whole body.
     * Unlike BestSnapAnchor() this arbitrates nothing: the caller has no competing snap of its
     * own, so the guide either wins or nothing does.
     *
     * @param aPoint the grid-aligned position the handle would take with no guides
     * @param aCollectSkip when set, (re)collect the neighbours first, excluding these items.
     *                     Pass it on the first motion of a resize only: the sweep walks the
     *                     whole viewport, and it has to run after the move context is set.
     */
    VECTOR2I AlignPointToGuides( const VECTOR2I&      aPoint,
                                 const SCH_SELECTION* aCollectSkip = nullptr );

private:
    /// The symbol editor frame this helper belongs to, or nullptr in the schematic.  The two
    /// editors have entirely different ideas of what an alignment target is, and one sweep
    /// serves both -- and the symbol editor additionally needs the frame's unit and body style,
    /// because filtering those is the painter's job and the view holds every one of them.
    SYMBOL_EDIT_FRAME* inSymbolEditor() const;

    std::set<SCH_ITEM*> queryVisible( const BOX2I& aArea, const SCH_SELECTION& aSkipList ) const;

    ANCHOR* nearestAnchor( const VECTOR2I& aPos, int aFlags, GRID_HELPER_GRIDS aGrid );

    /**
     * Insert the local anchor points in to the grid helper for the specified
     * schematic item, given the reference point and the direction of use for the point.
     *
     * @param aItem The schematic item for which to compute the anchors
     * @param aRefPos The point for which to compute the anchors (if used by the symbol)
     * @param aFrom Is this for an anchor that is designating a source point (aFrom=true) or not
     * @param aIncludeText if true will compute anchors for text items
     */
    void computeAnchors( SCH_ITEM* aItem, const VECTOR2I& aRefPos, bool aFrom = false,
                         bool aIncludeText = false );
};

#endif
