/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
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

#ifndef GRID_HELPER_H
#define GRID_HELPER_H

#include <vector>
#include <optional>

#include <geometry/point_types.h>
#include <math/vector2d.h>
#include <preview_items/anchor_debug.h>
#include <preview_items/snap_indicator.h>
#include <preview_items/construction_geom.h>
#include <preview_items/alignment_guide_geom.h>
#include <tool/construction_manager.h>
#include <tool/selection.h>
#include <origin_viewitem.h>

class TOOL_MANAGER; // Forward declaration to avoid hard dependency in tests

class EDA_ITEM;

enum GRID_HELPER_GRIDS : int
{
    // When the item doesn't match an override, use the current user grid
    GRID_CURRENT,

    GRID_CONNECTABLE,
    GRID_WIRES,
    GRID_VIAS,
    GRID_TEXT,
    GRID_GRAPHICS
};

class GRID_HELPER
{
    friend void TEST_CLEAR_ANCHORS( GRID_HELPER& helper );
public:
    /**
     * @param aIuScale the owning editor's internal-unit scale (pcbIUScale, schIUScale, ...).
     *                 Only the alignment-guide badges read it, but there is no sane default:
     *                 a wrong scale renders a wrong number rather than failing, so every
     *                 subclass names its own.
     */
    GRID_HELPER( const EDA_IU_SCALE& aIuScale );
    GRID_HELPER( TOOL_MANAGER* aToolMgr, int aConstructionLayer, const EDA_IU_SCALE& aIuScale );
    virtual ~GRID_HELPER();

    VECTOR2I GetGrid() const;
    VECTOR2D GetVisibleGrid() const;
    VECTOR2I GetOrigin() const;

    /**
     * Reset all internal state.  Used to remove any dangling pointers to items
     * that have been deleted.
     */
    virtual void FullReset()
    {
        m_constructionGeomPreview.ClearSnapLine();
        m_snapManager.Clear();
        m_anchors.clear();
        m_moveContext = std::nullopt;
        m_alignGuidePreview.ClearGuides();
        SetOffGridWarnings( {} );
        clearMoveState();
    }

    /**
     * Provide the context needed for smart alignment guides during a move:
     * the moving selection's bbox and the cursor position at drag start.
     * While set, BestSnapAnchor implementations may offer alignment snaps.
     *
     * @param aPreferGuides rank an available alignment guide above item-anchor snapping.
     *                      Only for selections that have no business snapping to anchors
     *                      (whole symbols); a wire end must keep snapping to pins.
     */
    void SetMoveContext( const BOX2I& aOriginalBBox, const VECTOR2I& aOriginalCursor,
                         bool aPreferGuides = false )
    {
        m_moveContext = MOVE_CONTEXT{ aOriginalBBox, aOriginalCursor, aPreferGuides };
    }

    void ClearMoveContext()
    {
        m_moveContext = std::nullopt;
        m_snapManager.GetAlignmentEngine().Clear();
        m_alignGuidePreview.ClearGuides();
        SetOffGridWarnings( {} );
        clearMoveState();
    }

    /**
     * Paint a warning glyph at each of aPositions, replacing any already showing.
     *
     * Separate from the alignment guides on purpose: clearAlignmentGuides() fires mid-drag
     * (axis lock, arrow-key nudge) and must not take the warnings with it -- an item that
     * cannot sit on the grid stays that way whether or not anything is aligning to it.
     */
    void SetOffGridWarnings( std::vector<VECTOR2I> aPositions );

    /// Remove any painted alignment guides.  Cheap no-op when none are showing.
    ///
    /// Public because the move tools have to call it too: any path that repositions the
    /// selection without going through BestSnapAnchor (e.g. the arrow-key nudge in
    /// SCH_MOVE_TOOL) would otherwise leave the guides painted at a stale ordinate.
    void clearAlignmentGuides();

    // Manual setters used when no TOOL_MANAGER/View is available (e.g. in tests)
    void SetGridSize( const VECTOR2D& aGrid ) { m_manualGrid = aGrid; }
    void SetVisibleGridSize( const VECTOR2D& aGrid ) { m_manualVisibleGrid = aGrid; }
    void SetOrigin( const VECTOR2I& aOrigin ) { m_manualOrigin = aOrigin; }
    void SetGridSnapping( bool aEnable ) { m_manualGridSnapping = aEnable; }

    void SetAuxAxes( bool aEnable, const VECTOR2I& aOrigin = VECTOR2I( 0, 0 ) );

    virtual VECTOR2I Align( const VECTOR2I& aPoint, GRID_HELPER_GRIDS aGrid ) const
    {
        return Align( aPoint, GetGridSize( aGrid ), GetOrigin() );
    }

    virtual VECTOR2I AlignGrid( const VECTOR2I& aPoint, GRID_HELPER_GRIDS aGrid ) const
    {
        return AlignGrid( aPoint, GetGridSize( aGrid ), GetOrigin() );
    }

    virtual VECTOR2I Align( const VECTOR2I& aPoint ) const;
    virtual VECTOR2I Align( const VECTOR2I& aPoint, const VECTOR2D& aGrid,
                            const VECTOR2D& aOffset ) const;

    VECTOR2I AlignGrid( const VECTOR2I& aPoint ) const;
    VECTOR2I AlignGrid( const VECTOR2I& aPoint, const VECTOR2D& aGrid,
                        const VECTOR2D& aOffset ) const;

    /**
     * Gets the coarsest grid that applies to a selecion of items.
     */
    virtual GRID_HELPER_GRIDS GetSelectionGrid( const SELECTION& aSelection ) const;

    /**
     * Get the coarsest grid that applies to an item.
     */
    virtual GRID_HELPER_GRIDS GetItemGrid( const EDA_ITEM* aItem ) const { return GRID_CURRENT; }

    /**
     * Return the size of the specified grid.
     */
    virtual VECTOR2D GetGridSize( GRID_HELPER_GRIDS aGrid ) const;

    void SetSkipPoint( const VECTOR2I& aPoint )
    {
        m_skipPoint = aPoint;
    }

    /**
     * Clear the skip point by setting it to an unreachable position, thereby preventing matching.
     */
    void ClearSkipPoint()
    {
        m_skipPoint = VECTOR2I( std::numeric_limits<int>::min(), std::numeric_limits<int>::min() );
    }

    void SetSnap( bool aSnap ) { m_enableSnap = aSnap; }
    bool GetSnap() const { return m_enableSnap; }

    void SetUseGrid( bool aSnapToGrid ) { m_enableGrid = aSnapToGrid; }
    bool GetUseGrid() const { return m_enableGrid; }

    void SetSnapLine( bool aSnap ) { m_enableSnapLine = aSnap; }
    void SetSnapLineDirections( const std::vector<VECTOR2I>& aDirections );
    void SetSnapLineOrigin( const VECTOR2I& aOrigin );
    void SetSnapLineEnd( const std::optional<VECTOR2I>& aEnd );
    void ClearSnapLine();
    std::optional<VECTOR2I> SnapToConstructionLines( const VECTOR2I& aPoint,
                                                     const VECTOR2I& aNearestGrid,
                                                     const VECTOR2D& aGrid,
                                                     double aSnapRange ) const;

    void SetMask( int aMask ) { m_maskTypes = aMask; }
    void SetMaskFlag( int aFlag ) { m_maskTypes |= aFlag; }
    void ClearMaskFlag( int aFlag ) { m_maskTypes = m_maskTypes & ~aFlag; }

    std::optional<VECTOR2I> GetSnappedPoint() const;

    enum ANCHOR_FLAGS
    {
        CORNER = 1,
        OUTLINE = 2,
        SNAPPABLE = 4,
        ORIGIN = 8,
        VERTICAL = 16,
        HORIZONTAL = 32,

        // This anchor comes from 'constructed' geometry (e.g. an intersection
        // with something else), and not from some intrinsic point of an item
        // (e.g. an endpoint)
        CONSTRUCTED = 64,
        ALL = CORNER | OUTLINE | SNAPPABLE | ORIGIN | VERTICAL | HORIZONTAL | CONSTRUCTED
    };

protected:

    struct ANCHOR
    {
        /**
         * @param aPos The position of the anchor.
         * @param aFlags The flags for the anchor - this is a bitfield of ANCHOR_FLAGS,
         *               specifying the type of anchor (which may be used to filter out
         *               unwanted anchors per the settings).
         * @param aPointTypes The point types that this anchor represents in geometric terms.
         * @param aItem The item to which the anchor belongs.
         */
        ANCHOR( const VECTOR2I& aPos, int aFlags, int aPointTypes, std::vector<EDA_ITEM*> aItems ) :
                pos( aPos ), flags( aFlags ), pointTypes( aPointTypes ),
                items( std::move( aItems ) )
        {
        }

        VECTOR2I  pos;
        int       flags;
        int       pointTypes;

        /// Items that are associated with this anchor (can be more than one, e.g. for an
        /// intersection).
        std::vector<EDA_ITEM*> items;

        double Distance( const VECTOR2I& aP ) const
        {
            return VECTOR2D( (double) aP.x - pos.x, (double) aP.y - pos.y ).EuclideanNorm();
        }

        bool InvolvesItem( const EDA_ITEM& aItem ) const
        {
            return std::find( items.begin(), items.end(), &aItem ) != items.end();
        }
    };

    void addAnchor( const VECTOR2I& aPos, int aFlags, EDA_ITEM* aItem,
                    int aPointTypes = POINT_TYPE::PT_NONE )
    {
        addAnchor( aPos, aFlags, std::vector<EDA_ITEM*>{ aItem }, aPointTypes );
    }

    void addAnchor( const VECTOR2I& aPos, int aFlags, std::vector<EDA_ITEM*> aItems,
                    int aPointTypes )
    {
        if( ( aFlags & m_maskTypes ) == aFlags )
            m_anchors.emplace_back( ANCHOR( aPos, aFlags, aPointTypes, std::move( aItems ) ) );
    }

    void clearAnchors()
    {
        m_anchors.clear();
    }

    /**
     * Check whether it is possible to use the grid -- this depends both on local grid helper
     * settings and global (tool manager) KiCad settings.
     */
    bool canUseGrid() const;

    VECTOR2I computeNearest( const VECTOR2I& aPoint, const VECTOR2I& aGrid,
                             const VECTOR2I& aOffset ) const;

    /**
     * An alignment-guide snap that has been computed but not painted.
     *
     * Computing and painting are separate steps so a caller can rank the guide against its
     * other snap sources and paint only the winner: painting first and then returning some
     * other snap leaves guide lines on the canvas for a snap that isn't happening.
     *
     * Position carries the resolved point rather than the raw offset so it cannot be added
     * to a different base than the one it was computed from.
     */
    struct GUIDE_SNAP
    {
        VECTOR2I                       Position; ///< Where the caller should snap to
        ALIGNMENT_GUIDE_ENGINE::RESULT Guides;   ///< Graphics, for showAlignmentGuides()
    };

    /**
     * Compute an alignment-guide snap for a move in progress.  Paints nothing; hand the
     * result to showAlignmentGuides() if and only if the guide beat the caller's other
     * snap candidates.
     *
     * @param aPos       the position the caller would otherwise return; the moving box is
     *                   extrapolated from it, so it must be the same reference the move
     *                   tool's OriginalCursor was captured at
     * @param aSnapRange maximum snap distance in world units
     * @param aGridStep  when set, only offsets that are whole multiples are accepted;
     *                   pass std::nullopt when the caller's position is not grid-aligned
     * @return the snap, or std::nullopt if no guide applies
     */
    std::optional<GUIDE_SNAP> computeAlignmentGuideSnap( const VECTOR2I& aPos, int aSnapRange,
                                                         const std::optional<VECTOR2I>& aGridStep
                                                                 = std::nullopt );

    /// Paint a guide returned by computeAlignmentGuideSnap().
    void showAlignmentGuides( const GUIDE_SNAP& aSnap );

protected:
    /**
     * Per-drag state owned by a subclass, dropped whenever the move context is.
     *
     * A stale flag here exempts the *next* drag from whatever the last one was allowed, and the
     * next drag is usually a symbol.
     */
    virtual void clearMoveState() {}

    /**
     * Called with the extrapolated moving box before each snap is scored, for containers that
     * depend on where the selection currently *is* rather than where the drag started.
     *
     * A logo is picked up somewhere on the page and carried to the corner box, so a container
     * computed once at drag start is the wrong one for the whole gesture.
     */
    virtual void updateDynamicContainers( const BOX2I& aMovingBox ) {}

    void showConstructionGeometry( bool aShow );

    SNAP_MANAGER& getSnapManager() { return m_snapManager; }

    void updateSnapPoint( const TYPED_POINT2I& aPoint );

    /**
     * Enable the anchor debug if permitted and return it
     *
     * Returns nullptr if not permitted by the advancd config
     */
    KIGFX::ANCHOR_DEBUG* enableAndGetAnchorDebug();

    struct MOVE_CONTEXT
    {
        BOX2I    OriginalBBox;
        VECTOR2I OriginalCursor;

        /// Alignment guide outranks item-anchor snapping.  See SetMoveContext().
        bool     PreferGuides = false;
    };

    std::optional<MOVE_CONTEXT> m_moveContext;
    KIGFX::ALIGNMENT_GUIDE_GEOM m_alignGuidePreview;

    std::vector<ANCHOR>     m_anchors;

    TOOL_MANAGER*           m_toolMgr;
    std::optional<VECTOR2I> m_auxAxis;

    int                     m_maskTypes;      // Mask of allowed snap types

    bool                    m_enableSnap;     // Allow snapping to other items on the layers
    bool                    m_enableGrid;     // If true, allow snapping to grid
    bool                    m_enableSnapLine; // Allow drawing lines from snap points
    std::optional<ANCHOR>   m_snapItem;       // Pointer to the currently snapped item in m_anchors
                                              //   (NULL if not snapped)
    VECTOR2I                m_skipPoint;      // When drawing a line, we avoid snapping to the
                                              //   source point
    KIGFX::SNAP_INDICATOR   m_viewSnapPoint;
    KIGFX::ORIGIN_VIEWITEM  m_viewAxis;

    // Manual grid parameters used when no TOOL_MANAGER is provided
    VECTOR2D                m_manualGrid;
    VECTOR2D                m_manualVisibleGrid;
    VECTOR2I                m_manualOrigin;
    bool                    m_manualGridSnapping;

private:
    /// Show construction geometry (if any) on the canvas.
    KIGFX::CONSTRUCTION_GEOM m_constructionGeomPreview;

    /// Manage the construction geometry, snap lines, reference points, etc.
    SNAP_MANAGER m_snapManager;

    /// #VIEW_ITEM for visualising anchor points, if enabled.
    std::unique_ptr<KIGFX::ANCHOR_DEBUG> m_anchorDebug;
};

#endif
