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
#pragma once

#include <optional>
#include <vector>

#include <geometry/seg.h>
#include <math/box2.h>
#include <math/vector2d.h>

/**
 * Pure-geometry engine computing "smart" alignment and spacing snaps for a moving
 * bounding box against a set of neighbor bounding boxes (Figma-style guides).
 *
 * Coordinates are KiCad world units.  The engine has no view, tool or wx
 * dependencies: callers translate items to boxes and interpret the returned
 * offset.  Headless unit-testable.
 */
class ALIGNMENT_GUIDE_ENGINE
{
public:
    struct GAP_BADGE
    {
        VECTOR2I Pos;      ///< World position of the gap midpoint
        int      Gap;      ///< Gap size in world units
        bool     Vertical; ///< True if the gap is measured along Y

        /// This gap is not exactly equal to the others in the run: the grid had no legal
        /// position that would have made them equal, so the snap is a best effort.  Renderers
        /// must mark it, or the badge claims an equality it does not have.
        bool Approximate = false;
    };

    struct RESULT
    {
        VECTOR2I               Offset;      ///< Add to the moving box position to snap
        std::vector<SEG>       Lines;       ///< Guide lines, already at snapped position
        std::vector<GAP_BADGE> Badges;      ///< Equal-spacing distance badges
        std::vector<VECTOR2I>  CenterMarks; ///< Crosshair marks for center snaps
    };

    void SetNeighbors( std::vector<BOX2I> aBoxes ) { m_neighbors = std::move( aBoxes ); }
    void SetContainers( std::vector<BOX2I> aBoxes ) { m_containers = std::move( aBoxes ); }

    void Clear()
    {
        m_neighbors.clear();
        m_containers.clear();
    }

    bool HasInputs() const { return !m_neighbors.empty() || !m_containers.empty(); }

    /**
     * Compute the best snap for aMoving.
     *
     * Precondition: deltas are computed in int, so callers must not pass boxes more
     * than ~2.1 m (INT_MAX nm) apart.  A default-constructed (uninitialised) BOX2I is
     * treated as a real point box at the origin, not as "absent".
     *
     * @param aMoving    the moving selection's bbox at the unsnapped position
     * @param aSnapRange maximum snap distance in world units
     * @param aGridStep  if set, candidates whose offset is not a whole multiple of this
     *                   step are rejected outright.  Callers whose items must stay on a
     *                   grid (schematic pins) pass it.  Three things to know:
     *
     *                   - an *alignment* offset is rejected, never rounded: a guide line
     *                     claiming two edges are level has to be telling the truth.  A spacing
     *                     offset gets a rounded fallback instead, because a schematic's bodies
     *                     routinely sit on half steps and rejecting outright left equal
     *                     spacing firing on 9 motions out of 1819 on a real sheet.  The
     *                     fallback is ranked by the distance to the position it actually
     *                     wants, so rounding cannot make it beat nearer candidates, and its
     *                     badges come back flagged Approximate so nothing claims an equality
     *                     the grid refused;
     *                   - a non-positive component rejects every candidate on that axis,
     *                     so a zero step degrades to "guides don't engage" rather than to
     *                     "every candidate is legal";
     *                   - precondition: aMoving must already be grid-aligned.  A
     *                     whole-multiple offset only keeps the item on grid if it started
     *                     on grid.  The engine is handed the *unsnapped* box and cannot
     *                     check this; KiCad's move tools satisfy it by feeding a
     *                     grid-snapped cursor.
     * @return snap offset + guide graphics, or std::nullopt if nothing in range
     */
    std::optional<RESULT> FindSnap( const BOX2I& aMoving, int aSnapRange,
                                    const std::optional<VECTOR2I>& aGridStep = std::nullopt ) const;

private:
    /// A maximal run of neighbors that overlap or touch along one axis, merged into a
    /// single interval.
    ///
    /// Sort order is not spatial order: sorting boxes by their low edge and pairing
    /// consecutive entries invents gaps that run straight through a third box (a test
    /// point nested in a courtyard, silkscreen under an IC).  Merging first means the
    /// space between two clusters is genuinely empty, and — because touching boxes
    /// merge too — that consecutive clusters are always separated by a strictly
    /// positive gap.
    struct CLUSTER
    {
        int Min;      ///< Merged extent along the axis
        int Max;
        int CrossMin; ///< Merged extent across the axis; positions badges, nothing else
        int CrossMax;
    };

    /// One potential snap position along one axis
    ///
    /// NOTE: named SNAP_CANDIDATE, not CANDIDATE — include/eda_item_flags.h:46
    /// defines a CANDIDATE macro that leaks in through the include chain and
    /// breaks compilation.
    struct SNAP_CANDIDATE
    {
        int    Delta;  ///< Offset along the axis to reach this candidate

        /// Distance used for ranking and the range test, which is *not* always |Delta|.  A
        /// grid-legal fallback is offered at a rounded Delta but must still be judged on how
        /// far the cursor is from the position it actually wants, or rounding a near-miss down
        /// to a Delta of 0 would make it beat every real candidate and pin the item in place.
        int    Dist;
        int    Kind;   ///< KIND_* — drives which guide graphics get built
        size_t N1;     ///< See below — meaning depends on Kind
        size_t N2;
        int    Ord;    ///< Guide ordinate along the axis (KIND_ALIGN), in post-snap coords
        bool   Approx; ///< Delta was rounded onto the grid; exact was unreachable
    };

    // What N1/N2 index, per kind.  There is no single convention; each generator
    // documents its own and buildGraphics must match it:
    //
    //   KIND_ALIGN      both are the same index into m_neighbors
    //   KIND_CONTAINER  both are the same index into m_containers
    //   KIND_EQUAL_GAP  indices into the axis' CLUSTER list.  N1 = far cluster,
    //                   N2 = near cluster (the one the moving box ends up next to).
    //                   The generator emits both directions from one pair and swaps
    //                   the two indices between them, so N1/N2 is not left/right.
    //   KIND_BETWEEN    indices into the axis' CLUSTER list.  N1 = left, N2 = right,
    //                   never swapped.
    enum
    {
        KIND_ALIGN,     ///< Edge/center aligned with a neighbor edge/center
        KIND_EQUAL_GAP, ///< Extends an existing neighbor gap (a->b == b->moving)
        KIND_BETWEEN,   ///< Equal gap on both sides between two neighbors
        KIND_CONTAINER, ///< Centered inside a container box
    };

    /// Neighbors that cross-overlap aMoving, merged along aAxis, ordered ascending.
    std::vector<CLUSTER> buildClusters( const BOX2I& aMoving, int aAxis ) const;

    /// @param aGridStep step for this axis, or 0 for unconstrained.  Exact candidates are
    ///                  emitted regardless; where the step makes an exact spacing unreachable a
    ///                  rounded fallback is emitted after it, flagged Approx.
    void collectAxisCandidates( const BOX2I& aMoving, int aAxis,
                                const std::vector<CLUSTER>& aClusters, int aGridStep,
                                std::vector<SNAP_CANDIDATE>& aOut ) const;

    void buildGraphics( const BOX2I& aSnapped, int aAxis, const SNAP_CANDIDATE& aWinner,
                        const std::vector<CLUSTER>& aClusters, int aGridStep,
                        RESULT& aResult ) const;

    /// A badge on every gap in the run matching aRefGap to within aTolerance -- with three or
    /// more boxes in line, the equality is a property of all the gaps, not just the pair the
    /// snap was computed from.  Pass a tolerance of 0 for an exact snap; anything a rounded
    /// fallback lands on is flagged Approximate per gap.
    void buildGapBadges( const BOX2I& aSnapped, int aAxis, const std::vector<CLUSTER>& aClusters,
                         int aRefGap, int aTolerance, RESULT& aResult ) const;

    /// Guide lines for an alignment snap: one per ordinate the snapped box shares with a
    /// neighbor, each spanning every box sitting on it.
    void buildAlignmentLines( const BOX2I& aSnapped, int aAxis, int aWinnerOrd,
                              RESULT& aResult ) const;

    std::vector<BOX2I> m_neighbors;
    std::vector<BOX2I> m_containers;
};
