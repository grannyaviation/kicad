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

    bool HasCandidates() const { return !m_neighbors.empty() || !m_containers.empty(); }

    /**
     * Compute the best snap for aMoving.
     *
     * @param aMoving    the moving selection's bbox at the unsnapped position
     * @param aSnapRange maximum snap distance in world units
     * @param aGrid      if set, offsets are quantized to multiples of this grid so
     *                   items that started on-grid stay on-grid; quantized offsets
     *                   that leave aSnapRange are dropped
     * @return snap offset + guide graphics, or std::nullopt if nothing in range
     */
    std::optional<RESULT> FindSnap( const BOX2I& aMoving, int aSnapRange,
                                    const std::optional<VECTOR2D>& aGrid = std::nullopt ) const;

private:
    /// One potential snap position along one axis
    struct SNAP_CANDIDATE
    {
        int    Delta;  ///< Offset along the axis to reach this candidate
        int    Kind;   ///< KIND_* — drives which guide graphics get built
        size_t N1;     ///< Index of first involved neighbor (or container)
        size_t N2;     ///< Index of second involved neighbor (equal-gap kinds)
    };

    enum
    {
        KIND_ALIGN,     ///< Edge/center aligned with a neighbor edge/center
        KIND_EQUAL_GAP, ///< Extends an existing neighbor gap (a->b == b->moving)
        KIND_BETWEEN,   ///< Equal gap on both sides between two neighbors
        KIND_CONTAINER, ///< Centered inside a container box
    };

    void collectAxisCandidates( const BOX2I& aMoving, int aAxis,
                                std::vector<SNAP_CANDIDATE>& aOut ) const;

    void buildGraphics( const BOX2I& aSnapped, int aAxis, const SNAP_CANDIDATE& aWinner,
                        RESULT& aResult ) const;

    std::vector<BOX2I> m_neighbors;
    std::vector<BOX2I> m_containers;
};
